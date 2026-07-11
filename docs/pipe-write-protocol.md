# PIPE_WRITE Wire Protocol

Sliding-window, selectively-acknowledged image transfer for OpenDisplay panels.
Opcodes **0x0080–0x0082**. Introduced in commit `bcfad8b`.

This document walks the protocol end to end: **negotiation → data transfer →
acknowledgement → error recovery → completion**. It is written against the
firmware in [src/display_service.cpp](../src/display_service.cpp),
[src/structs.h](../src/structs.h), [src/communication.cpp](../src/communication.cpp),
and [src/esp32_ble_callbacks.h](../src/esp32_ble_callbacks.h).

---

## 1. Overview & motivation

The legacy image path (`0x70/0x71/0x72`) is strictly in-order and stop-and-wait:
the client sends a chunk, waits for an ACK, sends the next. Over BLE, that
round-trip latency dominates transfer time.

PIPE_WRITE replaces the stop-and-wait loop with a **sliding window**. The client
may have up to *W* frames outstanding (unacknowledged) at once, and the device
acknowledges with a **QUIC-style selective ACK (SACK)** that reports the highest
sequence number seen plus a bitmask of which earlier frames arrived. This lets a
single lost frame be retransmitted without stalling the whole stream.

Key design point: **data streams directly to the panel controller — there is no
framebuffer.** Bytes for frame *k* are written to the display IC as soon as frame
*k* is accepted in order. Because the controller stream cannot skip a hole, a
small **reorder queue** holds out-of-order frames until the missing one arrives.

The legacy `0x70/0x71/0x72` (and partial `0x76`) paths are byte-for-byte
unchanged. A new client probes with `0x0080`; if it goes unanswered (old
firmware) the client falls back to legacy. Old clients are unaffected.

### Opcode family

| Opcode   | Name              | Direction     | Purpose                              |
|----------|-------------------|---------------|--------------------------------------|
| `0x0080` | PIPE_WRITE_START  | client→device | Negotiate window / cadence / frame   |
| `0x0081` | PIPE_WRITE_DATA   | client→device | Carry one sequenced payload frame     |
| `0x0082` | PIPE_WRITE_END    | client→device | Finalize, refresh, commit etag        |

The device replies on the same notify channel with status-prefixed responses
(`0x00…` success / SACK, `0xFF…` NACK) described below.

### Framing convention

The 2-byte **big-endian** opcode is stripped by the dispatcher
([communication.cpp:621-635](../src/communication.cpp)) before the handler runs.
Inside a handler, `data[0]` is therefore the first *post-opcode* byte and `len`
is the post-opcode payload length. All multi-byte fields **in the transfer body
are little-endian** — except the END etag, which mirrors legacy and is big-endian
(see §6).

### Negotiated constants

Defined in [structs.h:108-124](../src/structs.h). Two profiles: the default and a
reduced profile (`PIPE_SMALL_DRAM_WINDOW`) used only by the classic-ESP32
`env:esp32-N4` build, whose static DRAM cannot hold the full queue.

| Constant                 | Default | Small-DRAM | Meaning                                    |
|--------------------------|---------|-----------|--------------------------------------------|
| `PIPE_VERSION`           | `0x01`  | `0x01`    | Protocol version                           |
| `PIPE_MAX_W`             | 32      | 16        | Device max window                          |
| `PIPE_MAX_N`             | 32      | 16        | Device max ACK cadence                     |
| `PIPE_MAX_FRAME`         | 244     | 244       | Device max frame size (HA ATT write ceiling)|
| `PIPE_ACK_MASK_BITS`     | 32      | 32        | SACK bitmask width                         |
| `PIPE_REORDER_SLOTS`     | 33      | 17        | Reorder queue depth (= max window + 1)     |
| `PIPE_REORDER_SLOT_SIZE` | 248     | 248       | Per-slot buffer bytes                      |
| `PIPE_FLAG_COMPRESSED`   | `0x01`  | `0x01`    | START flag bit: payload is a zlib stream   |

---

## 2. Negotiation — PIPE_WRITE_START (0x0080)

One round trip establishes the whole session. Handler:
[handlePipeWriteStart, display_service.cpp:2057-2141](../src/display_service.cpp).

A new START **aborts any in-flight transfer** (of any family) and calls
`resetPipeWriteState()` before parsing.

### 2.1 Request (client → device)

10 bytes, little-endian, after the `00 80` opcode:

| Offset | Field              | Size | Notes                                             |
|--------|--------------------|------|---------------------------------------------------|
| 0      | `ver`              | 1    | Must equal `PIPE_VERSION` (`0x01`)                |
| 1      | `flags`            | 1    | bit0 = `PIPE_FLAG_COMPRESSED` (zlib); other bits reserved |
| 2      | `req_w`            | 1    | Proposed window size                              |
| 3      | `req_n`            | 1    | Proposed ACK cadence (ack every N)                |
| 4–5    | `client_max_frame` | 2 LE | Proposed max frame size                           |
| 6–9    | `total_size`       | 4 LE | Decompressed panel byte total                     |

The guard is `if (len < 10)` — trailing bytes are tolerated for future fields.

> **Historical bug (fixed in `a39082e`).** The handler is dispatched as
> `handlePipeWriteStart(data+2, len-2)`, so `len` is the 10-byte post-opcode
> length. The original guard required `len >= 12` (the full on-wire frame
> including the opcode), so **every valid request was rejected** with
> `sendPipeStartNack(0x01)` and clients silently fell back to legacy on every
> upload. The parse itself was correct; only the guard was wrong.

### 2.2 Negotiation rule (min of both sides)

The device computes effective values ([display_service.cpp:2092-2098](../src/display_service.cpp)):

- `W_eff  = clamp(min(req_w, PIPE_MAX_W), 1..)` — additionally clamped to ≤32
  when encryption is active (the ACK mask is 32 bits wide).
- `N_eff  = min(clamp(min(req_n, PIPE_MAX_N), 1..), W_eff)` — cadence never
  exceeds the window.
- `frame_eff = min(client_max_frame, PIPE_MAX_FRAME)` (≤ 244).

`total_size` must **exactly** match the device-computed geometry
(`directWriteComputeGeometry`), else NACK `0x03`.

### 2.3 Response (device → client)

8 bytes ([display_service.cpp:2126-2128](../src/display_service.cpp)):

```
00 80  VER  MAX_W  MAX_N  FRAME_LO FRAME_HI  FLAGS
```

| Offset | Value                         | Meaning                                    |
|--------|-------------------------------|--------------------------------------------|
| 0–1    | `00 80`                       | Success + opcode echo                       |
| 2      | `0x01`                        | Device protocol version                     |
| 3      | `PIPE_MAX_W`                  | Device max window                           |
| 4      | `PIPE_MAX_N`                  | Device max ACK cadence                      |
| 5–6    | `PIPE_MAX_FRAME` (244 → `F4 00`) | Device max frame, LE                     |
| 7      | flags — **bit0 = 1**          | Device buffers out-of-order (selective repeat) |

The response returns device **maxima**, not the effective values. The client
applies the same `min` rule to arrive at the identical `W_eff / N_eff /
frame_eff`. Both sides now agree without a second round trip.

> The response is sent **before** the panel is powered up
> ([display_service.cpp:2117-2140](../src/display_service.cpp)) so that slow
> panel init cannot exceed the client's 2-second probe timeout.

### 2.4 START NACK

`sendPipeStartNack(err)` → 4 bytes `FF 80 <err> 00`. No teardown is needed (the
geometry check is pure config math before any hardware is touched).

| `err`  | Cause                                                     |
|--------|-----------------------------------------------------------|
| `0x01` | Bad length (`len < 10`) or version mismatch               |
| `0x02` | Unsupported flag bits set                                 |
| `0x03` | `total_size` disagrees with device geometry              |

---

## 3. Data transfer — PIPE_WRITE_DATA (0x0081)

Handler: [handlePipeWriteData, display_service.cpp:2143-2229](../src/display_service.cpp).

### 3.1 Frame layout

After the `00 81` opcode:

| Offset | Field     | Size   | Notes                                    |
|--------|-----------|--------|------------------------------------------|
| 0      | `seq`     | 1      | Rolling sequence number, wraps mod 256    |
| 1…     | `payload` | ≤243   | Controller bytes (or zlib stream)         |

`plen = len - 1`. A frame carries at most `frame_eff - 1` payload bytes (the seq
consumes one), so at the 244-byte cap the payload is ≤243 bytes.

### 3.2 Sequence numbers and the window

- The transfer starts at `expected_seq = 0` and increments per in-order accept.
- `seq` is a `uint8_t`; it wraps 255 → 0. Distances use unsigned 8-bit
  subtraction so wrap is automatic:
  - `fwd  = (uint8_t)(seq - expected_seq)` — `0` = exactly in order; `1..W-1` =
    ahead, within window.
  - `back = (uint8_t)(expected_seq - seq)` — `≥1` = at/below expected (duplicate
    or stale).
- The window width is `W = W_eff`. Because a live window spans at most *W* < 33
  distinct seqs, `seq % PIPE_REORDER_SLOTS` indexes the reorder queue without
  collisions even across the mod-256 wrap.

### 3.3 The three cases

**(a) In order — `fwd == 0`** ([display_service.cpp:2155-2184](../src/display_service.cpp)):

1. `pipeConsumePayload()` streams the bytes straight to the panel controller
   (same machinery as legacy `0x71`: raw write, or zlib inflate, or gray4 plane
   split). A failure NACKs (`0x02` compressed / `0x03` uncompressed).
2. `expected_seq++`, counters advance, `highest_seen` updates.
3. **Drain the queue:** while the slot for the new `expected_seq` holds a
   matching frame, consume it, free the slot, `queued_count--`, and advance
   `expected_seq` again. This is how the stream catches up past a filled hole.
4. If the queue is now empty, `gap_open = false`.
5. Cadence: if `frames_since_ack >= N_eff`, send a SACK (§4).

**(b) Ahead, in window — `0 < fwd < W`** ([display_service.cpp:2187-2211](../src/display_service.cpp)):

This is the **pause point** — nothing past the hole reaches the controller. The
frame is `memcpy`'d into `pipeReorder[seq % SLOTS]`, `queued_count++`. If this
opens a new gap (`gap_open` was false), a SACK is sent **immediately** so the
sender learns which frame is missing (fast retransmit). While the gap stays open,
further out-of-order arrivals are rate-limited to one SACK per `N_eff` arrivals.
Queue overflow (`queued_count >= PIPE_REORDER_SLOTS`) NACKs `0x03` — a protocol
violation the sender's window rule should make impossible.

**(c) Duplicate / stale — `back <= W`** ([display_service.cpp:2216-2224](../src/display_service.cpp)):

The frame is discarded but a rate-limited SACK is sent so the sender re-learns
the receiver's position. Anything outside the window on both sides (`fwd >= W`
and `back > W`) is a protocol violation → NACK `0x04`.

### 3.4 ESP32 ingest ring (transport, not protocol)

On ESP32 the BLE callback copies each command into a lock-free SPSC ring
(`COMMAND_QUEUE_SIZE = 33`, `MAX_COMMAND_SIZE = 256`) with acquire/release
atomics; the main loop drains up to 33 per pass and flushes responses **between**
commands so small ACK cadences can't overflow the response ring. Sized to hold a
full W=32 window plus END across an SPI stall.

---

## 4. Acknowledgement — QUIC-style SACK

Builder: [pipeBuildAckPayload, display_service.cpp:1967-1979](../src/display_service.cpp).

### 4.1 Format

A data ACK is 7 bytes:

```
00 81  highest_seen  mask[0] mask[1] mask[2] mask[3]
```

- Byte 2 `highest_seen` — the highest seq received (accepted **or** queued),
  mod 256. If nothing has arrived yet, it reports `expected_seq - 1`.
  `highest_seen` is itself implicitly acknowledged.
- Bytes 3–6 — a **32-bit mask, little-endian**. Bit *i* (LSB first, i = 0..31)
  means **chunk `(highest_seen - 1 - i)` was received**. So bit0 = `hs−1`,
  bit1 = `hs−2`, …, bit31 = `hs−32`.

"Received" means either accepted in the in-order prefix *or* currently held in
the reorder queue ([pipeChunkReceived, display_service.cpp:1957-1963](../src/display_service.cpp)).
The accepted-prefix depth is bounded by `received_count` so the first 32 chunks
of a transfer never set phantom bits from mod-256 wrap. A zeroed bit is the
sender's cue to retransmit that seq.

### 4.2 When a SACK is sent

| Trigger                        | Behaviour                                          |
|--------------------------------|----------------------------------------------------|
| Cadence                        | Every `N_eff` in-order accepts                      |
| Gap opens                      | **Immediately** (fast retransmit)                   |
| Out-of-order / duplicate while gap open | Rate-limited: one per `N_eff` such arrivals |
| END / auto-complete            | Tail flush before the final result                  |

`sendPipeAck()` resets both the cadence counter (`frames_since_ack`) and the
gap-ACK rate-limit counter (`ooo_acks_since_gap`).

### 4.3 Worked example

Window 8, cadence 4. Frames 0,1,2 arrive, then 3 is lost, then 4,5 arrive:

- After 0,1,2 in order: `expected_seq = 3`. (No ACK yet — only 3 accepts, cadence
  is 4.)
- Frame 4 arrives, `fwd = 1` → queued, gap opens → **immediate SACK**
  `highest_seen = 4`, mask bit for seq 3 is **0** (missing), bit for seq 2 is 1.
  The sender sees the hole at 3.
- Frame 5 arrives, `fwd = 2` → queued; rate-limited, no new ACK yet.
- Sender retransmits 3. It arrives `fwd = 0` → written, then the drain loop
  writes queued 4 and 5, `expected_seq = 6`. The next cadence SACK reports
  `highest_seen = 5` with a full mask.

---

## 5. Error recovery

### 5.1 Data NACK — all fatal

`sendPipeNack(err)` → 8 bytes `FF 81 <err> highest_seen mask[0..3]`
([display_service.cpp:1999-2005](../src/display_service.cpp)). The SACK tail is
built from state **before** any teardown so the reported position stays
consistent.

Every `0x81` NACK is **fatal**:

1. `pipeState.error = true`.
2. `cleanupDirectWriteState(true)` runs the **same** hardware cleanup as the
   legacy mid-stream `0xFF 71` failure: sleep a powered controller cleanly, cut
   power, resume touch.
3. `pipeState` and the reorder queue are **deliberately not reset**, so
   subsequent `0x0081` frames are silently discarded until the next `0x0080`
   START or a BLE disconnect.

| `err`  | Cause                                                              |
|--------|--------------------------------------------------------------------|
| `0x02` | Compressed (zlib) consume failure                                  |
| `0x03` | Uncompressed consume failure, over-size frame, or reorder overflow |
| `0x04` | Out of window on both sides (protocol violation)                   |

### 5.2 Loss recovery flow (the normal case)

Loss recovery is **not** a NACK — it is the SACK mechanism:

1. A frame lands ahead of the hole → device queues it and sends an immediate SACK
   whose zeroed mask bits name the missing seqs.
2. The sender retransmits exactly those seqs.
3. The retransmit arrives in order → written, and the contiguous run of queued
   successors drains → the stream resumes.

NACKs are reserved for unrecoverable conditions (bad payload, protocol
violation), not ordinary packet loss.

### 5.3 Stuck-transfer timeout

The main loop enforces a 15-minute (`900000 ms`) ceiling on a stalled
direct-write / pipe transfer ([main.cpp](../src/main.cpp)), after which the panel
hardware is released even if no END or NACK ever arrives.

---

## 6. Completion — PIPE_WRITE_END (0x0082)

Handler: [handlePipeWriteEnd, display_service.cpp:2231-2266](../src/display_service.cpp).
END shares the legacy `0x72` finalizer `directWriteFinishAndRefresh(data, len, 0x82)`.

### 6.1 Payload (after `00 82`)

| Offset | Field         | Size   | Notes                                              |
|--------|---------------|--------|----------------------------------------------------|
| 0      | `refresh_mode`| 1      | `1` = fast refresh; anything else = full refresh   |
| 1–4    | `etag`        | 4 **BE** | New etag, big-endian (`parse_be_u32`), optional  |

The etag is big-endian to match legacy `0x72` exactly. A nonzero etag is
committed on a successful refresh; a zero/absent etag clears `displayed_etag` so a
later partial update falls back cleanly on an etag mismatch.

### 6.2 Flow

1. Not active → `FF 82`. Already in fatal error → `FF 82` + defensive cleanup +
   reset.
2. **Tail-flush SACK** (`sendPipeAck()`) so the sender sees the final receiver
   state.
3. **Completeness check:** if the reorder queue is non-empty (`queued_count > 0`),
   or (uncompressed) fewer than `total_size` bytes were written, the transfer is
   incomplete → `FF 82` + `cleanupDirectWriteState(true)` + reset. (Compressed
   incompleteness surfaces as a zlib-flush NACK inside the shared finalizer.)
4. Otherwise finalize: success `00 82`, panel refresh, then `00 73` on refresh
   success or `00 74` on refresh timeout, then `resetPipeWriteState()`.

### 6.3 Auto-complete (uncompressed only)

When an in-order accept pushes `directWriteBytesWritten >= total_size`, the device
finalizes **without** waiting for an END frame
([display_service.cpp:2177-2182](../src/display_service.cpp)): it flushes a final
SACK, calls `directWriteFinishAndRefresh(nullptr, 0, 0x82)` (an unsolicited
`00 82` + full refresh, no etag), and resets. This mirrors the legacy
auto-finish behaviour.

---

## 7. End-to-end sequence

```
Client                                   Device
  |  00 80  ver flags W N frame total  -->|   negotiate
  |<-- 00 80  01 MAXW MAXN FRAME flags     |   grant (device maxima)
  |                                        |
  |  00 81  00 <payload>              ---->|   seq 0  -> panel
  |  00 81  01 <payload>              ---->|   seq 1  -> panel
  |  00 81  02 <payload>              ---->|   seq 2  -> panel
  |  00 81  03 <payload>   (LOST)      -x  |
  |  00 81  04 <payload>              ---->|   queued; gap opens
  |<-- 00 81  04  mask(bit@3=0)             |   immediate SACK
  |  00 81  03 <payload>  (retransmit)---->|   seq 3 -> panel, drains 4
  |             ... cadence SACKs ...       |
  |  00 82  mode etag                 ---->|   finalize
  |<-- 00 81  hs mask   (tail flush)        |
  |<-- 00 82                                |   success
  |<-- 00 73                                |   refresh complete
```

---

## 8. Compatibility

- Legacy `0x70/0x71/0x72` and partial `0x76` paths are **byte-identical** — old
  clients are unaffected.
- A new client that sends `0x0080` and gets no response within its 2 s probe
  assumes legacy-only firmware and falls back to `0x70`.
- Pipe data ACKs at `highest_seen` = `0xFE`/`0xFF` stay **encrypted**: the
  `sendResponse` encryption-skip heuristic (which normally treats a `0xFE`/`0xFF`
  status byte as an unencrypted response) is scoped to exclude the 7-byte
  `00 81 …` ACK shape ([communication.cpp:170-182](../src/communication.cpp)).
```
