# Phase 0 — BLE Link-Drop Seam (2026-07-31)

> **SUPERSEDED 2026-07-31 — do not implement from this document.**
>
> Both deliverables were folded into **Phase 2 (BLE-HAL foundation)** of
> [`PLAN_FREEZE_HARDENING_2026-07-31.md`](PLAN_FREEZE_HARDENING_2026-07-31.md), which
> is the live plan. Read that instead; this file is kept only for the reasoning trail.
>
> Two things here are **out of date** and were corrected in the fold:
>
> - **The seam signature.** This document specifies `disconnect(uint8_t reason)`. The
>   live plan specifies `disconnect(uint16_t handle)` with 0x13 hard-coded and *no*
>   reason parameter. Both stacks were read to settle it: Bluefruit's
>   `disconnect(uint16_t conn_hdl)` (`bluefruit.h:171`) has no reason parameter at all
>   — it delegates to `sd_ble_gap_disconnect(_conn_hdl, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION)`
>   (`BLEConnection.cpp:206`) — and NimBLE already *defaults* its reason to 0x13
>   (`NimBLEServer.h:66`). A handle is what both stacks genuinely take, and Phase 3's
>   admission policy needs to drop a *specific* link, not "the current one".
> - **The phase numbering.** References below to "Phases 2, 4 and 5" are the earlier
>   five-phase draft. The live plan has four phases; the seam is in Phase 2.
>
> Deliverable 2 (the ESP32 disconnect-reason truncation fix) and the deferred
> `OdDiscReason` classifier carried over unchanged, and now live in the live plan's
> Phase 2 seam section.

The foundational seam for [`PLAN_FREEZE_HARDENING_2026-07-31.md`](PLAN_FREEZE_HARDENING_2026-07-31.md).
Phases 2, 4 and 5 all need to **drop a BLE link from the loop task**, and none can
today. This phase adds that one capability, plus the minimal fix to stop the
disconnect-reason log from lying.

No wire change (a disconnect reason is an HCI byte, not an app-protocol field).

## Scope decision — why this is small

An earlier draft of this phase also normalized the *inbound* disconnect reason into
a six-value enum. That was cut after checking what actually consumes it: **nothing
in Phases 2–5 branches on why a link dropped.**

- Phase 2 (owner token) releases and tears down on *any* disconnect.
- Phase 3 (abort) runs the same teardown regardless of reason.
- Phases 4 and 5 *initiate* the drop, and their authoritative "did I cause this" is
  a `*DropPending` flag (see [Deliverable 1](#deliverable-1)), not the reason byte.

So the normalized reason would feed a log line and nothing else. A classification
layer nothing consumes is not worth its surface. It is deferred to
[Deferred](#deferred-until-something-consumes-it) — a small header and a `switch`,
cheap to add the day a phase branches on a reason (repeated-MIC-failure handling is
the likely first customer).

What is **not** deferred is the outbound drop, and the one honest bug in the
current reason handling.

## Deliverable 1 — `BleTransport::disconnect(uint8_t reason)`

Add to the abstraction ([ble_transport.h](../src/ble_transport.h)); implement per
target; call **only from the loop task**.

- **ESP32:** `s_server->disconnect(s_connHandle, reason)` when `s_connHandle !=
  BLE_HS_CONN_HANDLE_NONE`. Return the call's bool; log WARN on failure.
- **nRF:** `Bluefruit.disconnect(s_connHandle)` when `s_connHandle !=
  BLE_CONN_HANDLE_INVALID`; keep `restartOnDisconnect(true)` (unlike the DFU path at
  [device_control.cpp:857](../src/device_control.cpp), which disables it).

**Reason to send: `0x13`** (`BLE_ERR_REM_USER_CONN_TERM` /
`BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION`, identical in both stacks and in the
Core Spec's legal `HCI_Disconnect` allowlist). **Do NOT send `0x09`** (`CONN_LIMIT`):
it is not a legal host-disconnect reason, so the controller rejects it (0x12) while
the code looks like it worked and the gatecrasher stays connected. Neither constant
exists in `src/` today; add one named constant with a comment carrying the 0x09
trap.

**Loop-task only.** The call is made from a loop-serviced helper (or inline in the
existing `serviceBle*` helpers), never a stack callback — a callback that severs its
own link mid-dispatch is exactly the class of bug `#132` removed. The phases that
request a drop do so by raising a `*DropPending` flag; the loop services it. That
flag — not any reason byte read back afterward — is the authoritative record of a
self-initiated drop.

## Deliverable 2 — stop the disconnect-reason log from lying (ESP32)

Not a new feature; a correctness fix to what is already logged at
[main.cpp:472](../src/main.cpp). Today ESP32 stores the reason wrong:

```cpp
// ble_transport_esp32.cpp:35,99
static volatile uint8_t s_disconnectReason = 0;
...
s_disconnectReason = (uint8_t)reason;   // int -> uint8_t: truncates
```

NimBLE's `onDisconnect(int reason)` uses two ranges: HCI reasons wrapped as
`BLE_HS_ERR_HCI_BASE + code` (`0x200 + code`), and host-layer `BLE_HS_E*` codes in
`1..31`. The `uint8_t` cast keeps only the low byte, so:

- an HCI reason survives by luck (`0x213 & 0xFF == 0x13`), but
- a host code like `BLE_HS_ENOTCONN = 7` truncates to `0x07`, which reads back as
  the unrelated HCI code "memory capacity exceeded". The stored byte is ambiguous
  and the log can name the wrong reason.

nRF is unaffected — it stores a raw HCI `uint8_t` from the SoftDevice with no
wrapping.

**Fix (ESP32 only, ~3 lines):** widen `s_disconnectReason` to `uint16_t` so the
`0x200` offset survives capture, and log the raw value as-is:

```cpp
static volatile uint16_t s_disconnectReason = 0;
...
s_disconnectReason = (uint16_t)reason;   // keep the full value, no truncation
```

`takeDisconnectedEvent`'s out-param widens to `uint16_t*`
([ble_transport.h:81](../src/ble_transport.h), one caller at
[main.cpp:471](../src/main.cpp)), and the log line becomes
`"Disconnect reason: 0x%03X"` so a wrapped HCI reason (`0x213`) and a host reason
(`0x007`) are visibly distinct rather than colliding on `0x13`/`0x07`. No enum, no
classifier, no interpretation — just stop discarding half the value.

## Files touched

| File | Change |
|---|---|
| `src/ble_transport.h` | add `disconnect(uint8_t)` + the `0x13`/`0x09` reason constant & comment; widen `takeDisconnectedEvent`'s reason out-param to `uint16_t*` |
| `src/ble_transport_nrf.cpp` | implement `disconnect()`; reason storage unchanged (already a raw HCI byte, widened only to match the signature) |
| `src/ble_transport_esp32.cpp` | implement `disconnect()`; widen `s_disconnectReason` to `uint16_t`, drop the truncating cast |
| `src/main.cpp` | update the one `takeDisconnectedEvent` caller + its log line |

No new file, no host test (nothing here is pure logic worth a standalone test — the
drop needs a board; the widening is a type change verified by build + bench log).

## Verification

- **Build** all envs.
- **Bench (closes the phase):** on nRF and ESP32, call `disconnect(0x13)` from the
  loop task and confirm the link actually drops (the `0x09` trap means "it
  compiled" is not enough — watch for the disconnect on a scanner or the client
  side). Confirm a real client disconnect logs a sensible reason, and that a
  NimBLE host-layer reason now logs as `0x0xx` rather than masquerading as an HCI
  code.

## Deferred until something consumes it

The normalized inbound reason (a `src/ble_disc_reason.h` with an `OdDiscReason`
enum — `SUCCESS / REMOTE / LOCAL / TIMEOUT / MIC_FAILURE / OTHER` — a pure
`od_disc_classify(uint8_t hci)` switch, the ESP32 `0x200`-offset normalization, and
a host test) is **not built here**. It is deferred until a phase branches on a
reason rather than just logging it. The most likely trigger is MIC-failure-driven
behaviour (0x3D signals encryption desync — the failure class this whole effort
targets), e.g. forcing re-auth after repeated MIC failures. When that lands, the
enum is a small header and a `switch`; the `uint16_t` raw value this phase already
preserves is exactly the input the classifier needs, so nothing here has to be
redone.

## Out of scope

- Any behaviour that *acts* on a link drop — that is Phases 2/4/5. Phase 0 only
  makes the drop possible and the reason log honest.
- LAN disconnects; the owner token (Phase 2) handles LAN, and TCP has no HCI reason
  to preserve.
