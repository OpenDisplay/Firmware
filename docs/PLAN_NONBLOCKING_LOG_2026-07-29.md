# Plan — make `od_log` non-blocking on nRF via TinyUSB, short-circuited on ESP32

**Date:** 2026-07-29
**Branch:** `fix/loop-hang-3`
**Supersedes:** the reverted `a84e512` / `1fc524b`, and the check-guarded draft of this plan
**Related:** [`FINDINGS_NRF_BLOCKING_CALLS_2026-07-29.md`](FINDINGS_NRF_BLOCKING_CALLS_2026-07-29.md)
§C (this path), §D (starvation), §E4 (stack pressure)

**Architecture note.** An earlier version of this plan kept writing through Arduino
`Stream::write` and *prevented* the hang with a capacity check. That check was load-bearing for
safety, so it needed a mutex to be correct, static allocation for the mutex, a context-aware
budget, a level-aware budget, a drop backoff, tick deadlines, and short-write rules — 521 lines,
six review rounds, and three defects found in the scaffolding itself.

This version writes through `tud_cdc_write()` instead. **Blocking becomes impossible by
construction rather than prevented by a correct check.** Everything that remains survives for
*output quality*, so a bug in it costs a mangled log line, not a bricked tag. That reordering of
what is load-bearing is the whole point of the rewrite.

---

## The bug

`Adafruit_USBD_CDC::write()` spins with no timeout and no iteration cap while DTR is asserted and
the host is not draining:

```c
// Adafruit_TinyUSB_Arduino/src/arduino/Adafruit_USBD_CDC.cpp:218
size_t Adafruit_USBD_CDC::write(const uint8_t *buffer, size_t size) {
  size_t remain = size;
  while (remain && tud_cdc_n_connected(_instance)) {
    size_t wrcount = tud_cdc_n_write(_instance, buffer, remain);
    remain -= wrcount;  buffer += wrcount;
    if (remain) { yield(); }
  }
```

`od_log` writes through it ([`src/od_log.cpp:40`](../src/od_log.cpp)), so a terminal that stays
open but stops reading wedges the logging task. On `loop()` that is fatal: nRF has no watchdog
and every fault handler in the linked image is `b .`, so `epdSessionTick()` stops, the keep-alive
never expires, and the tag goes silent while the link stays up.

**The wrapper is the only blocking layer.** Everything beneath it already reports backpressure
instead of waiting on it:

```c
// class/cdc/cdc_device.c:168 -- writes what fits, returns the count. No loop.
uint32_t tud_cdc_n_write(uint8_t itf, void const* buffer, uint32_t bufsize) {
  uint16_t ret = tu_fifo_write_n(&_cdcd_itf[itf].tx_ff, buffer, bufsize);
  if (tu_fifo_count(&p_cdc->tx_ff) >= BULK_PACKET_SIZE) tud_cdc_n_write_flush(itf);
  return ret;
}

// :182 -- endpoint busy? return 0. Never waits.
uint32_t tud_cdc_n_write_flush(uint8_t itf) {
  TU_VERIFY(tud_ready(), 0);
  TU_VERIFY(usbd_edpt_claim(rhport, p_cdc->ep_in), 0);
  ...
}
```

The Arduino wrapper takes a non-blocking primitive that already says "I only took 40 of your 210
bytes" and retries it forever instead of reporting it.

**Necessary conditions** — only one host state hangs:

| Host state | `tud_cdc_n_connected()` | Hangs? |
|---|---|---|
| Unplugged / not enumerated | false (`tud_mounted()`) | No |
| Bus suspended | false (`tud_suspended()`) | No |
| Port closed, DTR low | false | No — and the FIFO is *overwritable*, so writes never fill |
| **Enumerated, not suspended, DTR high, app not reading** | true | **Yes** |

DTR does two independent things: it keeps the loop's continue condition true, *and* `cdcd_init`
flips the TX FIFO from overwritable to strict on it (`cdc_device.c:394`,
`tu_fifo_set_overwritable(&tx_ff, !dtr)`). Both halves come from the same bit, which is why an
unattended tag never hits this.

This is **not** the freeze presently under investigation — the audit ranks TWIM I²C spins (§A1)
and SoftDevice flash `portMAX_DELAY` (§A2) above it, with a watchdog as the remedy for both. It
removes one confirmed unbounded wait and makes log gaps self-describing.

---

## Design

### 1. One backend function — the only place the targets differ

```c
// All-or-nothing. Returns false without writing anything if the port cannot take
// the whole record. NEVER blocks on nRF: tud_cdc_write() is a bare tu_fifo_write_n.
static bool od_port_write(const uint8_t *b, size_t n) {
#ifdef TARGET_ESP32
    return s_port->write(b, n) == n;      // Stream, as today; see §6
#else
    return tud_cdc_write(b, n) == n;      // caller has already reserved capacity
#endif
}
```

Contained in one function rather than spread through `od_log` as conditionals. nRF loses the
`Stream*` abstraction; that is the price of the guarantee, and it is paid in exactly one place.

`#include <Adafruit_TinyUSB.h>` already compiles in this project
([`src/utilities/nrf52840_reformat/main.cpp:22`](../src/utilities/nrf52840_reformat/main.cpp)),
and `tud_cdc_write` / `tud_cdc_write_available` / `tud_cdc_write_flush` are instance-0 inlines at
`class/cdc/cdc_device.h:219-236`.

### 2. Capacity reservation — for line integrity, not for safety

Check `tud_cdc_write_available() >= total` once, then issue the record's writes. Because the
reservation and the writes happen **under our mutex**, no producer of ours can consume the space
in between, so every write is guaranteed to take its bytes in full. Partial records are therefore
impossible on nRF without an assembly buffer.

| Case | Writes |
|---|---|
| Untagged | `write(text, len)`, `write("\r\n", 2)` |
| Tagged | `write(text, tagAt)`, `write(tag, tagLen)`, `write(text + tagAt, len - tagAt)`, `write("\r\n", 2)` |

Note what this check is **not** doing any more: it is not what keeps the firmware alive. If it
were wrong, the consequence is a truncated line, because `tud_cdc_write()` returns short rather
than spinning.

Byte-identity is exact for any record ≤232 bytes. `len = min(strlen(text), 232)`; the longest
current caller is [`command_queue.cpp:82`](../src/command_queue.cpp) `char line[192]` plus a ≤20
char header = 211.

### 3. The mutex — line serialisation only

TinyUSB's FIFO is already multi-writer safe: `CFG_TUSB_OS = OPT_OS_FREERTOS`
(`arduino/ports/nrf/tusb_config_nrf.h:43`) ⇒ `CFG_FIFO_MUTEX` (`common/tusb_fifo.h:48`), and
`cdcd_init` gives `tx_ff` a write mutex (`cdc_device.c:252`). So a **single** `tud_cdc_write()`
is atomic against other writers.

Our mutex exists because the *reservation-plus-N-writes span* is not. Without it, a producer can
consume the reserved space between our check and our third write, truncating the record. It is a
`xSemaphoreCreateMutexStatic()` with a file-scope `StaticSemaphore_t`
(`configSUPPORT_STATIC_ALLOCATION`, `FreeRTOSConfig.h:72`) — static purely to avoid an
allocation-failure branch, not because failure is now dangerous.

**On lock timeout: drop and count.** A dropped line reads better than a mangled one. This is
now a quality choice with no safety consequence, which is why it needs no fail-closed reasoning.

### 4. The 20 ms bounded wait — delivery quality

```c
const TickType_t start  = xTaskGetTickCount();     // ONCE, before xSemaphoreTake
const TickType_t budget = pdMS_TO_TICKS(od_budget_ms());
#define OD_EXPIRED() ((TickType_t)(xTaskGetTickCount() - start) >= budget)

for (;;) {
    if (tud_cdc_write_available() >= need) break;   // ORDER MATTERS -- see below
    if (OD_EXPIRED()) { drop(); return; }
    vTaskDelay(1);
}
```

Without a wait, a single-shot check fails constantly on a *healthy* host: the FIFO is 256 bytes,
the longest line is ~210, and one image push emits ~300 lines back to back. The wait is what
keeps the frame dumps.

`start` is stamped **once, before the mutex take**, and the take's timeout is the remaining
budget — not a fresh one — or worst case per line is 2× budget.

**Ticks, not `millis()`.** `millis()` on nRF is `tick2ms(xTaskGetTickCount())` with
`tick2ms(t) = (uint64_t)t * 1000 / configTICK_RATE_HZ` (`cores/nRF5/rtos.h:65`) at 1024 Hz, so it
wraps at **4,194,303,999**, not `2³²`. The `(int32_t)(a - b)` idiom is unsound on it; the tick
counter does wrap modulo `2³²`, so unsigned tick subtraction is correct by construction.

**The room check must precede the expiry check**, so a 0 ms budget degenerates to "try once, then
discard" rather than "discard without trying". Reversing them makes off-loop logging drop 100%.

**`vTaskDelay(1)`, not `delay(1)`.** `delay()` returns *without* `vTaskDelay` when the CDC flush
spans a tick (`cores/nRF5/delay.c:33-48`: `if (flush_tick >= ticks) return;`, and `ms2tick(1)` is
1 tick). Under load that degrades into a busy-spin at priority 2 that never yields to `loop()`;
time slicing is disabled (`FreeRTOSConfig.h:68`).

### 5. Budget: 0 off-loop, backoff on stall

```c
static uint32_t od_budget_ms(void) {
    // NULL-check first: without it an uncaptured handle makes the inequality true
    // for every caller and silently puts everything in try-once-then-drop mode.
    if (s_loopTask != NULL && xTaskGetCurrentTaskHandle() != s_loopTask) return 0;
    if (s_loopConsecutiveDrops >= 3) return 0;
    return 20;
}
```

**0 ms off-loop.** Waiting above `loop()` is priority inversion even though it is no longer
dangerous. See the inventory below for exactly which sites this covers.

**Backoff.** Waiting is a bet that pays off on transient fullness (~1 ms drain) and loses on a
real stall. Without it a stalled port costs 20 ms on *every* line — ~300 lines per push ≈ **6 s of
added `loop()` latency**, enough to starve BLE ACK draining and time out a transfer. Three
consecutive drops presumes a stall and zeroes the budget; any successful write resets it.

Fed by **loop-context drops only** — it only gates the loop budget, and a global counter is
poisoned across contexts on a healthy host: a `-debug` frame burst drops three off-loop hex lines
(budget 0, FIFO draining ~64 B/ms) and a loop-task ERROR 1 ms later then gets budget 0 when
20 ms would have saved it. Loop-only also makes it single-writer, so it needs no atomics.

Two further rules: a **lock-timeout** drop does not feed the backoff (it says nothing about the
port), and the **ready-hook** early return neither counts nor resets, so a counter ≥3 can survive
a DTR-low period — self-healing on the first success.

**Cut from the previous design: the level-aware budget** (DEBUG 5 ms vs 20 ms). It existed to
ration a safety-critical resource. With safety structural, and with 0-off-loop plus the backoff
covering the pathological cases, the extra state is not worth it.

`s_loopTask` is captured via `od_log_set_loop_task(xTaskGetCurrentTaskHandle())` **immediately
after `od_log_init()`** in `setup()`. Both cores run `setup()` and `loop()` on the same task —
nRF's `loop_task` calls `setup()` then loops (`cores/nRF5/main.cpp:47-73`), and the ESP32
`loopTask` does the same.

### 6. ESP32 — unchanged, and it never drops

`od_port_write()` uses `Stream::write` there; `od_budget_ms()` is irrelevant because
`od_port_wait_ready()` short-circuits to true. Neither ESP32 log port can block on host
backpressure:

| Port | Envs | Bound |
|---|---|---|
| HWCDC | all except `-extuart` | 100 ms lock + 20 × 100 ms ≈ **2.1 s**, then short write |
| UART1 | the three `-extuart` envs | **baud-bounded**, ~18 ms for a 210 B line at 115200 |

`HWCDC::write()` caps at `max_consec_timeouts` × `tx_timeout_ms` and separates real unplug from
backpressure. `uartBegin()` hardwires `flow_ctrl = UART_HW_FLOWCTRL_DISABLE`
(`esp32-hal-uart.c:1117`) and `begin()` never enables CTS.

Two caveats: `UART_MUTEX_LOCK()` is `do {} while (xSemaphoreTake(uart->lock, portMAX_DELAY))`
(`esp32-hal-uart.c:108`) — bounded by the invariant that the logger is UART1's only user, not by
construction. And bounded is not fast; "always ready" means "cannot hang", not "cannot stall".

**ESP32 never counts a drop**, so `[DROP: x]` can never appear there and byte-identity holds
unconditionally. Short writes are consequently *not* counted there — an unqualified "short writes
count" rule would set `s_dropped`, put the next line on a tagged path that is nRF-only, and break
that guarantee. ESP32 keeps today's behaviour exactly: return ignored, loss silent.

> **Correction to an earlier claim in this investigation.** The bounded `HWCDC::write()` was
> reported as an untracked local patch CI would not have. Wrong — every file in `cores/esp32/`
> carries the package install mtime, so nothing was hand-edited; the cap ships in pioarduino
> `55.03.39`, which every build pins.

### 7. Drop counter and `[DROP: x]`

`__atomic_fetch_add` on every increment — producers increment outside the mutex. Wrap accepted,
not saturated (saturation needs a CAS loop; 2³² drops is 49 days of continuous dropping at
1000/s). `reported = load()` before the writes; `fetch_sub(reported)` after they complete.

Placement is after the level, before the message, so the timestamp stays in column 1:

```
[0416.212|C0] I: === [BLE] PIPE WRITE END COMMAND (0x0082) ===
[0416.213|C0] I: [DROP: 214] DW complete: 307 chunks, 96000/96000 bytes
[0416.214|C0] I: EPD refresh: FULL (mode=0, end payload 0x00)
```

No extra line; zero drops ⇒ no tag. `tagAt` is the `pos` returned by `_od_log`'s header
`snprintf` ([`od_log.cpp:27-30`](../src/od_log.cpp)); the worst-case header is ~29 chars so it
cannot truncate, and `pos < 0` is handled at `:31`. Max tag is 19 bytes
(`"[DROP: 4294967295] "`), so the worst wire line is 232 + 19 + 2 = **253**, inside the 256-byte
`CFG_TUD_CDC_TX_BUFSIZE`.

`od_log_raw()` passes `tagAt = -1`. It emits partial lines — `waitforrefresh()` progress dots —
so it respects the wait and counts drops but is never spliced; the count surfaces on the next
`_od_log()` line.

### 8. Dark-port guard — nRF only

With DTR low the FIFO is overwritable (`cdc_device.c:394`), so `tud_cdc_write_available()` can
read 0 while a write would in fact succeed by discarding old bytes. All-or-nothing gating would
then drop every line on an unattended tag and hand the first attaching terminal
`[DROP: 4102931]` — a true number that says nothing.

`od_log_set_ready_hook([]() -> bool { return (bool)Serial; })` returns early **without counting**.
`Adafruit_USBD_CDC::operator bool()` is `tud_cdc_n_connected()` — literally the old write loop's
continue condition.

Not installed on ESP32: `HWCDC::isCDC_Connected()` documents that its SOF watchdog "is known to
flap even on a healthy link" (`HWCDC.cpp:268-275`), so a hook there would discard good output.

### 9. `od_log_flush()`

Routes to `tud_cdc_write_flush()` on nRF (non-blocking by construction) and `s_port->flush()` on
ESP32. Takes the mutex to avoid racing a mid-record emitter; the existing unconditional `delay(5)`
([`od_log.cpp:92`](../src/od_log.cpp)) stays **outside** the lock — 16 boot call sites × 5 ms of
hold is not something to add.

---

## Off-loop logging inventory

Bluefruit defers callbacks to a dedicated task — `xTaskCreate(adafruit_callback_task, "Callback",
..., TASK_PRIO_NORMAL, ...)` (`cores/nRF5/utility/AdaCallback.c:147`), `TASK_PRIO_NORMAL == 2`
(`cores/nRF5/rtos.h:59`). Every site below runs at priority 2, not on `loop()`:

| Site | Level | Fires on |
|---|---|---|
| [`command_queue.cpp:49`](../src/command_queue.cpp) "Empty BLE frame received" | WARN | every zero-length write |
| [`command_queue.cpp:53`](../src/command_queue.cpp) "Command too large for queue" | WARN | every oversized write |
| [`command_queue.cpp:63`](../src/command_queue.cpp) "Command queue full" | ERROR | ring full |
| [`command_queue.cpp:84`](../src/command_queue.cpp) the `ERX`/`URX` hex line | DEBUG | every accepted frame not suppressed by `imageWriteLogQuietFrame` |
| [`ble_transport_nrf.cpp:126`](../src/ble_transport_nrf.cpp) "BLE CLIENT CONNECTED" | INFO | per connect |
| [`ble_transport_nrf.cpp:133`](../src/ble_transport_nrf.cpp) "BLE CLIENT DISCONNECTED" | INFO | per disconnect |

The first four are inside `bleRxQueuePush()`, reached from `onWriteCb`
([`ble_transport_nrf.cpp:155`](../src/ble_transport_nrf.cpp)), which deliberately logs nothing
itself.

Plus the **FreeRTOS timer task** (also priority 2): `linkDiagCallback`
([`ble_transport_nrf.cpp:101`](../src/ble_transport_nrf.cpp)) → `logLinkParams()` → the
`[LINK negotiated]` line at [`:89`](../src/ble_transport_nrf.cpp). `logLinkParams()` is also
called from `requestFastLink()` on `loop()` ([`main.cpp:449`](../src/main.cpp)), so the task check
must be a **runtime** test, not a compile-time split.

**The write callback can escalate to priority 3.** `setWriteCallback(onWriteCb)` defaults to
`useAdaCallback = true`, but the dispatch has an inline fallback:

```c
// Bluefruit52Lib/src/BLECharacteristic.cpp:538
if ( !(_use_ada_cb.write && ada_callback(...)) ) {
    _wr_cb(conn_hdl, this, request->data, request->len);   // inline: BLE task, priority 3
}
```

`ada_callback()` fails when its queue is full (`xQueueSend(..., CFG_CALLBACK_TIMEOUT)`, 100 ms,
`AdaCallback.h:42`). So under exactly the flood conditions that matter, those four log lines move
to priority **3**. The `xTaskGetCurrentTaskHandle() != s_loopTask` test catches it identically —
recorded because it makes the worst case "priority 3 above everything", which is the strongest
argument for a 0 ms off-loop budget rather than a small non-zero one.

---

## The single-writer invariant — now integrity, not safety

Under the previous design this proved the firmware could not hang. It no longer does: a foreign
writer consuming our reservation truncates a line, it cannot make `tud_cdc_write()` spin. Still
worth keeping as an integrity check.

Verified for shipping envs: no nRF env sets `CFG_DEBUG` ([`platformio.ini:43`](../platformio.ini);
platform default 0 in `nordicnrf52/builder/frameworks/arduino/adafruit.py:250`), so Bluefruit
`LOG_LV*` compiles out; SEGGER RTT is a separate buffer; no `tud_cdc_tx_complete_cb` refill writer
is linked. **But the stdio retarget is linked and dormant, not absent** — `_write()` routes stdout
to `Serial.write()` (`cores/nRF5/main.cpp:121`).

---

## Files

| File | Change |
|---|---|
| [`src/od_log.cpp`](../src/od_log.cpp) | `od_port_write()` backend; `od_emit()`; capacity reservation; tick deadline; static mutex; budget + backoff; atomic drop counter; `od_log_flush()` routing |
| [`src/od_log.h`](../src/od_log.h) | `od_log_set_ready_hook()`, `od_log_set_loop_task()`, drop/tag contract comment |
| [`src/main.cpp`](../src/main.cpp) | nRF-only ready hook after `od_log_init`; `od_log_set_loop_task()` immediately after it |
| [`src/command_queue.cpp`](../src/command_queue.cpp) | correct the now-false "od_log ends in a blocking serial write (~9 ms …)" comment at `:36-41` — load-bearing rationale for logging on the callback task |
| `tests/serial_stall_test.py`, `tests/README.md` | restore from `13fb679`; match string `"[od_log] dropped"` → `"[DROP:"` (4 sites: `DROP_NOTICE`, docstring, `--expect-drop-notice` help, failure messages) |

Estimated code change: **~100 lines** in `od_log.cpp`, against the previous design's surface. No
`platformio.ini` change. No `diagnostics.*`. No `display_service.cpp` change.

**Out of scope:** the heap-stats and 5 s heartbeat from the reverted `a84e512`. The audit's
Stage 1 items 5–6 still want a heartbeat; separate commit.

---

## Accepted limitations

- **Starvation is reduced, not solved.** Off-loop logging becomes non-blocking, removing the
  priority-inversion wait — but a flood still costs formatting time on those tasks. Audit §D stays
  open; the watchdog is its remedy.
- **The debug build is not diagnostically equivalent to today under load.** The 256-byte FIFO
  drains 64 bytes per completed bulk-IN transaction (`cdc_device.c:168-179`, `:465-467`), so a
  sustained `-debug` rate above the IN completion rate drops even with a healthy host, and
  off-loop lines drop immediately. `imageWriteLogQuietFrame`
  ([`display_service.cpp:1910`](../src/display_service.cpp)) already suppresses the highest-rate
  source. The trade is completeness for not wedging.
- **Evidence density collapses where the freeze hunt needs it.** The first lines to drop are the
  `[hb]` heartbeat and the ERX arrival lines. "No ERX line" becomes ambiguous between *frame never
  arrived* and *logger dropped it*; `[DROP: n]` gives the count, not the identity. Worth an
  addendum to the §D bracketing methodology in the findings doc.
- **ESP32 loss is silent** — short writes uncounted there, as today. The drop counter is an
  nRF-only instrument.
- **nRF loses the `Stream*` abstraction.** Contained to `od_port_write()`, but `od_log` is no
  longer backend-agnostic on that target.
- **Two diagnostic envs still write `Serial` directly**: `OPENDISPLAY_BOOT_DIAG` (unbounded
  `while (!Serial)` at [`src/main.cpp:70`](../src/main.cpp)) and
  [`src/utilities/nrf52840_reformat/main.cpp`](../src/utilities/nrf52840_reformat/main.cpp)
  (`:149`). Both excluded from `default_envs`.
- **"Just enlarge the FIFO" is unavailable.** `CFG_TUD_CDC_TX_BUFSIZE` is a bare `#define` with no
  `#ifndef` guard (`arduino/ports/nrf/tusb_config_nrf.h:66`).

---

## Verification

- `pio run` — all 11 CI envs, plus `nrf52840custom-debug`, `nrf52840-reformat`,
  `nrf52840-bootdiag`, `esp32-s3-N16R8-extuart-debug`.
- **Zero-drop byte-identity on a quiet link**, normalising the timestamp prefix (every line starts
  `[%04lu.%03lu|C%lu]`, [`od_log.cpp:27`](../src/od_log.cpp), and millis differs every boot):
  `sed -E 's/^\[[0-9]+\.[0-9]+\|C[0-9]+\] //' before.log > a; …; diff a b`.
  Must hold unconditionally on ESP32.
- **Single-writer / integrity regression test**, anchored so it is usable:
  ```bash
  grep -rnE '\b(printf|puts)\s*\(|\bSerial\.(write|print|println|printf)\b' src/ \
       --exclude=od_log.cpp --exclude-dir=utilities
  ```
  Expect **9 hits, all in `src/main.cpp`, all inside `#ifdef OPENDISPLAY_BOOT_DIAG`** (lines
  79-81, 137, 142, 156, 161, 171, 176 — verified 2026-07-29). The unanchored form gives 121 hits
  on a clean tree because `printf` substring-matches `snprintf`.
- **Bench, nRF** (`nrf52840custom-debug`): `./tests/serial_stall_test.py --port /dev/ttyACM0
  --stall 45 --expect-drop-notice`, triggering an image push during the stall. Expect the transfer
  and refresh to complete and the first complete line on resume to carry `[DROP: n]` after its
  level. **Run it on the parent commit first** — a FAIL there is the single result that validates
  the whole diagnosis.
- **Callback-task WARN flood**: drive [`command_queue.cpp:47,52,62`](../src/command_queue.cpp)
  with malformed/oversized frames and confirm `loop()` keeps being scheduled.
- **Backoff engages and releases**: `loop()` latency must not grow with the number of attempted
  lines during a stall (~3 × budget, not ~300 ×), and the first line after the host resumes must
  restore full-budget behaviour rather than staying latched.
- **Stall during a refresh's progress dots**: drops counted but untagged; the count appears on the
  next `_od_log()` line, not inside the dot run.
- Scope the timing claim correctly: the **waits** are budgeted. Formatting, preemption and the
  writes are outside it.
