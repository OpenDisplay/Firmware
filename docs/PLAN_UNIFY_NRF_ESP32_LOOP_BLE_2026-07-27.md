# Investigation: unifying the nRF52840 and ESP32 loop / BLE architecture

**Date:** 2026-07-27 · **Status:** investigation only — no code changed

> **Amendment 2026-07-27 — the RAM gate is retired.** The owner confirms nRF has
> sufficient headroom for the shared rings, so the "≥15 KB or don't bother"
> condition in §5 no longer blocks Level 2, and the RX ring keeps its full depth
> (no narrowed PIPE window, no throughput trade). Idle current and pipe-write
> throughput remain worth capturing, but as before/after **baselines** taken
> ahead of the execution-model change — not as go/no-go gates. Sequencing and
> file-level detail live in `PLAN_BLE_TRANSPORT_ABSTRACTION_2026-07-27.md`.

## Question

What would it take to collapse the two per-target loop/BLE paths into one, so the
firmware stops carrying two ways of doing the same thing?

## Answer in one paragraph

The *protocol* layer is already unified and should be left alone. The divergence
that matters is a **threading model difference**, not a code-organisation one: on
ESP32 every command is dispatched from `loop()`; on nRF every command is
dispatched from the Bluefruit callback task and `loop()` is an idler. Everything
else people notice as "two paths" — two `#ifdef` tails in `sendResponse()`, two
advertising blocks in `updatemsdata()`, two halves of `ble_init.cpp` — is a
consequence or a cosmetic sibling of that. There are three separable levels of
work; **Level 1 is cheap and worth doing now, Level 2 is the real unification and
should not start until nRF RAM headroom and idle current are measured on
hardware.**

---

## 1. What is already unified (do not touch)

| Area | Where | Notes |
|---|---|---|
| Command dispatch | `communication.cpp:625` `imageDataWritten()` | One opcode switch serving nRF BLE, ESP32 BLE and ESP32 LAN. `BLEConnHandle`/`BLECharPtr` typedefs absorb the signature difference. |
| All command handlers | `display_service.cpp`, `communication.cpp`, `device_control.cpp` | Config read/write/chunk, direct write, partial write, PIPE_WRITE, LED, buzzer — no target branches in the logic. |
| Encryption envelope | `communication.cpp:663-711` | AES-CCM gate, replay window, origin-gated decrypt — shared. |
| PIPE_WRITE window/reorder | `display_service.cpp:2483-2900`, `structs.h:32-54` | Sequence, ACK cadence, reorder queue — fully shared. |
| Wire constants | `include/opendisplay_protocol.h` (vendored) | Single source of truth. |
| Transport-origin routing | `g_commandOrigin` / `originTag()` | Already models "N transports, one dispatcher". |

This is the important part: **a third transport (LAN) was added without forking
the dispatcher.** The abstraction the codebase is missing is not on the command
path — it is on the *delivery* path and the *scheduling* path.

## 2. What actually diverges

### A. Execution model — the root cause

**ESP32.** NimBLE host task → `MyBLECharacteristicCallbacks::onWrite()`
(`esp32_ble_callbacks.h:81`) copies the frame into a 33-slot SPSC ring →
`loop()` (`main.cpp:406-423`) drains it, dispatches, and flushes a 10-slot TX ring
back out via `notify()`. The host task touches nothing but the ring. Heavy or
state-mutating work is deferred to `loop()` behind flags:
`bleDisconnectCleanupPending`, `msdUpdatePending`, `bleRestartAdvertisingPending`.

**nRF.** Bluefruit's SoftDevice callback task calls `imageDataWritten()`
**directly** (`ble_init.cpp:157` `setWriteCallback`). Dispatch, zlib inflate, EPD
SPI streaming and `notify()` all run on the BLE task. `loop()`
(`main.cpp:518-530`) is a housekeeping idler: `idleDelay(500)` (or
`sleep_timeout_ms`), advertising tick, buttons, touch, buzzer.

Direct consequences, all visible in the tree today:

- `pwrmgmLock` (`main.h:185`, `display_service.cpp:401-526`) exists **only**
  because of this: it is a genuine cross-task try-lock on nRF (BLE task vs loop
  task) and uncontended on ESP32. The audit already records it as deliberate
  (`docs/AUDIT_FIRMWARE_2026-07-13.md:274`).
- The flag-and-defer callback pattern exists only on ESP32.
- `handleReadConfig()` needs an `#ifdef` in the middle of a loop
  (`communication.cpp:468-476`): ESP32 flushes the TX ring between chunks, nRF
  just `delay(50)`.
- Audit finding M1 (4 KB stack buffer in the BLE-callback context) is an
  nRF-only class of bug that cannot exist under the ESP32 model.

### B. Response path

| | nRF | ESP32 |
|---|---|---|
| Call site | inline from BLE task | enqueue → `flushResponseQueueToBle()` in `loop()` |
| Backpressure | 4 retries × 5 ms on `notify()==false` (`communication.cpp:345-349`) | leave entry queued, retry next pass (`main.cpp:288`) |
| Overflow | none possible (synchronous) | 10-slot ring; mid-drain flushes added to stop pipe ACKs overflowing it |
| Latency | same radio event | ≥1 loop pass |

### C. BLE stack API

Bluefruit value-objects (`imageService`, `imageCharacteristic`, `bledfu`,
`Bluefruit.Advertising.*`) vs NimBLE-Arduino pointers, aliased back to the
historical `BLE*` spellings in `ble_init.h:21-30`. Behavioural gaps that are
*not* stack limitations, just work done on one target only:

- **Link tuning**: nRF explicitly requests 2M PHY + 251-octet DLE and logs
  negotiated params (`ble_init.cpp:83-145`). ESP32/NimBLE has no equivalent.
- **Advertising interval boost** on button press: nRF only
  (`ble_init.cpp:46-76`, `device_control.cpp:616`).
- **MSD update**: nRF rebuilds and restarts advertising inside `updatemsdata()`;
  ESP32 pushes `setAdvertisementData()` and restarts only while disconnected
  (`display_service.cpp:1767-1800`).
- **MTU**: nRF fixed at 247 by `configPrphBandwidth(BANDWIDTH_MAX)`; ESP32
  requests `OD_BLE_PREFERRED_ATT_MTU` (256). Documented in `structs.h:64-66`.
- **DFU**: nRF registers `bledfu` when encryption is off; ESP32 has no analogue.

### D. Lifecycle / power — genuine capability difference

`main.cpp`'s largest `#ifdef TARGET_ESP32` regions are deep-sleep machinery:
wake-cause detection, `pollActivity()`, min-wake window, advertising-timeout
window, `fullSetupAfterConnection()`, `enterDeepSleep()` with stack teardown.
nRF has none (`getDeepSleepCount()` returns 0). **This is not accidental
divergence and should not be merged into shared code** — only hooked.

### E. Adjacent, non-BLE platform splits (out of scope for this question)

Filesystem (`InternalFS` vs `LittleFS`, ~21 branches in `config_parser.cpp`),
crypto backend (CC310 vs mbedTLS, ~16 in `encryption.cpp`), chip ID, chip
temperature, FastEPD, WiFi/LAN, ADC ladder, `IRAM_ATTR` ISRs.

---

## 3. Three levels of work

### Level 1 — symmetric BLE port layer (recommended now)

Introduce `src/ble_port.h` with one interface and two implementations
(`ble_port_nrf.cpp`, `ble_port_esp32.cpp`), each compiled whole — no `#ifdef`
*inside* either file:

```c
void     od_ble_init(const char* name);
void     od_ble_advertising_start(void);
void     od_ble_advertising_restart(void);
void     od_ble_set_manufacturer_data(const uint8_t* msd, uint8_t len);
uint8_t  od_ble_connected_count(void);
bool     od_ble_notify_ready(void);
bool     od_ble_notify(const uint8_t* data, uint16_t len);
void     od_ble_request_fast_link(void);      // no-op where unsupported
void     od_ble_deinit(void);                 // no-op on nRF
```

Removes, without touching threading:

- both `#ifdef` tails in `sendResponse()` / `sendResponseUnencrypted()`
  (`communication.cpp:216-238`, `324-355`) → one `od_ble_notify()` call;
- both advertising blocks in `updatemsdata()` → one
  `od_ble_set_manufacturer_data()`;
- `pServer` / `advertisementData` externs leaking into `display_service.cpp`,
  `communication.cpp`, `main.cpp`;
- the target split inside `ble_init.cpp` (becomes a file split).

Also the natural place to close the C-gaps: implementing `od_ble_request_fast_link()`
for NimBLE (2M PHY + DLE) gets ESP32 the link tuning nRF already has.

**Cost:** ~2–3 days including a bench pass on both targets. **Risk:** low — no
scheduling change, and the notify/advertising call sequences move verbatim.
**Note:** NimBLE's "`setAdvertisementData()` must be the last call before
`start()`" constraint (`ble_init.cpp:308-312`) has to survive the port; it is the
one non-obvious ordering rule in the ESP32 implementation.

### Level 2 — unify the execution model (the actual fix)

Make nRF adopt the ESP32 model: the Bluefruit write callback enqueues, `loop()`
dispatches.

1. Move `CommandQueueItem` / `commandQueue` / heads out of
   `esp32_ble_callbacks.h` and out of `main.h`'s `#ifdef TARGET_ESP32` block into
   a shared `src/ble_rx_queue.{h,cpp}`. Move `ResponseQueueItem` out of
   `structs.h:77-91`'s ESP32 guard likewise.
2. nRF write callback becomes a thin `od_ble_rx_push(data, len)` — the same SPSC
   acquire/release ring, now shared.
3. `flushResponseQueueToBle()` becomes shared `bleServiceTx()`; nRF's retry
   policy folds into the existing "stop on `notify()==false`, retry next pass"
   rule (the ESP32 policy is strictly better — it never blocks).
4. Rewrite `loop()` as one shared skeleton — drain commands (bounded, flush TX
   between) → service deferred flags → timeouts → input polling — plus a
   `platform_idle(bool workInFlight)` hook that is the deep-sleep policy on
   ESP32 and `idleDelay` + advertising tick on nRF. `pollActivity()` becomes
   shared but only the ESP32 policy consumes it.
5. **`idleDelay()` must also drain both queues.** Today nRF cannot stall a
   command inside `idleDelay` because dispatch is on the BLE task. After the
   change it can, for up to 100 ms per chunk. This is the single largest
   behavioural regression risk in the whole plan.
6. `handleReadConfig()`'s `#else delay(50)` disappears — nRF gains the
   flush-between-chunks semantics.
7. `pwrmgmLock` becomes uncontended. Keep it (it is nearly free) rather than
   remove it — touch/button paths still run from `loop()` and ISRs.

**Measurements:**

- ~~**RAM.**~~ **Resolved 2026-07-27 — headroom confirmed sufficient.** The
  shared rings add ≈8.4 KB (`33 × 256`) + ≈2.6 KB (`10 × 256`) of `.bss` on nRF,
  on top of the ≈8.3 KB PIPE reorder queue it already carries. That fits, so the
  RX ring keeps its full depth. The concern was that shrinking it below `W+1`
  would cap the PIPE_WRITE window and cost throughput — the `env:esp32-N4`
  `PIPE_SMALL_DRAM_WINDOW` precedent (`structs.h:40-53`). That trade is no longer
  on the table for nRF.
- **Idle current** *(baseline, not a gate)*. nRF currently spends idle time inside `delay()`, which yields
  to the FreeRTOS idle task and `sd_app_evt_wait`. A loop that spins at
  `delay(1)` while a link is up (as ESP32 does under `workInFlight`) changes the
  battery profile. Measure before/after on a battery unit.
- **Throughput** *(baseline, not a gate)*. nRF ACKs a PIPE frame within the same radio event today; via
  the queue it waits for a loop pass. Re-run the pipe-write benchmark in
  `docs/pipe-write-protocol.md` on both targets and compare.

**Cost:** ~1–2 weeks including bench validation. **Risk:** high — it touches the
two properties the firmware is most sensitive to (transfer throughput and battery
current) on the target with the least headroom, and there are no automated tests;
CI builds all 11 environments but verifies nothing at runtime.

### Level 3 — full platform HAL (independent)

`od_fs_*`, `od_crypto_*`, `od_chip_id()`, `od_chip_temperature()`. Mechanical,
orthogonal to this question, removes ~40 more `#ifdef`s. Can be done any time,
before or after Level 2.

---

## 4. What should stay divergent

- **Deep sleep / wake / power latch** — real capability difference. Hook it
  (`platform_idle()`), do not merge it.
- **FastEPD, WiFi/LAN** — ESP32-only subsystems, already cleanly guarded.
- **The stack APIs themselves** — two implementation files with no internal
  `#ifdef` is the goal, not one file that branches.

## 5. Recommendation

Do **Level 1** now: it removes the visible duplication, is independently
valuable, has low hardware risk, and creates the seam Level 2 needs anyway.

~~Then take the two nRF measurements … **Level 2 is only worth it if nRF has
≥15 KB of headroom**~~ — **superseded 2026-07-27.** nRF headroom is confirmed
sufficient, so Level 2 is cleared to proceed with a full-depth RX ring. The
feared outcome (smaller ring → narrower PIPE window → throughput regression
traded for architectural tidiness) does not apply. Capture idle current and a
pipe-write run as "before" baselines ahead of the execution-model change, so a
regression is detectable rather than gate-keeping.

The payoff for Level 2, stated plainly, is: one loop to reason about, one place
to fix a queue/backpressure bug, nRF gains the response-ring flow control that
ESP32 got from real overflow incidents, and a whole class of
"runs-on-the-BLE-callback-stack" bugs (audit M1, L4) becomes structurally
impossible. That is a real payoff — it is just not free, and the cost lands
entirely on the most constrained target.
