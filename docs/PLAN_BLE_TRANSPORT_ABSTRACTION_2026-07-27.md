# Plan: BLE transport abstraction + nRF copy-and-enqueue callbacks

**Date:** 2026-07-27 · **Scope:** `Firmware` repo only · **Status:** plan, not implemented

> **Amendment 2026-07-27 — Phase 0 retired.** The owner confirms nRF has
> sufficient RAM headroom for the ≈11 KB of new `.bss`, so the measurement gate
> no longer blocks the start of work and `BLE_RX_QUEUE_SLOTS` keeps its full
> 33-slot depth. The other two Phase 0 numbers (battery idle current,
> pipe-write throughput) were never gates on Phases 1–2 — they are before/after
> baselines and are now required **before Phase 3 lands**, not before Phase 1
> starts. See §6.

Companion to `PLAN_UNIFY_NRF_ESP32_LOOP_BLE_2026-07-27.md`, which established
*why* this direction is the correct one. This document is the *how*.

## Goal

1. nRF BLE callbacks do **copy-and-enqueue only** — no command dispatch, no
   crypto, no SPI/I2C, no panel work, no `notify()` on the callback task.
2. All application work runs on the main `loop()` task.
3. Stack-specific code lives in its own `.cpp` + `.h` per platform, gated by
   platform macro; no `#ifdef TARGET_*` inside application files.
4. A **thin** `BleTransport` class abstracts the small feature set the
   application actually uses.
5. Every BLE touch in application code routes through that class.

### In scope

`ble_init.{h,cpp}`, `esp32_ble_callbacks.h`, and the BLE call sites in
`main.{h,cpp}`, `communication.cpp`, `display_service.cpp`, `device_control.cpp`,
`wifi_service.cpp`.

### Explicitly out of scope

Deep sleep / wake policy (stays ESP32-only, behind the existing guards);
FastEPD; WiFi/LAN transport; filesystem and crypto backends; DFU entry
(`enterDFUMode()` stays in `device_control.cpp` — it is a bootloader handoff,
not a link concern); any protocol or wire change.

---

## 1. The threading contract

Stated once, enforced everywhere afterwards:

> **BLE stack callbacks may do exactly two things: copy bytes into the RX ring,
> and set a flag. Everything else happens on the `loop()` task.**

This already holds on ESP32. The work is making it hold on nRF, and encoding it
so it cannot silently regress.

**Ordering hazard — read before sequencing the work.** Today nRF's
`connect_callback` / `disconnect_callback` do heavyweight work
(`updatemsdata()` → I2C + ADC + advertising rebuild;
`cleanupDirectWriteState(true)` → SPI + rail cut). That is currently *safe*
only because command dispatch is on the same task, so the two are serialized by
construction. **The moment dispatch moves to `loop()`, those callbacks become a
genuine cross-task race** — precisely findings #1/#2/#3 from
`FIRMWARE_NIMBLE_PORT_CODE_REVIEW_2026-07-17.md`, which is exactly how ESP32
acquired them during the NimBLE migration. Converting the nRF callbacks to
flag-only is therefore **not a separate follow-up**; it must land in the same
commit as the dispatch move (Phase 3 below), or the refactor reintroduces a
known Critical bug on nRF.

---

## 2. The transport interface

`src/ble_transport.h` — portable, includes **no** stack headers, safe for any
translation unit:

```cpp
#ifndef BLE_TRANSPORT_H
#define BLE_TRANSPORT_H
#include <stdint.h>

class BleTransport {
public:
    // --- lifecycle (kept as separate calls so each target keeps its own
    //     init ordering: nRF must start the SoftDevice BEFORE display/SPI and
    //     advertise after the boot screen; ESP32 inits BLE after the display) ---
    bool    begin(const char* deviceName);
    void    startAdvertising();
    void    restartAdvertising();      // idempotent; NEVER defers (see note)
    void    stopAdvertising();
    void    end();                     // full teardown; no-op on nRF

    // --- state ---
    uint8_t connectedCount() const;
    bool    isConnected() const { return connectedCount() > 0; }
    bool    notifyReady() const;       // connected AND CCCD subscribed

    // --- data out ---
    // false = backpressure ("retry next pass"), not failure. Caller must leave
    // the entry queued and not advance its tail.
    bool    notify(const uint8_t* data, uint16_t len);

    // --- advertising payload ---
    void    setManufacturerData(const uint8_t* msd, uint8_t len);

    // --- link policy (no-op where the stack does not support it) ---
    void    requestFastLink();         // nRF: 2M PHY + 251-octet DLE
    void    boostAdvertising();        // nRF: temporary fast adv interval
    void    tick();                    // periodic housekeeping (adv interval restore)

    // --- events: consume-once, polled from loop(). No app callbacks. ---
    bool    takeConnectedEvent();
    bool    takeDisconnectedEvent();

    // --- identity ---
    const char* addressString();       // wifi_service.cpp's advertised-MAC use
};

extern BleTransport ble;
#endif
```

Design notes that matter:

- **`restartAdvertising()` never defers.** Today `esp32_restart_ble_advertising()`
  re-pends itself when `epdRefreshInProgress` — an application concern living
  inside link code. Under this design the *app* owns deferral policy and simply
  doesn't call the method yet. Strictly simpler, and it removes
  `bleRestartAdvertisingPending` from the transport's surface.
- **`notify()` gets one unified contract**: return false, leave queued, retry
  next pass. nRF's current inline `delay(5)` × 4 retry loop is deleted — it
  blocks, and the ESP32 policy is the proven one.
- **Events are polled, not dispatched.** No virtuals, no app-facing callbacks;
  that is what keeps callback context from leaking back into application code.
- **No RX method.** RX is buffering, not link state — see §4.

### Why one class, two `.cpp`s (not an abstract base)

Exactly one implementation is live per build, so virtual dispatch would cost a
vtable and indirect calls for zero benefit. Same class name, same header, the
platform selects the translation unit. Zero-overhead, and application code sees
one type.

---

## 3. File layout and platform gating

| File | Contents | Gate |
|---|---|---|
| `src/ble_transport.h` | the class above; no stack headers | none — portable |
| `src/ble_transport_nrf.h` | Bluefruit objects, `connect_callback`/`disconnect_callback` decls | whole file in `#ifdef TARGET_NRF` |
| `src/ble_transport_nrf.cpp` | Bluefruit impl of every method | whole file in `#ifdef TARGET_NRF` |
| `src/ble_transport_esp32.h` | NimBLE aliases, `MyBLEServerCallbacks`, `MyBLECharacteristicCallbacks` | whole file in `#ifdef TARGET_ESP32` |
| `src/ble_transport_esp32.cpp` | NimBLE impl of every method | whole file in `#ifdef TARGET_ESP32` |
| `src/ble_rx_queue.{h,cpp}` | shared RX ring (§4) | none — portable |

Each platform `.h` is included **only** from its own `.cpp`, inside that file's
gate. Application files include `ble_transport.h` and nothing else.

**Gating mechanism:** wrap each platform file's entire body in its `#ifdef`, so
the wrong-target build produces an empty translation unit. This needs **no
`build_src_filter` changes across the 11 CI environments** and matches the
convention already used in `esp32_ble_callbacks.h`. If genuinely-not-compiled is
preferred later, `build_src_filter` is the stricter alternative — but that means
editing every ESP32 env (most currently set no filter), so it is deliberately
not the default here.

**Deletions this enables:** `ble_init.h`'s `using BLEDevice = NimBLEDevice;`
alias block currently leaks NimBLE types into six translation units
(`main.h`, `communication.cpp`, `display_service.cpp`, `device_control.cpp`,
`wifi_service.cpp`, `esp32_ble_callbacks.h`). It moves into
`ble_transport_esp32.h` and stops leaking. `ble_init.{h,cpp}` and
`esp32_ble_callbacks.h` are removed once empty.

Globals `pServer` / `pService` / `pTxCharacteristic` / `pRxCharacteristic` /
`advertisementData` / `imageService` / `imageCharacteristic` / `bledfu` become
**file-static** inside their platform `.cpp`, and leave `main.h` entirely.
(`main.h` is included only by `main.cpp`, so it is a single-inclusion globals
header — moving definitions out is safe and a strict improvement.)

---

## 4. Shared RX and TX queues

Both rings move out of ESP32-only guards into portable code.

**RX** — `src/ble_rx_queue.{h,cpp}`, lifted from `esp32_ble_callbacks.h` /
`main.h`'s `#ifdef TARGET_ESP32` block, keeping the SPSC acquire/release
atomics unchanged:

```cpp
bool    bleRxQueuePush(const uint8_t* data, uint16_t len);  // callback task
bool    bleRxQueuePop(uint8_t* out, uint16_t* outLen);      // loop task
uint8_t bleRxQueueDepth();                                  // pollActivity()
```

Sizing stays `COMMAND_QUEUE_SIZE 33` (`W=32` pipe window + END) ×
`MAX_COMMAND_SIZE 256` (`OD_BLE_MAX_FRAME`) ≈ **8.4 KB**, now on both targets.
Add a per-env override knob mirroring the existing `PIPE_SMALL_DRAM_WINDOW`
precedent, in case nRF cannot afford the full depth:

```c
#ifndef BLE_RX_QUEUE_SLOTS
#define BLE_RX_QUEUE_SLOTS 33
#endif
```

Shrinking it below `PIPE_MAX_W + 1` caps the pipe window and costs throughput —
a deliberate trade, never a link-time discovery.

**TX** — move `ResponseQueueItem` / `RESPONSE_QUEUE_SIZE` / `MAX_RESPONSE_SIZE`
out of `structs.h`'s `#ifdef TARGET_ESP32` (10 × 256 ≈ **2.6 KB**).
`flushResponseQueueToBle()` becomes portable `bleServiceTx()`.

**New `.bss` on nRF: ≈11 KB**, atop the ≈8.3 KB pipe reorder queue it already
carries. This was the plan's primary risk; it is **retired** — nRF headroom is
confirmed sufficient, so the full 33-slot depth stands and the
`BLE_RX_QUEUE_SLOTS` knob is kept only as a future escape hatch, not as an
expected fallback. See §7.

---

## 5. Call-site migration

| Today | Becomes |
|---|---|
| `pServer->getConnectedCount()` (main.cpp ×4, communication.cpp) | `ble.connectedCount()` |
| `esp32_ble_notify_enabled()` | `ble.notifyReady()` |
| `pTxCharacteristic->notify(d,l)` | `ble.notify(d,l)` |
| `imageCharacteristic.notify()` + 4× retry loop | `ble.notify(d,l)`, retry next pass |
| `Bluefruit.connected() && imageCharacteristic.notifyEnabled()` | `ble.notifyReady()` |
| `ble_init()` / `ble_nrf_stack_init()` | `ble.begin(name)` |
| `ble_nrf_advertising_start()` | `ble.startAdvertising()` |
| `esp32_restart_ble_advertising()` | `ble.restartAdvertising()` (app gates on `epdRefreshInProgress`) |
| `BLEDevice::deinit(true)` + `esp32_ble_clear_handles()` | `ble.end()` |
| `updatemsdata()`'s two advertising blocks | `ble.setManufacturerData(msd_payload, 16)` |
| `ble_nrf_boost_advertising()` | `ble.boostAdvertising()` |
| `ble_nrf_advertising_tick()` | `ble.tick()` |
| `ble_nrf_request_fast_link()` / `_arm_link_diag()` / `_log_link_params()` | `ble.requestFastLink()` (diagnostics become impl-private) |
| `NimBLEDevice::getAddress().toString()` (wifi_service.cpp:396) | `ble.addressString()` |
| `bleRestartAdvertisingPending` | removed — app-side deferral |
| `msdUpdatePending` / `bleDisconnectCleanupPending` | `ble.takeConnectedEvent()` / `takeDisconnectedEvent()` |

`updatemsdata()` splits cleanly: payload computation (I2C/ADC/pack — loop task
only) stays in `display_service.cpp`; the advertising push becomes one
transport call.

---

## 6. Phasing

Each phase must build all 11 CI environments green and be independently
revertable.

- ~~**Phase 0 — measurement gate (no code).**~~ **Retired 2026-07-27** — nRF RAM
  headroom confirmed sufficient by the owner. Work starts at Phase 1 with the
  full 33-slot `BLE_RX_QUEUE_SLOTS`.
- **Phase 1 — introduce the abstraction, no behaviour change.** Create the six
  files; move existing per-target code behind the class verbatim; migrate all
  ~50 call sites. Threading untouched — nRF still dispatches in its callback.
  Delete `ble_init.*` and `esp32_ble_callbacks.h`. *This phase alone delivers
  requirements 3, 4 and 5.*
- **Phase 2 — portable queues.** Move RX/TX rings out of the ESP32 guards
  (§4). ESP32 behaviour identical; nRF still bypasses them.
- **Phase 3 — nRF copy-and-enqueue (the core change).** *Prerequisite (was
  Phase 0): capture the two nRF **baselines** first — battery idle current, and
  a pipe-write throughput run per `docs/pipe-write-protocol.md`. Neither gates
  the work; both are the "before" half of a before/after pair, and Phase 3 is
  the commit that can move them.* nRF write callback →
  `bleRxQueuePush()`. `loop()` drains and dispatches. `idleDelay()` gains a
  queue-service call. **In the same commit:** nRF `connect_callback` /
  `disconnect_callback` become flag-only (see §1 ordering hazard).
  `handleReadConfig()`'s `#else delay(50)` becomes the shared TX flush.
- **Phase 4 — shared `loop()` skeleton.** Common body (drain → service events →
  timeouts → input polling) with a `platformIdle(bool workInFlight)` hook:
  deep-sleep policy on ESP32, `idleDelay` + `ble.tick()` on nRF.
- **Phase 5 — cleanup.** Update the `pwrmgmLock` comment (now uncontended, kept
  as defence in depth); update `structs.h:64-66` MTU commentary; refresh
  `AUDIT`/`CODE_REVIEW` docs — Phase 3 closes review finding **#20**
  (`loadGlobalConfig()` rebuilding `globalConfig` on the callback task while
  `loop()` reads it), which should be marked resolved.

Phases 1–2 are safe to land without Phase 3. **Phase 3 must not be split.**

### Implementation record (2026-07-27)

All five phases are implemented on `feat/unify-nrf-esp-phase3`. Deviations from
the plan as written, each deliberate:

| Phase | Deviation |
|---|---|
| 1 | The `sendResponse()` `#ifdef` tails were **not** fully removed here. The stack-API half went (both arms call `ble.notify()`), but the queue-vs-inline split is a *threading* difference and only dissolved in Phase 3. The plan overstated what Phase 1 could deliver. |
| 1 | `requestFastLink()` keeps no parameter, but the nRF impl latches the connection handle from its own connect callback rather than being handed one. |
| 2 | Accessors are `peek`/`consume`, not the sketched copying `bleRxQueuePop(out, outLen)`: the consumer owns the slot until it advances the tail, so a pointer is safe and avoids a 256-byte stack buffer plus a memcpy per frame. |
| 2 | Does **not** land the ~11 KB on nRF as implied — `--gc-sections` drops the rings while nothing references them. The `.bss` first appears in Phase 3 (measured: +11 184 B, against the predicted ≈11 KB). |
| 3 | `idleDelay()` drains TX but deliberately does **not** drain RX, contrary to §7's mitigation. Dispatching there would make command handlers reentrant the moment anything calls `idleDelay()` from a handler. It returns early on RX instead, which also caps command latency at one 100 ms check interval rather than the caller's full delay — a stronger mitigation than the one specified. |
| 3 | The app hooks were deleted rather than converted. Routing all teardown through `serviceBleDisconnectCleanup()` was necessary: keeping `bleAppOnDisconnect()` alongside the flag ran the teardown twice, the first time without the `epdRefreshInProgress` guard. That also closed a pre-existing nRF bug — its old disconnect callback ran the teardown with neither the mid-refresh nor the LAN-ownership guard. |
| 3 | `restartsAdvertisingOnDisconnect()` was added so the one genuine capability difference reads as a query instead of a target `#ifdef` at the call site. |
| 4 | nRF **gains** the two session watchdogs (15-minute direct-write timeout, `checkPartialWriteTimeout()`). Both are transport-agnostic and were ESP32-only only because they lived in the ESP32 loop arm. |
| 5 | Audit **L4** is not closed by this work and is now worse on nRF — see the correction in `PLAN_UNIFY_NRF_ESP32_LOOP_BLE_2026-07-27.md` §5. |

Still outstanding: the entire §7 bench matrix, and the two nRF baselines, which
were never captured — so the idle-current and throughput regressions Phase 3
could cause remain unmeasured. The three §8 open questions are also unanswered:
`Bluefruit.connected()`'s exact return semantics, a NimBLE `requestFastLink()`,
and whether the `SoftwareTimer` link-diagnostic one-shot should fold into
`tick()`.

---

## 7. Risks and gates

| Risk | Severity | Mitigation |
|---|---|---|
| ~~nRF `.bss` +11 KB doesn't fit alongside SoftDevice S140 @ `BANDWIDTH_MAX`~~ | ~~High~~ **Retired** | Headroom confirmed sufficient (2026-07-27); `BLE_RX_QUEUE_SLOTS` knob kept as an escape hatch only |
| Command stalls up to 100 ms inside `idleDelay()` — nRF cannot stall today | **High** | `idleDelay()` must drain RX/TX every iteration, not just poll input |
| nRF idle current rises if `loop()` spins to stay responsive | Medium | Baseline before Phase 3; spin only while `workInFlight`, as ESP32 does |
| Pipe-write throughput regression on nRF (ACK now costs a loop pass) | Medium | Benchmark before/after per `docs/pipe-write-protocol.md` |
| Callback→flag conversion missed somewhere on nRF | **High** | Same-commit rule (§1); grep `ble_transport_nrf.cpp` for any call outside push/flag |
| Init-ordering regression (SoftDevice before SPI on nRF) | Medium | `begin()`/`startAdvertising()` deliberately kept separate; `setup()` ordering unchanged |

Verification is **hardware bench only** — CI builds all 11 environments but
executes nothing.

**Bench matrix, both targets:** pipe-write full image (throughput + retry
count); multi-chunk config read-back > 864 B; disconnect mid-transfer;
reconnect and re-subscribe; button + touch during an active transfer; buzzer
command during transfer; ESP32 deep-sleep/wake cycle; nRF battery idle current.

**Rollback:** phases are independent commits; Phase 3 is the only one that
changes runtime behaviour on nRF and can be reverted alone, leaving the
abstraction (Phases 1–2) in place.

---

## 8. Open questions

1. `connectedCount()` on nRF — confirm whether `Bluefruit.connected()` returns a
   connection count or a bool in the pinned core version, and adapt.
2. Should `requestFastLink()` gain a NimBLE implementation (2M PHY + DLE)? ESP32
   has no link tuning today; the abstraction makes adding it a one-file change.
   Recommend yes, but as separate work after Phase 1 so it is measured on its own.
3. nRF's `SoftwareTimer` link-diagnostic one-shot fires on the FreeRTOS timer
   task. It only logs, so it is safe, but it is a third context — either keep it
   impl-private and documented, or fold it into `tick()`.
