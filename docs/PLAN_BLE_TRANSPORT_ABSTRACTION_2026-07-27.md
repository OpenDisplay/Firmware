# Plan: BLE transport abstraction + nRF copy-and-enqueue callbacks

**Date:** 2026-07-27 · **Scope:** `Firmware` repo only · **Status:** plan, not implemented

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
carries. This is the plan's primary risk — see §7.

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

- **Phase 0 — measurement gate (no code).** On nRF: free RAM with a live
  connection mid-pipe-transfer, and battery idle current. Record both.
  **If headroom < ~15 KB, stop and re-scope** to a reduced `BLE_RX_QUEUE_SLOTS`
  with the throughput cost accepted explicitly.
- **Phase 1 — introduce the abstraction, no behaviour change.** Create the six
  files; move existing per-target code behind the class verbatim; migrate all
  ~50 call sites. Threading untouched — nRF still dispatches in its callback.
  Delete `ble_init.*` and `esp32_ble_callbacks.h`. *This phase alone delivers
  requirements 3, 4 and 5.*
- **Phase 2 — portable queues.** Move RX/TX rings out of the ESP32 guards
  (§4). ESP32 behaviour identical; nRF still bypasses them.
- **Phase 3 — nRF copy-and-enqueue (the core change).** nRF write callback →
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

---

## 7. Risks and gates

| Risk | Severity | Mitigation |
|---|---|---|
| nRF `.bss` +11 KB doesn't fit alongside SoftDevice S140 @ `BANDWIDTH_MAX` | **High** | Phase 0 gate; `BLE_RX_QUEUE_SLOTS` knob |
| Command stalls up to 100 ms inside `idleDelay()` — nRF cannot stall today | **High** | `idleDelay()` must drain RX/TX every iteration, not just poll input |
| nRF idle current rises if `loop()` spins to stay responsive | Medium | Measure in Phase 0; spin only while `workInFlight`, as ESP32 does |
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
