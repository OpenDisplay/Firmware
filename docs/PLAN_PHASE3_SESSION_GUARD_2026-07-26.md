# Phase 3 Implementation Plan — `abortToKnownState()` + queue flushes + drain-trap fix

> Companion to `PLAN_FREEZE_PROOFING_2026-07-26.md` (§ "Phase 3") and
> `FINDINGS_FREEZE_PROOFING_PLAN_REVIEW_2026-07-26.md` (findings `[M3]`, `[M5]`, `[H4]`,
> build/portability check).
> Branch: `debug/ble-hardening`. Target: land after Phase 1 (nonce) and Phase 2 (bounded
> waits), before Phase 4 (owner token).
>
> **Bound by the parent plan's *"Hard constraint — NO wire protocol changes"* (§32-57 there).
> Its application to Phase 3 is §2 below; it settles Decision D3 and removes step 3 from the
> teardown.**

---

## 1. What Phase 3 is, and what it deliberately is not

Phase 3 builds the **recovery mechanism** — a single, ordered, idempotent teardown that
returns the device to a known-good state from any wedged transfer — plus the two
ring-buffer primitives it needs and one latent correctness bug in the command drain.

It does **not** decide *when* to recover. Every trigger lives elsewhere:

| Trigger | Phase | Calls |
|---|---|---|
| **BLE/LAN disconnect teardown** | **3 (D1b — see below)** | `abortToKnownState("disconnect", false)` |
| `integrity_failures >= 3` | 5 | `abortToKnownState(..., true)` |
| `reloadConfigAfterSave` | 5 | `abortToKnownState(..., true)` |
| 10-min no-progress supervisor | 6 | `abortToKnownState("supervisor", true)` |
| Recurring command-ring overflow | 7 | via supervisor, not directly |
| BLE idle timeout (5 min) | 7 | `abortToKnownState("idle", true)` |

**Decided (D1 = b): Phase 3 wires the two existing disconnect teardowns to
`abortToKnownState()` now.** As originally scoped by the parent plan Phase 3 would have
shipped with zero callers — the review flags this ("`abortToKnownState` has no callers until
Phase 5/6, so it is dead code"). Instead, [main.cpp:343-347](../src/main.cpp) (ESP32) and
[device_control.cpp:237-239](../src/device_control.cpp) (nRF) are replaced with
`abortToKnownState("disconnect", false)`, which is a strict superset of what those sites do
today. The teardown therefore becomes the most-exercised path in the firmware immediately,
and the hardware tests in §12 test real behaviour rather than a synthetic trigger.

This has three consequences that shape the rest of this plan; all are handled in **§9**:
1. The ESP32 `ownerStillUp` guard must stay **in front of** the call — a WiFi drop also routes
   into `serviceBleDisconnectCleanup` ([wifi_service.cpp:812](../src/wifi_service.cpp)).
2. The nRF site runs on the Bluefruit Callback task, not `loop()`, and the teardown now
   includes work ESP32 deliberately defers. nRF needs the same deferral — see §9.2.
3. Combined with **D6**, Phase 3 now delivers the "clear encryption session on BLE disconnect"
   behaviour the parent plan assigned to Phase 5. See §9.3.

Deliverables:

1. `src/session_guard.h` / `src/session_guard.cpp` (new) — flags, progress stamp storage,
   `abortToKnownState()`.
2. `flushCommandQueue()` / `flushResponseQueue()` in `main.cpp`.
3. Drain-trap fix `[M5]` in `main.cpp`, + removal of the vestigial `pending` field.
4. `g_commandInFlight` as a `volatile uint8_t` depth counter `[H4]`.
5. New helpers the teardown needs and that do not exist today:
   `resetChunkedWriteState()`, `touchForceResume()` `[M3]`, `buzzerForceStop()`,
   `ledForceStop()`.
6. **(D1b)** Both disconnect teardowns rewired to `abortToKnownState()`, plus a new
   loop-serviced deferral on nRF (§9).
7. **(D6c)** `linkIsUp()` — portable per-target predicate defined in `main.cpp`, declared in
   `session_guard.h` (§3a), gating the session clear.
8. **(D7)** `epdStreamInProgress` + `epdForceOffPending` + `serviceDeferredPanelOff()` — the
   panel-safety pair that makes an ungated abort safe and guarantees a refresh always completes
   (§8.3.1); and the invalidate/scrub split in `clearEncryptionSession()`.

---

## 2. Hard constraint — NO wire protocol changes, applied to Phase 3

The parent plan's *"Hard constraint — NO wire protocol changes"* (§32-57) governs every phase.
Phase 3 is the phase least likely to bump into it — it is pure internal state management —
but it touches two things that *look* like protocol surface and one decision that genuinely
was. Recorded here so implementation does not have to re-derive the reasoning.

**In bounds, and why:**

| Phase 3 change | Why it stays inside the constraint |
|---|---|
| Deleting the `pending` field from `CommandQueueItem` / `ResponseQueueItem` (§5) | Both are **firmware-local runtime structs** — [esp32_ble_callbacks.h:25](../src/esp32_ble_callbacks.h) and [structs.h:86](../src/structs.h). Neither is in `include/opendisplay_structs.h`; neither appears on the wire. This is *not* the config-packet layout the constraint protects. |
| New flags, `g_commandInFlight`, `g_lastProgressMs` | Firmware-local scalars in a new `.cpp`. No client can observe them. |
| `flushCommandQueue()` discarding queued commands | Client-observably identical to the existing ring-full drop ([esp32_ble_callbacks.h:128](../src/esp32_ble_callbacks.h)) — a frame the device never answers. The pipe protocol is designed for exactly this: the zero bit in the next SACK triggers a client retransmit (`docs/pipe-write-protocol.md` §5.2). No new behaviour, no new code on the wire. |
| `flushResponseQueue()` discarding queued responses | Same shape as the existing full-ring drop at [communication.cpp:113-116](../src/communication.cpp), and as the unconditional drain-to-nowhere when no central is connected ([main.cpp:307-312](../src/main.cpp)). |
| `abortToKnownState(..., dropLink=true)` terminating the link | Explicitly in bounds: *"Dropping a link … is always a legal outcome; clients already handle it and reconnect."* |
| `resetChunkedWriteState()` | Internal bookkeeping for a transfer the device has already abandoned. The client's own timeout is what it reacts to. |
| Drain-trap fix, depth counter, `main.h:365` comment fix | Correctness and documentation only. |

**Out of bounds for Phase 3 — do not do these:**

- **Do not invent a new abort/NACK frame.** A generic `{RESP_NACK, 0x00, reason}` was a
  candidate for the teardown's client notification; it is a new response shape and therefore
  forbidden. This is now settled in **Decision D3**, not an open option.
- **Do not send an existing `RESP_*`/NACK code in a situation that changes its documented
  meaning.** Reuse is permitted *"as long as the code's documented meaning is unchanged"* —
  so a `{0xFF,0x81}` pipe NACK may only be sent for a genuine pipe failure, never as a
  general-purpose "I aborted" signal for a chunked-config or direct-write abort.
- **Do not touch** `include/opendisplay_protocol.h` or `include/opendisplay_structs.h`, and
  do not push anything through `../opendisplay-protocol`.
- **Do not change** SACK semantics, discard rules, or NACK meaning as documented in
  `docs/pipe-write-protocol.md`. Phase 3 adds no note to that file; the §5.1 documentation
  note the parent plan permits belongs to **Phase 5** (the pipe error-release deadline).

**Escalation rule:** if any part of Phase 3 appears to need a protocol change to work, stop
and escalate rather than pushing a header change. Nothing in the scope above should reach
that point — the only candidate was D3's notification frame, and it is resolved by sending
nothing.

The constraint's own verification (§12) must pass before Phase 3 is called done.

---

## 3. Current-state facts this plan is built on (verified)

| Fact | Location |
|---|---|
| Command ring is ESP32-only, SPSC: producer = NimBLE host task (`onWrite`), consumer = loop | [esp32_ble_callbacks.h:118-128](../src/esp32_ble_callbacks.h), [main.cpp:406-423](../src/main.cpp) |
| `COMMAND_QUEUE_SIZE 33` is defined **twice** — `main.h:371` and `esp32_ble_callbacks.h:19` (guarded `#ifndef`) | [main.h:371](../src/main.h), [esp32_ble_callbacks.h:18-19](../src/esp32_ble_callbacks.h) |
| Response ring is ESP32-only (10 slots), head **and** tail both written on the loop task | [communication.cpp:112-120](../src/communication.cpp), [main.cpp:276-312](../src/main.cpp) |
| nRF has **neither** ring — `imageDataWritten` runs inline on the Bluefruit Callback task; `sendResponse` notifies directly | [communication.cpp:130-132](../src/communication.cpp) |
| Drain caches `tail` at [main.cpp:409](../src/main.cpp), stores `tail+1` at [:417](../src/main.cpp) — a flush from handler context is clobbered | [main.cpp:408-421](../src/main.cpp) |
| `CommandQueueItem.pending` / `ResponseQueueItem.pending` are written but never read | grep: only assignments |
| Existing disconnect teardown (the closest thing to `abortToKnownState` today) | [main.cpp:339-350](../src/main.cpp) ESP32, [device_control.cpp:237-239](../src/device_control.cpp) nRF |
| `transferActive() == directWriteActive \|\| pipeState.active \|\| partialCtx.active` | [display_service.cpp:2502-2504](../src/display_service.cpp) |
| Touch suspend is a **counter** `s_epd_refresh_suspend` (file-static in touch_input.cpp) paired with a **bool** `directWriteTouchSuspended` (file-static in display_service.cpp) | [touch_input.cpp:115-119](../src/touch_input.cpp), [display_service.cpp:2008](../src/display_service.cpp), [:2035-2038](../src/display_service.cpp) |
| Buzzer stop is `static buzzer_stop_internal()` — no public stop entry point | [buzzer_control.cpp:147](../src/buzzer_control.cpp) |
| LED stop is `static led_stop_internal(bool)` — no public stop entry point | [device_control.cpp:341](../src/device_control.cpp) |
| `chunkedWriteState` has no reset function; cleared field-by-field at 4 sites | [communication.cpp:496-513](../src/communication.cpp), [:550](../src/communication.cpp), [:558](../src/communication.cpp), [:574-576](../src/communication.cpp) |
| `esp32-N4` is ESP32 **without** WiFi → LAN code must be `#ifdef OPENDISPLAY_HAS_WIFI`, never `TARGET_ESP32` | platformio.ini:284 |
| nRF sets `lib_ignore = NimBLE-Arduino` → no NimBLE types may leak into shared headers | platformio.ini:36 |

---

## 3a. Header placement — `main.h` is NOT a header `[C1]`

**Read this before writing any declaration.** `src/main.h` looks like a header and is not one:
it has **no include guard** and it **defines** globals rather than declaring them —
[main.h:91](../src/main.h) `BBEPDISP bbep;`, [:165](../src/main.h) `bool directWriteActive = false;`,
[:283](../src/main.h) `chunked_write_state_t chunkedWriteState = {...};`, [:284](../src/main.h)
`globalConfig`, [:289](../src/main.h) `encryptionSession`, [:374-391](../src/main.h) the rings,
`pServer`, and the callback objects.

```
$ grep -rn '#include "main.h"' src/
src/main.cpp:1:#include "main.h"
```

**Exactly one translation unit includes it, and that is load-bearing.** A second includer gets
`multiple definition of 'bbep'` at link time; on nRF it additionally drags in `<bluefruit.h>`
and the NimBLE-aliased `BLE*` types that §10 forbids in shared headers. Note also that
`communication.cpp` does **not** include `main.h` — it re-declares what it needs itself
(`extern chunked_write_state_t chunkedWriteState;` at [communication.cpp:83](../src/communication.cpp)).

So every new declaration goes in a real guarded header. All of these exist and are correctly
guarded already:

| New symbol | Declared in | Defined in |
|---|---|---|
| `resetChunkedWriteState()` | `communication.h` | `communication.cpp` |
| `flushCommandQueue()` / `flushResponseQueue()` | `session_guard.h` | `main.cpp` (ESP32 real, nRF empty — **both out-of-line**, no `static inline` in a header) |
| `linkIsUp()` | `session_guard.h` | `main.cpp`, per-target `#ifdef` |
| `serviceLinkDrop()` / `g_linkDropPending` | `session_guard.h` | `main.cpp` / `session_guard.cpp` |
| `serviceDeferredPanelOff()` / `epdForceOffPending` | `session_guard.h` | `main.cpp` / `session_guard.cpp` |
| `nrfDisconnectCleanupPending` | `session_guard.h` | `session_guard.cpp` |
| `epdStreamInProgress` | `display_service.h` (next to `epdRefreshInProgress`, [:77](../src/display_service.h)) | `display_service.cpp` |
| `pipeWriteActive()` / `partialWriteActive()` | `display_service.h` | `display_service.cpp` |
| `touchForceResumeAll()` | `touch_input.h` | `touch_input.cpp` |
| `touchForceResume()` | `display_service.h` | `display_service.cpp` |
| `buzzerForceStop()` | `buzzer_control.h` | `buzzer_control.cpp` |
| `ledForceStop()` | `device_control.h` | `device_control.cpp` |

**`session_guard.h` may include** `<stdint.h>`, `<stdbool.h>`, `structs.h`. **Never** `main.h`,
`ble_init.h`, `<bluefruit.h>`, `<NimBLEDevice.h>`, or `<WiFi.h>`.
**`session_guard.cpp` may additionally include** `display_service.h`, `communication.h`,
`encryption.h`, `od_log.h`. **Never `main.h`.**

The nRF no-op flushes must be **out-of-line functions in `main.cpp`**, not `static inline` in a
header — a `static inline` defined in `main.h` is invisible to `session_guard.cpp`, which is the
whole point of the exercise.

`main.h` edits in this plan are therefore limited to what is *already* there: the `pending`
field removal (step 4) and the [main.h:365-370](../src/main.h) capacity comment (D5). Nothing
new is added to it.

---

## 4. Step 1 — Public stop/reset helpers (prerequisites)

`abortToKnownState()` needs four entry points that do not exist. Do these first; each is
independently mergeable and independently testable.

### 4.1 `resetChunkedWriteState()`

New in `communication.cpp`, declared in **`communication.h`** (§3a — *not* `main.h`, which
cannot be included twice; `communication.cpp` already re-declares `chunkedWriteState` itself at
[communication.cpp:83](../src/communication.cpp)).

```c
void resetChunkedWriteState(void) {
    chunkedWriteState.active         = false;
    chunkedWriteState.receivedSize   = 0;
    chunkedWriteState.expectedChunks = 0;
    chunkedWriteState.receivedChunks = 0;
    chunkedWriteState.totalSize      = 0;
    // buffer intentionally NOT zeroed: MAX_CONFIG_SIZE memset on every abort is
    // wasted work; `active=false` makes the contents unreachable.
}
```

Then replace the four ad-hoc clear sites ([:550](../src/communication.cpp),
[:558](../src/communication.cpp), [:574-576](../src/communication.cpp)) with calls to it, so
a future field addition cannot leave a partial reset behind. (The [:496-513](../src/communication.cpp)
*start* path stays as-is — it initialises rather than resets.)

**Note:** [:574-576](../src/communication.cpp) currently clears only 3 of the 5 fields —
`expectedChunks` and `totalSize` survive a completed config write. Harmless today because
`active=false` gates every reader, but the consolidation fixes it for free.

### 4.2 `touchForceResume()` `[M3]`

Two statics in two translation units, so this is two functions:

```c
// touch_input.cpp / touch_input.h
void touchForceResumeAll(void) {
    s_epd_refresh_suspend = 0;
}
```

```c
// display_service.cpp / display_service.h  — the one abortToKnownState calls
void touchForceResume(void) {
    directWriteTouchSuspended = false;
    touchForceResumeAll();
}
```

Rationale (review `[M3]`): zeroing the counter alone leaves `directWriteTouchSuspended`
`true`, so the *next* `cleanupDirectWriteState` calls `touchResumeAfterEpdRefresh()` against
an already-zero counter (early-returns at [touch_input.cpp:418](../src/touch_input.cpp)) and
the bool is consumed against a resume that never happened — the two drift apart. Clearing
both keeps them coupled.

Ordering is load-bearing and matches the parent plan: `cleanupDirectWriteState(true)` runs
**first** (it does the correct paired decrement for the normal case), `touchForceResume()`
runs **later** as the belt-and-braces reset. Do not reorder.

Add a debug-only assertion after the call that `s_epd_refresh_suspend == 0`.

### 4.3 `buzzerForceStop()` / `ledForceStop()`

Thin public wrappers, no behaviour change:

```c
// buzzer_control.cpp / buzzer_control.h
void buzzerForceStop(void) { buzzer_stop_internal(); }
```
```c
// device_control.cpp / device_control.h
void ledForceStop(void) { led_stop_internal(false); }   // false: leave configured mode intact
```

`clear_mode=false` matches [device_control.cpp:394](../src/device_control.cpp) and
[:562](../src/device_control.cpp) — an abort stops the *sequence*, it does not reconfigure
the LED. `clear_mode=true` is reserved for an explicit `0x0075` stop.

Both are no-ops when nothing is playing, so they are safe to call unconditionally.

---

## 5. Step 2 — The drain-trap fix `[M5]`

The single live-on-merge correctness fix. Today:

```c
imageDataWritten(NULL, NULL, commandQueue[tail].data, commandQueue[tail].len);   // :415
commandQueue[tail].pending = false;                                              // :416
__atomic_store_n(&commandQueueTail, (tail + 1) % COMMAND_QUEUE_SIZE, RELEASE);   // :417
```

`imageDataWritten` can (after Phase 5/6) call `abortToKnownState` → `flushCommandQueue()`,
which sets `commandQueueTail := commandQueueHead`. Line 417 then **overwrites** that with a
stale `tail+1`, resurrecting every command the flush just discarded.

**Exact placement — between `:415` and `:416`**, breaking *without* the tail store:

```c
{
    // [C6] Clear any flag left over from an abort that ran OUTSIDE a drain --
    // serviceBleDisconnectCleanup() calls abortToKnownState() at main.cpp:370 and
    // :428, neither of which is inside this loop. A stale flag would break the
    // FIRST command of the next drain out without storing the tail, and that same
    // slot would then be dispatched AGAIN on the following pass. See below.
    commandDrainAbortPending = false;

    uint8_t drained = 0;
    while (drained < COMMAND_QUEUE_SIZE) {
        uint8_t tail = __atomic_load_n(&commandQueueTail, __ATOMIC_RELAXED);
        uint8_t head = __atomic_load_n(&commandQueueHead, __ATOMIC_ACQUIRE);
        if (tail == head) break;

        imageDataWritten(NULL, NULL, commandQueue[tail].data, commandQueue[tail].len);

        // [M5] Must sit HERE -- after the dispatch, before the tail store. A flush
        // from handler context already did tail := head; storing tail+1 now would
        // resurrect everything it discarded.
        if (commandDrainAbortPending) {
            commandDrainAbortPending = false;
            break;                    // flushCommandQueue() already advanced the tail
        }
        __atomic_store_n(&commandQueueTail, (uint8_t)((tail + 1) % COMMAND_QUEUE_SIZE), __ATOMIC_RELEASE);
        drained++;
        flushResponseQueueToBle();
    }
}
```

Placing the check *after* the tail store would be wrong twice over: the store already
clobbered the flush, and the slot at `tail` may have been re-filled by the producer.

#### `[C6]` The flag must not survive a pass — double-dispatch hazard

`commandDrainAbortPending` is set by `flushCommandQueue()`, which is called from
`abortToKnownState()`, which D1b calls from `serviceBleDisconnectCleanup()` — and that runs at
[main.cpp:370](../src/main.cpp) (deep-sleep-wake branch, **before** the drain, then `return`s)
and [main.cpp:428](../src/main.cpp) (**after** the drain). Neither is inside the drain loop.

So without the reset above, with a non-empty ring at that moment:

1. Abort sets the flag. No drain consumes it this pass.
2. Next pass the drain dispatches `commandQueue[tail]`, *then* sees the stale flag, clears it,
   and `break`s **without storing the tail**.
3. The pass after that dispatches the **same slot again**.

For `CMD_CONFIG_WRITE`, `CMD_POWER_OFF`, `CMD_DEEP_SLEEP` or `CMD_REBOOT` a double dispatch is
not benign — and it would ship in the same commit as the `[M5]` fix it accompanies. Clearing at
the top of the drain block closes it: the flag then only ever spans the dispatch it was raised
during.

No race: `commandDrainAbortPending` is written only by `flushCommandQueue()` and this reset,
both loop-task-only. The producer never touches it.

**Alternative considered:** scope the flag to an active drain with a separate `commandDrainActive`
bool, so `flushCommandQueue()` raises the abort flag only when a drain is actually running. It is
more explicit but adds a second flag and a second invariant to keep true; the top-of-block reset
achieves the same thing in one line. **Take the reset; note the alternative if a future caller
ever needs to know whether it interrupted a drain.**

**Minor, worth a comment not a fix:** when `flushCommandQueue()` runs from *inside* the drain,
its `dropped` count over-reports by one — the currently-dispatching slot has not had its tail
store yet, so it is still counted as queued. Log-only.

**Delete the `pending` field** from both `CommandQueueItem` and `ResponseQueueItem` in
`structs.h`/`main.h` while here — it has no readers in either ring
(`grep -n 'pending' src/` shows assignments only, at
[esp32_ble_callbacks.h:125](../src/esp32_ble_callbacks.h),
[communication.cpp:119](../src/communication.cpp),
[main.cpp:291](../src/main.cpp), [:309](../src/main.cpp), [:416](../src/main.cpp)). Removing
it saves a byte per slot × 43 slots and, more importantly, removes a field that *looks* like
it participates in the SPSC protocol but does not. Update `tools/od-device-cli.py` only if
these structs are mirrored there — they are not (that file mirrors config packets, not
runtime rings), so no CLI change.

**Also fold in the parent plan's `[H1]` comment fix while in `main.h`** — the
[main.h:365-370](../src/main.h) comment claims 33 slots hold "a full W=32 window + END",
but the producer refuses at `nextHead == tail`, so usable capacity is 32. Fix the comment
here (a comment-only change, safe in Phase 3); the actual `COMMAND_QUEUE_SIZE` bump to 34
stays in Phase 7 where the DRAM decision belongs. **Decision D5.**

---

## 6. Step 3 — `g_commandInFlight` depth counter `[H4]`

Declared in `session_guard.h`, defined in `session_guard.cpp`:

```c
extern volatile uint8_t g_commandInFlight;
```

Incremented/decremented around the `imageDataWritten` body — **inside** `imageDataWritten`
itself (`communication.cpp`), not at each call site, so every dispatcher (ESP32 drain, nRF
inline callback, LAN frame dispatch) is covered by construction. Use an RAII guard or a
single-exit `goto done` — `imageDataWritten` has multiple `return`s, so verify every path
decrements.

Why a counter and not a bool (review `[H4]`): Bluefruit's `ada_callback_invoke()` falls back
to invoking the write callback **inline on the BLE event task** when `rtos_malloc` fails
(`BLECharacteristic.cpp:538-542`). Heap pressure during a large nRF52840 transfer is exactly
when that happens, so "one task on nRF" is not an invariant. With a bool, whichever
invocation nests out first clears it and the supervisor could abort under a live handler.

Phase 3 only *maintains* the counter. Phase 6 consumes it ("abort only when depth is 0").
Phase 5 consumes it for `nrfSessionClearPending`.

Increment/decrement need not be atomic RMW on either target (single writer per nesting
level, and the nested case is same-core), but mark it `volatile` and add a comment saying
so, rather than leaving the reader to work it out.

---

## 7. Step 4 — Queue flushes

Both defined in `main.cpp` (where the rings live), declared in **`session_guard.h`** (§3a),
bodies guarded `#ifdef TARGET_ESP32`. `session_guard.cpp` calls them through that declaration —
it must never touch the rings directly, or it drags NimBLE types into a nRF build.

```c
#ifdef TARGET_ESP32
// Loop-task only. SPSC-safe: commandQueueTail has exactly one writer (the consumer,
// i.e. this task), so snapshotting head into tail cannot race the producer's head
// store. Discards payloads without dispatching them.
void flushCommandQueue(void) {
    uint8_t head = __atomic_load_n(&commandQueueHead, __ATOMIC_ACQUIRE);
    uint8_t tail = __atomic_load_n(&commandQueueTail, __ATOMIC_RELAXED);
    if (tail == head) return;
    uint8_t dropped = (head - tail + COMMAND_QUEUE_SIZE) % COMMAND_QUEUE_SIZE;
    __atomic_store_n(&commandQueueTail, head, __ATOMIC_RELEASE);
    // Breaks an in-progress drain (§5). Harmless when no drain is running: the drain
    // block clears this at its top, so a flag set from serviceBleDisconnectCleanup()
    // cannot leak into the next pass and double-dispatch a slot -- see [C6] in §5.
    commandDrainAbortPending = true;
    od_log_warn("Command queue flushed (%u dropped)", dropped);
}

// Both head and tail are loop-task-only, so this is trivially safe.
void flushResponseQueue(void) {
    if (responseQueueTail == responseQueueHead) return;
    uint8_t dropped = (responseQueueHead - responseQueueTail + RESPONSE_QUEUE_SIZE) % RESPONSE_QUEUE_SIZE;
    responseQueueTail = responseQueueHead;
    od_log_warn("Response queue flushed (%u dropped)", dropped);
}
#endif
```

On nRF both are **empty out-of-line definitions in `main.cpp`** — `void flushCommandQueue(void) {}` —
so `session_guard.cpp` stays free of `#ifdef` clutter at the call sites. Deliberately *not*
`static inline` in a header: `main.h` cannot be included by `session_guard.cpp` at all (§3a), and
a `static inline` there would be invisible to it.

**Callable only from the loop task — documented, not asserted (D4).** No `configASSERT`, no
`g_loopTaskHandle` capture, no `OD_DEBUG_ASSERTS` block. The invariant holds structurally today
(every ESP32 handler runs on the loop task), so the enforcement is a doc comment carrying the
full reasoning. **D4 specifies the exact comment text — use it verbatim; it is the only thing
protecting this property.**

---

## 8. Step 5 — `src/session_guard.h` / `.cpp`

### 8.1 Header surface

```c
#pragma once
#include <stdint.h>
#include <stdbool.h>

// --- flags, serviced on the loop task ---
extern volatile bool commandDrainAbortPending;    // set by flushCommandQueue(), consumed by the drain
extern volatile bool commandQueueOverflowAbort;   // set by onWrite on ring-full (Phase 7 consumes)
extern volatile bool responseQueueOverflowAbort;  // set by sendResponse on ring-full (Phase 7 consumes)
extern volatile bool epdForceOffPending;          // deferred panel off — a refresh/stream must finish first (8.3.1)

// --- in-flight depth (H4) ---
extern volatile uint8_t g_commandInFlight;

// --- progress accounting (Phase 6 consumes) ---
extern volatile uint32_t g_lastProgressMs;
void markSessionProgress(void);

// --- the recovery ---
void abortToKnownState(const char* reason, bool dropLink);
```

No NimBLE, no `WiFiClient`, no Arduino `String`. The link drop is delegated (§8.4) so the
header stays portable across all eleven envs.

### 8.2 `markSessionProgress()`

```c
void markSessionProgress(void) { g_lastProgressMs = millis(); }
```

Phase 3 defines the storage and the setter. **Whether Phase 3 also installs the six stamp
call sites is Decision D2.** The sites, per `[C1]` (recorded here so they are not re-derived
in Phase 6):

1. `pipeState.expected_seq` advancing — inside the in-order accept, `display_service.cpp`
2. `directWriteBytesWritten` increasing
3. `chunkedWriteState.receivedChunks` incrementing — `communication.cpp:565`
4. `partialCtx` byte counter advancing
5. Refresh completion
6. `handleAuthenticate` success

**Never** on command dispatch, notify, or LAN frame dispatch — that is the defect `[C1]`
exists to prevent (a post-`clearEncryptionSession` `RESP_AUTH_REQUIRED` retry flood is a
dispatch *and* a notify per retry, and would keep the stamp fresh forever in exactly the
wedge the supervisor exists to catch).

### 8.3 `abortToKnownState()`

```c
void abortToKnownState(const char* reason, bool dropLink) {
    // 1. LOG FIRST — before any state is destroyed, so the log line describes the
    //    wedge and not the aftermath.
    od_log_error("ABORT: %s (dropLink=%d) transfer=%d direct=%d pipe=%d partial=%d "
                 "chunked=%d refresh=%d inflight=%u",
                 reason, (int)dropLink, (int)transferActive(), (int)directWriteActive,
                 (int)pipeState.active, (int)partialCtx.active,
                 (int)chunkedWriteState.active, (int)epdRefreshInProgress,
                 (unsigned)g_commandInFlight);

    // 2. Discard queued responses. No client-facing abort frame is sent — sending one
    //    would need either a new response shape (forbidden, §2) or repurposing an
    //    existing code (changes its documented meaning, also forbidden). The client's
    //    own timeout is the notification. See Decision D3.
    flushResponseQueue();

    // 3. Discard undispatched commands (also raises commandDrainAbortPending)
    flushCommandQueue();

    // 4. Transfer state, in dependency order
    cleanupDirectWriteState(true);        // paired touch resume + panel release
    cleanupPartialWriteOnDisconnect();    // 0x76 / pipe-partial session bookkeeping
    resetPipeWriteState();                // pipe + reorder queue
    resetChunkedWriteState();             // NEW (§4.1)

    // 5. Belt-and-braces peripheral reset
    touchForceResume();                   // NEW (§4.2) — after cleanupDirectWriteState
    buzzerForceStop();                    // NEW (§4.3)
    ledForceStop();                       // NEW (§4.3)

    // 6. Panel power. HARD INVARIANT: a refresh in flight ALWAYS runs to completion,
    //    and an in-flight controller stream is never cut mid-write (D7 hazard 1).
    //    Deferred, NOT abandoned -- serviceDeferredPanelOff() completes it.
    if (!epdRefreshInProgress && !epdStreamInProgress) {
        epdSessionForceOff();
    } else {
        epdForceOffPending = true;
        od_log_warn("ABORT: %s in progress — panel force-off deferred, not skipped",
                    epdRefreshInProgress ? "refresh" : "stream");
    }

    // 7. Crypto + link (D6c). The condition is the invariant, not the caller's
    //    opinion: a cleared session under a LIVE link is invisible to the client --
    //    it keeps sending encrypted frames that all bounce 0xFE and never
    //    re-authenticates mid-stream. Clear only when the link is going or gone.
    if (dropLink || !linkIsUp()) {
        clearEncryptionSession();
    }
    if (dropLink) {
        odLinkDropRequest();              // deferred; serviced in loop()
    }

    // 8. Owner token release — Phase 4 fills this in (no-op stub in Phase 3)
    linkReleaseIfHeld();

    markSessionProgress();                // clean slate: do not re-fire immediately
}
```

**The ordering defect in the parent plan, and how D3 dissolves it.** The parent plan orders
the teardown "optional client NACK → … → flush response ring". On ESP32 `sendResponse`
*enqueues* ([communication.cpp:112-120](../src/communication.cpp)) and `flushResponseQueue()`
*discards*, so NACK-then-flush would throw the NACK away and the client would learn nothing.
The fix was going to be an inversion (flush, then NACK, then `flushResponseQueueToBle()`).

D3 removes the notification entirely, so the ordering question disappears with it: the
response flush is unconditional and nothing is queued after it. Recorded because the defect is
real and will resurface if the pipe-only NACK follow-up in D3 is ever implemented — that
sender must run **after** the flush, not before.

**Idempotence.** Every callee is already idempotent (`cleanupDirectWriteState` no-ops when
`!directWriteActive`, `epdSessionForceOff` is documented idempotent at
[display_service.h:21](../src/display_service.h), the flushes early-return on empty).

**Re-entrancy guard must be ATOMIC (per D7).** A plain `static bool inAbort` is not enough:
D7 settles that `abortToKnownState` is **never gated**, so on nRF two tasks can enter it
concurrently and both read `false`. Use a test-and-set:

```c
static volatile uint8_t inAbort = 0;
if (__atomic_exchange_n(&inAbort, 1, __ATOMIC_ACQ_REL)) {
    od_log_warn("ABORT: re-entered (%s) — first pass owns the teardown", reason);
    return;
}
// ... teardown ...
__atomic_store_n(&inAbort, 0, __ATOMIC_RELEASE);
```

The guard is also needed for the plain nested case: step 4 can, through
`cleanupDirectWriteState`, reach code that could later grow an abort call.

**Never gated.** `abortToKnownState` runs whenever it is called, at any in-flight depth, on
either task. The `g_commandInFlight` check lives in nRF `loop()`-side **callers** (§9.2), never
here — see D7 for the research behind that, and copy D7's header comment onto the function so
Phase 6 does not reintroduce a gate.

#### 8.3.1 Invariant: a panel refresh always completes

**`abortToKnownState` must never interrupt an e-paper refresh.** This ranks above the teardown's
own urgency, and it is the one place where the abort defers rather than acts. Three reasons it
is not negotiable:

1. **An interrupted refresh damages the image, and on some panels the panel.** Cutting the rail
   or sleeping the controller mid-waveform leaves a partially-driven frame — ghosting, or a
   latched pixel state that the next full refresh has to clear. On 3/4/7-colour Spectra the
   waveform runs 30-60 s and `bbepWaitBusy` caps at 30 000 ms (`bb_ep.inl:3959-3975`); there is
   no safe abort point inside it.
2. **The device is not wedged during a refresh — it is working.** A refresh is the successful
   end of a transfer, not a symptom. Aborting one converts a completing operation into a failed
   one and forces the client to re-push the whole image.
3. **`displayed_etag` correctness.** A refresh that is cut partway leaves the panel showing
   neither the old nor the new image while RTC state may already claim the new etag, so the next
   push can be skipped as a no-op against an image that was never actually drawn.

**Deferred means deferred, not skipped.** The abort sets `epdForceOffPending` and returns; the
rest of the teardown (queues, transfer state, touch, buzzer/LED, session) completes immediately,
because none of it touches the panel. A loop-serviced tick then finishes the job the moment the
refresh does:

```c
// main.cpp — called from loop() on both targets, next to serviceBleDisconnectCleanup()
static void serviceDeferredPanelOff(void) {
    if (!epdForceOffPending) return;
    if (epdRefreshInProgress || epdStreamInProgress) return;   // still busy — try next pass
    epdForceOffPending = false;
    od_log_info("Deferred panel force-off completing (refresh/stream finished)");
    epdSessionForceOff();
}
```

Note the ESP32 disconnect path defers the **whole** teardown while a refresh is in flight
([main.cpp:322](../src/main.cpp)) and, per §9.2, nRF now does the same. So on the D1b disconnect
path the refresh is protected twice over. `epdForceOffPending` covers the callers that do *not*
defer wholesale — Phase 6's supervisor and Phase 7's idle timeout — where the abort itself must
run promptly but the panel must still be left alone.

**Do not add a timeout to this deferral in Phase 3.** The refresh is already bounded: Phase 2
`[X3]` makes `fastepd_wait_refresh` honour its timeout, `waitforrefresh(60)` bounds the bbep
path, and `bbepWaitBusy` caps at 30 s. If those bounds hold, `epdForceOffPending` clears within
a minute; if they do not, the bug is in the bound, not here. A timeout here would reintroduce
exactly the mid-refresh cut this invariant exists to prevent.

**Re-entrancy from the drain.** `abortToKnownState` will (Phase 5/6) be reached from
*inside* `imageDataWritten`, i.e. mid-drain. That is precisely why step 3 sets
`commandDrainAbortPending` rather than relying on the tail store — see §5.

### 8.4 `odLinkDropRequest()` — deferred, not inline

Dropping the link from inside `abortToKnownState` would mean calling
`pServer->disconnect()` (NimBLE) or `Bluefruit.disconnect()` from a header shared with an
env that `lib_ignore`s NimBLE. Instead:

```c
// session_guard.cpp — portable
void odLinkDropRequest(void) { g_linkDropPending = true; }
```

and a `serviceLinkDrop()` in `main.cpp` under the existing per-target `#ifdef`, called from
`loop()` next to `serviceBleDisconnectCleanup()`. Phase 4 extends it with the owner token
and the `BLE_ERR_REM_USER_CONN_TERM` (0x13) reason code + return check `[C3]`.

This also fixes a subtler problem: an inline disconnect from the NimBLE host task while
`abortToKnownState` is mid-teardown would fire `onDisconnect` → `bleDisconnectCleanupPending`
→ a *second* teardown pass. Deferring keeps it to one.

---

## 9. Step 6 — Wiring the two disconnect teardowns (D1b)

### 9.1 ESP32 — `serviceBleDisconnectCleanup()`

Replace [main.cpp:343-347](../src/main.cpp):

```c
    if (directWriteActive) cleanupDirectWriteState(true);
    cleanupPartialWriteOnDisconnect();
    resetPipeWriteState();
```

with a single call:

```c
    abortToKnownState("disconnect", /*dropLink=*/false);
```

Three things must hold, and each is a review point:

- **The `ownerStillUp` early-return at [main.cpp:328-338](../src/main.cpp) stays strictly in
  front of the call.** This is not optional. `disconnectWiFiServer()` raises the *same*
  `bleDisconnectCleanupPending` flag ([wifi_service.cpp:812](../src/wifi_service.cpp)), and it
  is called from the WiFi-lost tick at [main.cpp:447](../src/main.cpp) — so on a WiFi-enabled
  env, losing WiFi routes into this function while a BLE client is mid-transfer. The guard is
  what stops that from tearing the BLE transfer down. Wiring the abort in front of the guard
  would create the exact bug Phase 4 `[C4]` exists to fix.
- **`dropLink=false`, always.** The link is already gone by definition on this path; requesting
  a drop would queue a disconnect against a dead handle.
- **Do not move the guard out of `#ifdef OPENDISPLAY_HAS_WIFI` yet.** That is Phase 4 `[C4]`
  and it is tied to the refused-gatecrasher handle discrimination, which does not exist in
  Phase 3. On `esp32-N4` (no WiFi) the guard is absent, but so is the LAN path that raises the
  flag spuriously — the flag is only ever set by a genuine BLE disconnect there, so Phase 3 is
  safe without it. **Recheck this the moment Phase 4 lands.**

Net behavioural delta on ESP32: the teardown additionally resets chunked-config state, force-
resumes touch, stops buzzer/LED, and force-offs the panel when no refresh is in flight. All are
either no-ops or strictly correct on a link that just went away.

### 9.2 nRF — needs a new deferral, not a direct substitution

`disconnect_callback` ([device_control.cpp:227-240](../src/device_control.cpp)) runs on the
Bluefruit **Callback** task, not `loop()`. Today it calls three cleanup functions inline from
that task. A naive substitution would additionally run `epdSessionForceOff()` (bbepSleep →
`bbepWaitBusy` → SPI teardown → rail cut) and — via **D6** — `clearEncryptionSession()` from
the callback task.

Both are unsafe there, for reasons already established in this plan family:

- `epdSessionForceOff()` is precisely the heavyweight, SPI-touching work ESP32 defers to
  `loop()` for ("the session teardown below … is heavyweight, state-mutating work that races
  loop()'s SPI streaming", [esp32_ble_callbacks.h:62-68](../src/esp32_ble_callbacks.h)). nRF has
  no equivalent excuse to run it inline.
- `clearEncryptionSession()`'s `memset(session_key, 0, 16)` from a non-loop task is the `[H4]`
  race verbatim: Bluefruit's `ada_callback_invoke()` falls back to invoking the write callback
  inline on the BLE task when `rtos_malloc` fails, so `aes_ccm_decrypt` can be running
  concurrently.

**So nRF gets the same flag-and-defer shape ESP32 already has:**

```c
// device_control.cpp — disconnect_callback, now flag-only
void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
    od_log_info("=== BLE CLIENT DISCONNECTED === reason: %u", reason);
    nrfDisconnectCleanupPending = true;      // NEW
}
```

```c
// main.cpp, nRF arm of loop() — next to the other loop-serviced work
if (nrfDisconnectCleanupPending && !epdRefreshInProgress && g_commandInFlight == 0) {
    nrfDisconnectCleanupPending = false;
    abortToKnownState("disconnect", /*dropLink=*/false);
}
```

The `g_commandInFlight == 0` term is what makes the deferral actually close `[H4]` rather than
just move it: it guarantees no `imageDataWritten` is on the stack (on either task) when the
session key is zeroed. This is the one place in Phase 3 where the depth counter from §6 is
*read* rather than merely maintained — everywhere else it is Phase 5/6's to consume.

**This pulls part of `[H4]` forward from Phase 5 into Phase 3.** That is a deliberate
consequence of D1b: wiring the caller early means the safety property the caller depends on has
to come with it. Phase 5's `nrfSessionClearPending` becomes redundant — record that there so it
is not implemented twice.

Secondary benefit: nRF gains the `epdRefreshInProgress` deferral it does not have today. A
disconnect mid-refresh currently runs `cleanupDirectWriteState(true)` straight through the
refresh on nRF; after this it waits, matching ESP32.

### 9.3 D1b × D6 — Phase 3 now delivers the BLE-disconnect session clear

With D6(c)'s `dropLink || !linkIsUp()` condition, `abortToKnownState("disconnect", false)`
evaluates `!linkIsUp()` → true (the link is gone) → clears the session. That is the parent
plan's confirmed user decision *"clear encryption session on BLE disconnect"*, which the parent
plan scheduled for **Phase 5**.

**One documented exception:** on a WiFi-enabled env with a LAN client connected, `linkIsUp()`
returns true and the session is not cleared — `EncryptionSession` records no origin, so Phase 3
cannot tell whose session it is. Deliberate and safe (erring the other way would destroy a live
LAN session from a BLE event); Phase 4's owner token closes it. Full reasoning in D6.

This is a scope gain, not a scope creep — the behaviour is required, it is one condition, and
D1b makes it fall out for free. But it must be recorded so Phase 5 does not re-implement it:

| Parent-plan Phase 5 item | Status after Phase 3 + D1b |
|---|---|
| Clear session on BLE disconnect | **Delivered here.** Phase 5 verifies, does not re-add. |
| nRF deferred session clear `[H4]` | **Delivered here** (§9.2). `nrfSessionClearPending` is redundant. |
| `[H2]` clear placed after the `ownerStillUp` guard | **Satisfied here** by §9.1's ordering rule. Phase 5 re-checks it under the owner token. |
| Guard *inside* `clearEncryptionSession()` (dead-session-with-live-link) | **Still Phase 5.** Phase 3 relies on the call-site condition; Phase 5 makes it un-regressable. |
| `session_timeout_seconds` expiry disabled | **Still Phase 5.** Untouched here. |
| Pipe NACK latch / `error_since_ms` | **Still Phase 5.** Untouched here. |

**Risk this introduces:** the session clear goes live one phase earlier than planned, without
Phase 5's in-function guard as a backstop. The mitigation is that both call sites pass
`dropLink=false` on a path where the link is provably down, so the "dead session under a live
link" failure mode is unreachable from Phase 3's callers. Any *new* caller added before Phase 5
must be checked against that by hand.

---

## 10. Build guards — the eleven-env matrix

`session_guard.cpp` compiles into every env. The rules:

| Symbol | Guard |
|---|---|
| `commandQueue` / `responseQueue` / `pServer` | `#ifdef TARGET_ESP32`, and reached **only** through `session_guard.h`-declared functions defined in `main.cpp` — never named in `session_guard.cpp` (§3a) |
| Any LAN call (`wifiLanClientConnected`, `opendisplay_lan_*`) | `#ifdef OPENDISPLAY_HAS_WIFI` — **never** `TARGET_ESP32`; `esp32-N4` is ESP32 without WiFi |
| Any NimBLE type | must not appear in `session_guard.h` at all (nRF sets `lib_ignore = NimBLE-Arduino`) |
| FastEPD-only symbols | `#ifdef OPENDISPLAY_FASTEPD` |

RAM cost: `commandDrainAbortPending` + 2 overflow flags + `g_commandInFlight` +
`g_linkDropPending` + `epdForceOffPending` + `epdStreamInProgress` + `sessionScrubPending`
(8 bytes) + `g_lastProgressMs` (4 bytes) = **12 bytes `.bss`**, minus the `pending`-field
removal (−43 bytes across both rings). Net negative. `esp32-N4` — the
DRAM-tight env that already needs `PIPE_SMALL_DRAM_WINDOW` ([structs.h:45-48](../src/structs.h))
— is not at risk from Phase 3. (It *is* the gate for Phase 1's `replay_window[256]`; unrelated.)

---

## 11. Implementation order (each step independently buildable)

| # | Step | Files | Live on merge? |
|---|---|---|---|
| 1 | `resetChunkedWriteState()` + consolidate 4 clear sites | `communication.cpp`, `communication.h` | yes (fixes the partial reset at :574) |
| 2 | `touchForceResume()` / `touchForceResumeAll()` | `touch_input.*`, `display_service.*` | not yet |
| 3 | `buzzerForceStop()` / `ledForceStop()` wrappers | `buzzer_control.*`, `device_control.*` | not yet |
| 4 | Delete `pending` from both ring structs; fix the `main.h:365` capacity comment | `structs.h`, `main.h`, `main.cpp`, `communication.cpp`, `esp32_ble_callbacks.h` | yes |
| 5 | `g_commandInFlight` depth counter | `session_guard.*`, `communication.cpp` | yes — read by step 9 |
| 6 | `flushCommandQueue()` / `flushResponseQueue()` + nRF no-ops | `main.cpp`, `session_guard.h` | not yet |
| 7 | Drain-trap fix `[M5]` | `main.cpp` | **yes** |
| 8 | `session_guard.h/.cpp`: flags, `markSessionProgress`, `abortToKnownState`, `odLinkDropRequest` | new | not yet |
| 9 | `serviceLinkDrop()` + **(D6c) `linkIsUp()`** per-target | `main.cpp`, `session_guard.h` | not yet |
| 9a | **(D7) `epdStreamInProgress`** set/cleared at the two streaming choke points | `display_service.cpp/.h` | not yet |
| 9b | **(D7/§8.3.1) `epdForceOffPending` + `serviceDeferredPanelOff()` in `loop()`** | `session_guard.*`, `main.cpp` | not yet |
| 9c | **(D7) `clearEncryptionSession()` invalidate/scrub split** + `sessionScrubPending` service | `encryption.cpp`, `main.cpp` | **yes — changes existing behaviour** |
| 10 | **(D1b) Wire ESP32 `serviceBleDisconnectCleanup` → `abortToKnownState` (§9.1)** | `main.cpp` | **yes — steps 1-9 all go live here** |
| 11 | **(D1b) nRF: flag-only `disconnect_callback` + loop-serviced abort (§9.2)** | `device_control.cpp`, `main.cpp`, `session_guard.h` | **yes** |

**Commit split.** Steps 1–9 as one commit (mechanically inert: new helpers, new file, one
correctness fix — nothing calls the teardown yet). Steps 10–11 as a second commit, which is the
one that changes runtime behaviour on every disconnect and is therefore the one to review
hardest and bisect to if a soak regresses.

Step 11 is deliberately last: it is the only step that changes *which task* work runs on, and
it depends on step 5's depth counter existing. Do not merge 11 without 5.

Per D2, the six `markSessionProgress()` stamp sites are **not** in this list — they land in
Phase 6 with the rest of `[C1]`.

Build after each:

```bash
pio run -e nrf52840custom -e esp32-s3-N16R8 -e esp32-c3-N16 -e esp32-c6-N4 -e esp32-N4
```

CI builds all eleven on push — `esp32-N4` (no WiFi) and `nrf52840custom` (no NimBLE, no
rings) are the two that catch guard mistakes.

---

## 12. Verification

**Wire-protocol constraint (§2) — run first, before anything else is called done:**

```bash
cd ../opendisplay-protocol && tools/sync_protocol_header.py --check --only Firmware   # must pass, unchanged
cd ../Firmware && git diff main --stat -- include/opendisplay_protocol.h include/opendisplay_structs.h   # must be empty
```

Phase 3 touches neither file, so both must be clean on the first run — if either reports a
diff, something in the implementation drifted out of scope. Two Phase-3-specific additions to
the check:

```bash
git diff main -- src/ | grep -nE '^\+.*(RESP_|CMD_)[A-Z_]+ *=' # must be empty: no new opcode/response values
git diff main --stat -- docs/pipe-write-protocol.md            # must be empty in Phase 3 (§5.1 note is Phase 5)
```

**Build:** all eleven envs. Specifically confirm `esp32-N4` compiles `session_guard.cpp`
with no LAN symbols and `nrf52840custom` with no NimBLE symbols.

**Static:** `grep -n 'pending' src/` returns nothing in ring context after step 4.

**Client compatibility (the constraint's actual purpose):** an unmodified `py-opendisplay`
and an unmodified HA integration must both drive a full transfer against Phase 3 firmware
with no change and no new warning. Since Phase 3 sends no new frame and removes no existing
one, a passing pre-Phase-3 transfer must pass identically — any behavioural difference
visible to the client is a constraint violation, not a Phase 3 feature.

**Bench (no hardware needed):**
- Force `abortToKnownState("test", false)` from a debug command with no transfer active →
  logs once, every subsystem no-ops, device still serves commands afterwards.
- Call it twice back to back → second call is a clean no-op (idempotence + re-entrancy guard).

**D1b — the disconnect path is now the primary test surface.** Every one of these is a
connect/disconnect cycle, so they are cheap to run and must all pass on **both** targets:

| Case | Expected |
|---|---|
| Connect, do nothing, disconnect | One `ABORT: disconnect` line, all state flags false, advertising resumes |
| Disconnect **mid-pipe-transfer** | Pipe + reorder queue cleared, panel rail down, touch responsive, next transfer clean |
| Disconnect **mid-chunked-config-write** | `chunkedWriteState.active` false (this is new — the old teardown left it set) |
| Disconnect **during a refresh** | Teardown **deferred**, refresh completes intact, teardown runs on a later pass. On nRF this is new behaviour (§9.2) |
| **Supervisor/idle abort during a refresh** (Phase 6/7 trigger, forced here with a debug opcode) | Refresh completes **untouched**; abort tears down queues/state immediately; `epdForceOffPending` logs, then `serviceDeferredPanelOff()` powers the panel down after the refresh ends. **This is the §8.3.1 invariant test — the panel must never be cut mid-waveform** |
| Abort during PIPE streaming (mid-`bbepWriteData`) | Rail stays up (`epdStreamInProgress`), no SPI/CS activity against a dead panel, force-off completes after the stream unwinds |
| Abort mid-decrypt on nRF (heap-pressure double-handler) | `isAuthenticated()` false immediately; in-flight decrypt completes with an intact key; scrub lands on a later loop pass. No `integrity_failures` bump from a half-zeroed key |
| Disconnect with buzzer/LED active | Both stop immediately (new) |
| Disconnect, then reconnect and re-auth | Succeeds — session cleared via `!linkIsUp()` (D6c), client gets a fresh challenge |
| Disconnect, reconnect, replay a captured pre-disconnect frame | **Rejected.** Session was cleared, so there is no key to validate against. Depends on Phase 1 — run it on the merged Phase 1+3 tree |
| **WiFi env only:** LAN client connected, BLE disconnects, then BLE reconnects | Session **not** cleared (documented D6 gap) → the reconnecting BLE client re-authenticates and gets a fresh session anyway. Confirm no `0xFE` bounce loop |
| **WiFi env only:** BLE client transferring, WiFi drops | BLE transfer **survives** — `ownerStillUp` guard held (§9.1). This is the regression test for the guard-ordering rule |
| **WiFi env only:** LAN client transferring, BLE client disconnects | LAN transfer survives |
| **`esp32-N4` only:** BLE disconnect | Teardown runs (no guard present, none needed — no LAN path raises the flag) |

**nRF-specific, for §9.2:** confirm via log timestamps that the teardown runs on the loop task
and not the callback task — the `ABORT:` line must appear after a `loop()` boundary marker, not
interleaved with the disconnect callback's own logging. Then repeat the disconnect-mid-transfer
case under heap pressure (large transfer) to exercise the `g_commandInFlight` gate.

**Bench — `[C6]` double-dispatch regression test.** Queue several commands, then trigger an
abort from *outside* the drain (a disconnect is the natural way, since D1b wires it). Confirm
from the log that after the flush **no command is dispatched twice** across the following two
loop passes. Without the top-of-block reset, the first command of the next drain runs on two
consecutive passes. Use a command with a visible side effect (LED or buzzer) rather than a
silent one.

**Hardware — the drain-trap regression test (step 7's reason for existing):** start a
PIPE_WRITE with a deep in-flight window (W=32), trigger an abort from *inside* a dispatched
command (temporary debug opcode that calls `abortToKnownState`), and confirm via log that no
queued command is dispatched after the flush line. Without the fix, `drained` continues and
stale commands execute; with it, the drain breaks immediately.

Note that D1b does **not** exercise this: the disconnect path calls `abortToKnownState` from
`serviceBleDisconnectCleanup`, which runs *outside* the drain loop, so the tail-store clobber
never arises there. The drain trap still needs its own deliberate trigger.

**Hardware — teardown completeness:** mid-transfer abort, then verify by observation:
touch responds again (`s_epd_refresh_suspend == 0`), buzzer silent, LED off, panel rail
down, `transferActive()` false, a fresh transfer starts clean.

**Hardware — refresh protection:** abort during a Spectra full refresh → log shows "panel
force-off deferred", refresh completes intact, panel powers down normally afterwards. This
depends on Phase 2's `[X2]` (`epdRefreshInProgress` set around the boot-refresh paths) being
landed first.

---

## 13. Decisions

**All decisions settled — no blockers remain.**

| # | Decision | Where it lands |
|---|---|---|
| D1 | (b) wire both disconnect teardowns now | §9 (whole section exists for it) |
| D2 | (b) all of `[C1]` in one Phase 6 commit | §8.2 defines storage only |
| D3 | (c) send nothing — no client-facing abort frame | §2, §8.3 (step 3 deleted) |
| D4 | document only, no assert | §7 comment text |
| D5 | fix the `main.h:365` comment now; size bump stays Phase 7 | step 4 |
| D6 | (c) derive as `dropLink \|\| !linkIsUp()` | §8.3 step 7 |
| D7 | never gate `abortToKnownState` | §8.3 atomic guard, §9.2 caller-side check |

### D1 — Land `abortToKnownState()` with no callers, or wire one now? `(SETTLED — (b))`

**Decision: (b) — wire the two existing disconnect teardowns now.** Replace
[main.cpp:343-347](../src/main.cpp) and [device_control.cpp:237-239](../src/device_control.cpp)
with `abortToKnownState("disconnect", false)`. This is the same set of calls plus the new
chunked/touch/buzzer/LED/panel resets — a strict superset of today's behaviour — and it gets
the teardown exercised on every disconnect immediately rather than leaving it untested in the
tree until Phase 5.

Rejected: **(a) land dead** (parent plan as written) — clean phase boundaries, but an
unexercised teardown is exactly the kind of code that is wrong on first use; **(c) debug-only
trigger** — tests a synthetic path and then deletes the only caller.

**Full implementation consequences are in §9**, which exists because of this decision. In
brief: the ESP32 `ownerStillUp` guard must stay in front of the call (§9.1); the nRF site needs
a new loop-serviced deferral rather than a direct substitution, which pulls part of `[H4]`
forward from Phase 5 (§9.2); and combined with D6 this delivers the BLE-disconnect session
clear one phase early (§9.3).

### D2 — Do the six `markSessionProgress()` stamp sites land in Phase 3 or Phase 6? `(SETTLED — (b))`

**Decision: (b) — everything `[C1]` lands in one Phase 6 commit.** `[C1]` is the
highest-consequence finding in the review ("progress means the state machine advanced — never
'a command arrived' or 'a notify succeeded'"), and it reads and reviews far better as a single
self-contained change than as storage in one phase and semantics in another.

Rejected: **(a) stamps in Phase 3** — mechanically harmless (unread stamps change no
behaviour), but it splits one finding across two phases and two reviews, and a reviewer looking
at the Phase 6 diff would not see the stamp placement that is the entire point of `[C1]`.

**What Phase 3 still owes Phase 6**, so the split is clean:
- `g_lastProgressMs` storage and the `markSessionProgress()` setter (§8.2) — defined, callable,
  unused.
- The six stamp sites are *enumerated* in §8.2 so Phase 6 does not re-derive them from `[C1]`.
- One live call: `abortToKnownState()` ends with `markSessionProgress()` (§8.3) so a completed
  teardown leaves a clean slate and the Phase 6 supervisor cannot re-fire immediately on a
  wedge it just cleared.

**Consequence to accept:** Phase 3 as merged has `g_lastProgressMs` written by exactly one
caller and read by none. That is intentional dead storage, not an oversight — do not "clean it
up" before Phase 6, and do not let a linter strip it.

### D3 — What does the "optional client NACK" actually send? `(SETTLED by the hard constraint — §2)`

The parent plan says "optional client NACK (skip when dropping link)" without specifying the
frame. The wire-protocol constraint removes most of the option space:

- ~~**(b) A generic `{RESP_NACK, 0x00, reason_code}`**~~ — **forbidden.** A new response shape
  is a protocol change, and it would need a py-opendisplay change to interpret. Ruled out by
  §2, not by preference.
- **(a) Reuse `sendPipeNack(err)`** when `pipeState.active`, nothing otherwise. Permitted in
  principle — reusing an existing code is in bounds *"as long as the code's documented meaning
  is unchanged"* — but only for a genuine pipe failure. It may **not** be sent for a
  chunked-config or direct-write abort, which would repurpose `0x81` and change its documented
  meaning. It also has side effects: it sets `pipeState.error` and calls
  `cleanupDirectWriteState`/`cleanup_partial_write_state` itself
  ([display_service.cpp:2564-2578](../src/display_service.cpp)), duplicating step 5. Would need
  a payload-only variant.
- **(c) Send nothing.** The client already treats silence as a timeout
  (`TIMEOUT_PIPE_DATA_COMPRESSED = 5.0` × `MAX_PTO = 3` ≈ 15 s) and every `0x81` NACK as
  immediately fatal — it never re-reads the ACK position after one (established in `[L2]`).

**Decision: (c). Step 3 of `abortToKnownState` sends nothing, and the `dropLink` parameter no
longer gates a notification.** The client derives no benefit it does not already get from its
own timeout, and (c) is the only option with zero protocol surface. This means **step 3 does
not exist** — the teardown is flush → flush → state reset, with no client-facing frame.

If a fast-fail is later shown to matter on hardware, the follow-up is (a) narrowed to the pipe
case only: factor a `pipeBuildAckPayload`-only sender out of `sendPipeNack` so it has no side
effects, and call it **only** when `pipeState.active && !pipeState.error`. That stays inside
the constraint. It is explicitly not Phase 3 work.

**Simplification this unlocks:** with no notification, `dropLink` now controls exactly two
things — the session clear (D6) and `odLinkDropRequest()`. Update the `abortToKnownState`
sketch in §8.3 to drop step 3 and its `flushResponseQueueToBle()`; the response-ring flush in
step 2 becomes unconditional and final.

### D4 — Enforce "loop task only" with an assert, or document it? `(SETTLED — document only, no code)`

**Decision: document the invariant, add no assert and no task-handle capture.** No
`configASSERT`, no `g_loopTaskHandle`, no `OD_DEBUG_ASSERTS` block — §7's sketch drops its
assert line entirely.

Rationale: the invariant currently holds *structurally* rather than by enforcement. On ESP32
every handler runs on the loop task (`onWrite` only enqueues,
[esp32_ble_callbacks.h:118-128](../src/esp32_ble_callbacks.h); the drain dispatches,
[main.cpp:408-421](../src/main.cpp); LAN dispatch is also in `loop()`), so there is no existing
caller that could violate it — an assert would guard against a caller that does not exist. On
nRF both flushes are empty stubs, so the invariant is vacuous there. Adding a task handle and a
debug-only branch to protect a structurally-guaranteed property is cost without a current
benefit.

**What this costs, stated honestly:** the property becomes convention-enforced. D7 makes that
slightly sharper — `abortToKnownState` is now explicitly callable from any task, and it calls
`flushCommandQueue()`. Today that is still safe on ESP32 (every ESP32 abort caller is on the
loop task, because every ESP32 handler is), but the *function* no longer advertises a
task restriction that its *caller* does not. The doc comment has to carry that weight.

**Required comment on `flushCommandQueue()`** — it must state the invariant, why it holds, and
what breaks if it stops holding, because nothing else will:

```c
// LOOP TASK ONLY. Not asserted -- enforced by structure, not by code (plan D4).
//
// SPSC safety: commandQueueTail has exactly ONE writer (the consumer). Snapshotting
// head into tail is safe only from that consumer. Today every ESP32 caller is on the
// loop task because every ESP32 command handler is -- onWrite() only enqueues, the
// drain and LAN dispatch both run in loop(). abortToKnownState() is callable from any
// task by design (D7), but on ESP32 it is only ever REACHED from loop-task contexts.
//
// If a future change dispatches commands from the NimBLE host task, or calls
// abortToKnownState() from one, this becomes a genuine two-writer race on
// commandQueueTail -- silently resurrecting or double-dispatching queued commands.
// Add a task assert then; do not assume this comment still describes reality.
```

Mirror a shorter version on `flushResponseQueue()` (both head and tail are loop-task-only, so
its invariant is stronger and simpler).

**Escalation trigger for a future phase:** if Phase 4 or later adds any cross-task
`abortToKnownState` caller on ESP32, reopen D4 and add the assert — that is the condition under
which the structural guarantee lapses.

### D5 — Fix the `main.h:365` capacity comment in Phase 3, or leave it to Phase 7? `(SETTLED — fix now)`

**Decision: fix the comment in Phase 3 (step 4); the `COMMAND_QUEUE_SIZE` → 34 bump stays in
Phase 7.** Phase 3 already edits `CommandQueueItem` two lines below to remove `pending`, so
leaving a false capacity claim in the block being touched is the worst of both options.

The comment at [main.h:365-370](../src/main.h) currently claims 33 slots hold "a full W=32
in-flight window + END". They do not: the producer refuses at `nextHead == tail`
([esp32_ble_callbacks.h:121-122](../src/esp32_ble_callbacks.h)), so usable capacity is
`COMMAND_QUEUE_SIZE - 1 = 32` — a full window with **no** room for END. Finding `[H1]`.

Rewrite it to state the ring-capacity rule explicitly, the true usable depth, and that the
shortfall is deliberate-for-now with the fix scheduled:

```c
// Usable capacity is COMMAND_QUEUE_SIZE - 1 = 32: the SPSC producer refuses at
// nextHead == tail, so one slot is always reserved to distinguish full from empty.
// 32 holds a full W=32 in-flight PIPE_WRITE window but leaves NO room for the END
// frame -- a sustained full window can drop END, which the client recovers from via
// SACK retransmit (pipe-write-protocol.md 5.2). Phase 7 bumps this to 34 on envs with
// DRAM to spare (NOT esp32-N4). Sized for a 60 s Spectra SPI stall (loop blocked in
// bbepWriteData). OD_BLE_MAX_FRAME (256) covers pipe <=244, legacy <=232, HA <=244.
```

**Do not change the value to 34 here.** That is a DRAM decision per env and `esp32-N4` — which
already needs `PIPE_SMALL_DRAM_WINDOW` ([structs.h:45-48](../src/structs.h)) — must be excluded.
It belongs with Phase 7's queue-full handling, where the overflow policy that makes the extra
slot meaningful also lands.

**The duplicate definition is a trap here.** `COMMAND_QUEUE_SIZE 33` is defined in **two**
places — [main.h:371](../src/main.h) and [esp32_ble_callbacks.h:19](../src/esp32_ble_callbacks.h)
(under `#ifndef`). Phase 3 only touches the comment so no drift is possible now, but Phase 7
must change both or the `#ifndef` will silently keep 33 depending on include order. Flag it in
the comment so Phase 7 cannot miss it.

### D6 — Should `dropLink=false` ever clear the encryption session? `(SETTLED — (c) derive it)`

**Decision: (c) — derive the clear from `dropLink || !linkIsUp()`.** It encodes the actual
invariant ("never leave a dead session under a live link") in one place instead of making every
caller re-derive it, and it composes with Phase 5's guard inside `clearEncryptionSession()`
itself rather than duplicating it.

Rejected: **(a) a third `clearSession` parameter** — pushes a safety-critical judgement onto
every future caller, and the one thing D1b showed is that callers get added faster than the
plan expects; **(b) leave the clear out entirely** — same problem, plus it silently drops the
confirmed user decision on the disconnect path.

Amend §8.3 step 7:

```c
    // 7. Crypto + link. The condition is the invariant, not the caller's opinion:
    //    a cleared session under a LIVE link is invisible to the client -- it keeps
    //    sending encrypted frames that all bounce 0xFE and never re-authenticates
    //    mid-stream. So clear only when the link is going away or already gone.
    if (dropLink || !linkIsUp()) {
        clearEncryptionSession();
    }
    if (dropLink) {
        odLinkDropRequest();              // deferred; serviced in loop()
    }
```

#### `linkIsUp()` — scope, and the gap it leaves

Portable predicate, declared in **`session_guard.h`** (§3a — *not* `main.h`),
`#ifdef`-implemented per target in `main.cpp` like `serviceLinkDrop()` (§8.4). The declaration
must not pull NimBLE or WiFi types into the shared header; both live in the `main.cpp` body.

```c
// main.cpp, TARGET_ESP32
bool linkIsUp(void) {
    if (pServer != nullptr && pServer->getConnectedCount() > 0) return true;
#ifdef OPENDISPLAY_HAS_WIFI
    if (wifiLanClientConnected()) return true;      // wifi_service.cpp:355
#endif
    return false;
}
```
```c
// main.cpp, TARGET_NRF
bool linkIsUp(void) { return Bluefruit.connected() > 0; }
```

**This is "any transport", not "the session's transport", and that is forced.**
`EncryptionSession` ([encryption_state.h:11-30](../src/encryption_state.h)) has **no origin
field** — there is nothing recording which transport a session belongs to. `g_commandOrigin`
([communication.cpp:37](../src/communication.cpp)) is per-dispatch, not per-session, and
`transferSessionOrigin()` ([display_service.cpp:2116](../src/display_service.cpp)) tracks the
*transfer*, not the session. So in Phase 3 the question "is the link that owns this session
still up?" is genuinely unanswerable. This is the parent plan's wedge mechanism #4
(cross-transport session clobber), and Phase 4's owner token is its fix.

**The resulting gap, stated plainly:** on a WiFi-enabled env, if BLE disconnects while a LAN
client is connected, `linkIsUp()` returns true and the session is **not** cleared — so the
confirmed user decision "clear encryption session on BLE disconnect" is missed in that one
case. This is the deliberate, safe direction to err:

- Erring toward *not* clearing risks a stale session surviving a BLE disconnect. Bounded: a
  reconnecting client resets its counter to 0 and is rejected as out-of-window, and Phase 1
  closes the `counter_diff == 0` replay hole that would otherwise make the surviving session
  exploitable. **Phase 3 must land after Phase 1** — already the stated order — and this is now
  a second reason why.
- Erring toward *clearing* would destroy a live LAN client's session from a BLE event, which is
  wedge mechanism #4 firing in the opposite direction — the exact bug Phase 4 exists to fix, and
  strictly worse than a stale session.

**Phase 4 closes the gap** by scoping `linkIsUp()` to the session's owner
(`linkOwner() == OWNER_BLE ? bleUp() : lanUp()`). Record it there as a required follow-up, not
as an optional refinement.

**Rejected shortcut:** adding an origin byte to `EncryptionSession` now. It is firmware-local
(`src/encryption_state.h`) so it is *not* a wire-protocol change and would be in bounds — but
it is Phase 4's owner token wearing a different hat, and building half of it here guarantees
two half-mechanisms to reconcile later. Wait for the token.

### D7 — Does `abortToKnownState` respect `g_commandInFlight`, or is that Phase 6's job? `(RESEARCHED — ungating is safe, with two conditions)`

**Desired behaviour: `abortToKnownState` is never gated — when called, it runs.** This section
is the adversarial check on that: what was searched for, what was found, and what it costs.

Phase 6 says "abort only when the in-flight depth counter is 0". Taken literally as a
precondition *inside* `abortToKnownState`, that makes it un-callable from its most important
callers: Phase 5's `integrity_failures >= 3` and `reloadConfigAfterSave` triggers both fire from
*inside* `imageDataWritten`, where depth is ≥1 by construction. A recovery mechanism that
refuses to run precisely when a command is wedged is not a recovery mechanism.

#### The key reframing

**Depth is a proxy for the wrong thing.** The hazards below are all *cross-task concurrency*
hazards, not *stack depth* hazards. The distinction decides the question:

- **ESP32: depth ≥ 1 always means same-task.** Handlers run only on the loop task — `onWrite`
  merely enqueues ([esp32_ble_callbacks.h:118-128](../src/esp32_ble_callbacks.h)), the drain
  dispatches ([main.cpp:408-421](../src/main.cpp)), and LAN dispatch is also in `loop()`. A
  nested abort is therefore strictly sequential with the handler that called it. **No
  concurrency exists, so there is nothing for a depth gate to protect.**
- **nRF: depth ≥ 1 may mean another task.** `imageDataWritten` runs on the Bluefruit Callback
  task, and `[H4]`'s `rtos_malloc`-failure fallback can run a *second* copy inline on the BLE
  task. Here concurrency is real — but it is real between **loop() and a handler**, which is a
  property of the *caller's* context, not of the callee.

#### Hazards found (reasons not to ungate), ranked

**1. CRITICAL, nRF only — rail cut mid-SPI-write.** `pwrmgmLock` protects state *transitions*
only, not streaming: `epdSessionAcquire` releases at [display_service.cpp:488](../src/display_service.cpp)
before returning, and every `bbepWriteData` call ([:1990](../src/display_service.cpp),
[:2003](../src/display_service.cpp), [:2309](../src/display_service.cpp),
[:2624](../src/display_service.cpp), [:3223](../src/display_service.cpp)) runs unlocked. So
`abortToKnownState` → `cleanupDirectWriteState(true)` → `epdSessionForceOff()` acquires a *free*
lock and executes `bbepSleep` + `pwrmgm(false)` — dropping the rail while another task drives
the same SPI bus and CS. This is exactly the hazard ESP32's deferral comment cites
([esp32_ble_callbacks.h:62-68](../src/esp32_ble_callbacks.h)). **Real, not theoretical.**

**2. HIGH, nRF only — session key zeroed mid-decrypt.** `[H4]` verbatim: `clearEncryptionSession()`'s
`memset(session_key, 0, 16)` ([encryption.cpp:205](../src/encryption.cpp)) landing inside a
concurrent `aes_ccm_decrypt`. Produces a spurious tag failure → `integrity_failures++` → possibly
a second clear. Incorrect, not memory-unsafe.

**3. MEDIUM, both targets — the calling handler continues on reset state.** A nested abort
returns into its caller, which keeps running against zeroed state. Audited for the two Phase 5
triggers and both are benign: `handleWriteConfigChunk`'s completion path
([communication.cpp:566-576](../src/communication.cpp)) calls `reloadConfigAfterSave()`, then
`sendResponse()` (queues into a just-flushed ring — the response still goes out, which is
correct) and re-clears three `chunkedWriteState` fields that abort already cleared (idempotent).
`decryptCommand`'s failure path returns straight out to a `sendResponseUnencrypted` + `return`.
**This must be re-audited per trigger in Phase 5/6, not assumed.**

**4. LOW, nRF only — loop stall, not deadlock.** If the Callback task holds `pwrmgmLock` inside
`epdSessionAcquire`'s `bbepSendCMDSequence`, an abort from `loop()` spins in `pwrmgmLockTake`
with `delay(1)`. Bounded by Phase 2 `[C2]`'s 60 s deadline — and abort must then handle the
`false` return, which is a Phase 2 dependency, not a reason to gate.

#### Hazards searched for and NOT found

These are the ones that would have forced a gate. They are absent, and that is what makes
ungating defensible:

- **No use-after-free anywhere in the teardown.** Every buffer it touches is static or
  file-static: `pipeReorder` ([display_service.cpp:576](../src/display_service.cpp)),
  `pipeState` ([:575](../src/display_service.cpp)), `partialCtx` ([:556](../src/display_service.cpp)),
  `chunkedWriteState` ([main.h:283](../src/main.h)). `grep -n 'free(\|delete ' src/display_service.cpp`
  returns nothing. **Worst case is stale or inconsistent state — never memory unsafety.** A
  corrupted transfer is recoverable by the very mechanism doing the corrupting; a heap fault is
  not. This is the single strongest argument for ungating.
- **No self-deadlock on `pwrmgmLock`.** All four take/give pairs
  ([:439-488](../src/display_service.cpp), [:496-509](../src/display_service.cpp),
  [:513-515](../src/display_service.cpp), [:520-526](../src/display_service.cpp)) are contained
  within a single function; the lock is never held across a return into handler code. A nested
  abort can always acquire it. Checked specifically because the lock is non-recursive.
- **No new corruption in the double-handler case.** Where two `imageDataWritten` copies run
  concurrently on nRF, `plaintext[512]` and `decrypted_data[512]` are **`static`**
  ([communication.cpp:680](../src/communication.cpp), [:704](../src/communication.cpp)) and
  already race, independent of abort. That scenario is pre-existing and unsafe on its own terms;
  abort neither creates nor meaningfully worsens it. **Out of Phase 3's scope — do not try to
  fix it here, but do not let it be cited as a cost of ungating either.**

#### Accommodating hazard 1 (rail cut mid-SPI-write)

The caller-side depth check (condition 2 below) covers the *common* case, but it is not
sufficient on its own, and relying on it alone would be the weak point of this design. Two gaps:

- A **handler-context** caller (`integrity_failures`, `reloadConfigAfterSave`) does not check
  depth — correctly, since it is on the handler's own task. But in `[H4]`'s double-handler case
  (two `imageDataWritten` copies on nRF after an `rtos_malloc` failure), handler A can abort
  while handler B streams. Depth is 2 and nobody is checking.
- The check is a *caller* discipline. Phase 6 adds another caller, Phase 7 adds two more. One
  omission reintroduces the hazard silently.

**So do not rely on the gate. Close it structurally, at the panel.**

`abortToKnownState` already refuses to force the panel off during a refresh (§8.3 step 6). The
gap is that `epdRefreshInProgress` covers only the *refresh* — it is set at
[display_service.cpp:2415](../src/display_service.cpp) and cleared at
[:2436](../src/display_service.cpp), around `bbepRefresh` + `waitforrefresh` — and **not** the
streaming that precedes it. Extend the same idea to streaming:

```c
// display_service.cpp — new, next to epdRefreshInProgress
volatile bool epdStreamInProgress = false;
```

Set and clear it around the two choke points every controller write funnels through, so one flag
covers pipe, partial, compressed, legacy, FastEPD and E1004:

| Choke point | Covers |
|---|---|
| `pipeConsumePayload()` ([display_service.cpp:2600](../src/display_service.cpp)) | all PIPE paths — partial (`partial_consume_bytes`), compressed (`handleDirectWriteCompressedData`), raw (`streamGray4Bytes` / `directWriteSinkBytes` / `fastepd_direct_write_chunk`) |
| the legacy `0x0071` data handler | legacy direct-write streaming |

Then §8.3 step 6 becomes:

```c
    // 6. Panel power — NEVER interrupt a refresh OR an in-flight controller stream.
    //    Cutting the rail under bbepWriteData drives SPI/CS against a dead panel.
    //    pwrmgmLock does NOT protect this: epdSessionAcquire releases at :488 and all
    //    streaming runs unlocked, so the lock would be free and the force-off would
    //    succeed -- which is exactly the bug.
    if (!epdRefreshInProgress && !epdStreamInProgress) {
        epdSessionForceOff();
    } else {
        epdForceOffPending = true;   // completed later by serviceDeferredPanelOff()
        od_log_warn("ABORT: %s in progress — panel force-off deferred, not skipped",
                    epdRefreshInProgress ? "refresh" : "stream");
    }
```

**What happens to the deferred force-off.** It is **completed, not abandoned** —
`serviceDeferredPanelOff()` (§8.3.1) retries from `loop()` every pass until the refresh or
stream ends. The rest of the teardown does not wait on it: queues, transfer state, touch,
buzzer/LED and the session are all torn down immediately, since none of them touch the panel.

Three existing mechanisms remain as backstops if the pending flag were somehow lost: the
keep-alive deadline (`epdSessionTick`, [display_service.cpp:517-526](../src/display_service.cpp)),
the direct-write watchdog ([main.cpp:436-442](../src/main.cpp)), and the next transfer's
`epdSessionAcquire`.

> ⚠️ **Updated 2026-07-26 — Phase 2 scope cut.** This previously said "reuse Phase 2 `[C2]`'s
> `panelStateUnknown` flag rather than adding a second one." **P2-1 was dropped, so no such flag
> exists.** `pwrmgmLockTake()` keeps its unbounded spin and its `void` signature, and produces no
> signal for `abortToKnownState` to report. Phase 3 must either define its own flag if it wants one,
> or — preferably — drop panel-state reporting from `abortToKnownState`'s remit entirely, since the
> condition it was to report can no longer be detected. The three backstops above are unaffected.

**Cost:** one `volatile bool`, two set/clear pairs, one extra term in an existing condition. No
locking, no serialization, no change to the streaming hot path.

#### Accommodating hazard 2 (session key zeroed mid-decrypt)

Same principle: do not gate the abort, make the operation safe.

`clearEncryptionSession()` ([encryption.cpp:201-219](../src/encryption.cpp)) does two different
jobs in one function, and only one of them is racy:

| Job | Racy? |
|---|---|
| `authenticated = false`, `nonce_counter`/`last_seen_counter`/`integrity_failures`/`session_start_time`/`last_activity`/`auth_attempts`/`server_nonce_time` = 0 | **No.** Scalar stores. A concurrent reader sees old or new, both coherent. |
| `memset` of `session_key`, `client_nonce`, `server_nonce`, `pending_server_nonce`, `replay_window`; `ccm_session_free()` | **Yes.** A 16-byte buffer half-zeroed under `aes_ccm_decrypt` yields garbage plaintext; `ccm_session_free` frees an mbedTLS context that may be in use. |

**Split it: invalidate now, scrub later.**

```c
void clearEncryptionSession(void) {
    // Invalidate IMMEDIATELY -- this is the security-relevant half and it is
    // race-free (plain scalar stores). isAuthenticated() goes false at once, so
    // every subsequent command is refused regardless of when the scrub lands.
    encryptionSession.authenticated       = false;
    encryptionSession.session_start_time  = 0;
    encryptionSession.nonce_counter       = 0;
    encryptionSession.last_seen_counter   = 0;
    encryptionSession.integrity_failures  = 0;
    encryptionSession.last_activity       = 0;
    encryptionSession.auth_attempts       = 0;
    encryptionSession.server_nonce_time   = 0;

    // Scrub the key material + CCM context. Safe inline whenever no command can be
    // executing concurrently; deferred otherwise. See sessionScrubPending below.
    if (sessionScrubIsSafeNow()) sessionScrubNow();
    else                         sessionScrubPending = true;
    od_log_info("Encryption session cleared (scrub %s)",
                sessionScrubPending ? "deferred" : "done");
}
```

```c
// ESP32: always safe -- all handlers run on the loop task, so a caller of
// clearEncryptionSession() is never concurrent with a decrypt.
// nRF: safe only at depth 0.
static inline bool sessionScrubIsSafeNow(void) {
#ifdef TARGET_ESP32
    return true;
#else
    return g_commandInFlight == 0;
#endif
}
```

`sessionScrubPending` is serviced from `loop()` when `g_commandInFlight == 0` — the same
condition and the same place as §9.2's nRF disconnect service, so it is one mechanism, not two.

**Why deferring the scrub is not a security regression.** Three reasons, in order of weight:

1. **`isAuthenticated()` is already false.** [communication.cpp:663-670](../src/communication.cpp)
   gates on it *before* `decryptCommand` is ever reached, so no new command can use the key —
   the window is closed the instant the invalidate half runs.
2. **The one in-flight decrypt completes correctly rather than incorrectly.** It already passed
   the auth gate before the clear, so it was going to run either way. With an intact key it
   produces valid plaintext and dispatches a slightly stale command; with a half-zeroed key it
   produces garbage, fails the CCM tag, and increments `integrity_failures` — which under
   Phase 5 triggers *another* clear. **The racing memset is the worse outcome, not the safer
   one.**
3. **The scrub is anti-forensic hygiene, not access control.** It bounds how long key bytes sit
   in `.bss`. Deferring it by one loop pass (microseconds to milliseconds) does not change the
   threat model. Note also that deep sleep wipes `encryptionSession` outright — it is plain
   `.bss` at [main.h:289](../src/main.h), not `RTC_DATA_ATTR`.

**Phase 5 interaction:** Phase 5 adds a guard *inside* `clearEncryptionSession()` (raise a flag
when clearing under a live link). That guard goes in the **invalidate** half, which always runs
inline — it must not end up behind the deferred scrub.

#### Decision

**Ungate `abortToKnownState` — it always runs when called — subject to two conditions:**

1. **The re-entrancy guard must be atomic, not a plain `static bool`.** §8.3 specifies
   `static bool inAbort`; ungated cross-task entry means two tasks can both read `false` and
   both proceed. Use `__atomic_exchange_n(&inAbort, 1, __ATOMIC_ACQ_REL)` and return if it was
   already 1. **This is a required amendment to §8.3.**

   Plus the two structural accommodations above, which are what actually make ungating safe:
   `epdStreamInProgress` gating the panel force-off (hazard 1), and the invalidate/scrub split
   in `clearEncryptionSession()` (hazard 2). **Neither is optional** — with them, the
   caller-side check below is defence in depth rather than the only defence.
2. **Every `loop()`-side caller on nRF carries the depth check itself**, as defence in depth.
   The hazard is loop-aborting-under-a-live-handler, so the check belongs where that context is
   known — exactly the shape §9.2 already specifies for the nRF disconnect service
   (`nrfDisconnectCleanupPending && !epdRefreshInProgress && g_commandInFlight == 0`). Phase 6's
   supervisor arm on nRF must carry the same term. ESP32 callers need no such check, because
   ESP32 has no cross-task handler execution.

Handler-context callers (Phase 5's `integrity_failures`, `reloadConfigAfterSave`) call
**unconditionally on both targets** — they are on the handler's own task, so there is no
concurrency for a gate to prevent.

Record this in `abortToKnownState`'s header comment so Phase 6 does not "restore" the gate:

```c
// NEVER gated. Callable from any context, at any in-flight depth, on either task.
// Re-entrancy is handled internally by an atomic guard. The g_commandInFlight
// check is the CALLER's responsibility and ONLY on nRF, ONLY from loop()-side
// callers (disconnect service, supervisor) -- because the hazard is cross-task
// concurrency, not stack depth. ESP32 runs all handlers on the loop task, so a
// nested abort there is strictly sequential and needs no check. Do not move the
// depth check in here: it would disable abort for the in-handler triggers
// (integrity_failures, reloadConfigAfterSave) that need it most. See plan D7.
```

---

## 14. Out of scope for Phase 3 (recorded so review does not re-litigate)

- Owner token / `linkClaim()` / `linkRelease()` — Phase 4. `linkReleaseIfHeld()` is a no-op
  stub here.
- The `ownerStillUp` guard move out of `#ifdef OPENDISPLAY_HAS_WIFI` `[C4]` — Phase 4.
- Command-ring overflow *policy* (`[H1]`: log-and-drop, escalate only via the supervisor) —
  Phase 7. Phase 3 only declares the two overflow flags.
- The supervisor predicate and the wall-clock backstops `[H3]` — Phase 6.
- `COMMAND_QUEUE_SIZE` 33 → 34 — Phase 7.
- Any change to `sendPipeNack`'s latch semantics or the `error_since_ms` field — Phase 5.
- The `docs/pipe-write-protocol.md` §5.1 documentation note the parent plan permits — Phase 5;
  it describes the pipe error-release deadline, which Phase 3 does not implement.
- The pipe-only abort NACK (D3's follow-up) — deferred indefinitely, pending hardware evidence
  that the client's own timeout is insufficient.
- Anything that would edit `include/opendisplay_protocol.h`, `include/opendisplay_structs.h`,
  or push through `../opendisplay-protocol` — out of bounds for **every** phase (§2), not just
  this one.
