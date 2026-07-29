# Plan — orphaned transfer state: heal it, don't idle on it

**Date:** 2026-07-29
**Branch:** `fix/loop-hang-3`
**Follows:** `4d37d43` *fix(ble): let a transport event interrupt the cooperative idle wait*

Part (b) of the three-part fix for the ~40 s post-disconnect park. Parts (a) and
(c) landed in `4d37d43`.

### Revision history

This plan has been rewritten twice under review, and both rewrites changed the
conclusion rather than the wording. Recorded because the reasoning matters more
than the diff.

**Draft 1** — amend `transferActive()` to exclude `pipeState.error`, add it and
`ble.eventPending()` to `workInFlight`. *Refuted:* the 15-minute direct-write
watchdog orphans full-PIPE state in a form the exclusion does not catch, so the
predicate would still latch true forever. Split into two commits, the first a
standalone bug fix.

**Draft 2** — Commit 1 (the watchdog fix) landed as `77c2226`; Commit 2 kept the
gate terms. *Refuted:* `transferActive()` in the gate is redundant in every state
where the transfer can still progress, and actively harmful in the states where
it cannot — it converts a low-power park into up to fifteen minutes of full-CPU
spinning. The invariant it was meant to enforce is better enforced by healing the
orphan than by refusing to sleep on it.

**This draft** keeps `ble.eventPending()`, drops `transferActive()` from the gate,
and replaces it with a self-healing assertion in the watchdog.

---

## Commit 1 — landed as `77c2226`

`fix(pipe): terminate the pipe session when the transfer watchdog fires`

The direct-write watchdog released the panel but left `pipeState.active` set with
`pipeState.error` false. Because the `0x0081` handler gates on `pipeState` alone,
a timed-out full PIPE kept accepting frames into a torn-down session — and since
the cleanup zeroes the byte counters, the uncompressed auto-complete test read
`0 >= 0` and drove `bbepRefresh()` + `waitforrefresh(60)` at an unpowered panel.

Both watchdogs now live in `display_service.cpp` behind `checkTransferTimeouts()`
and share `TRANSFER_WATCHDOG_MS`. Full reasoning is in the commit message; it is
not repeated here.

---

## Commit 2 (revised) — `fix(loop): heal orphaned transfer state; count pending events as work`

### Why `transferActive()` does not belong in the gate

Draft 2 argued the gate was "wrong about what constitutes work" because a
half-finished transfer with the panel powered was not counted. That framing does
not survive contact with the state space. **A transfer whose transport is gone
cannot progress** — frames arrive only over BLE or LAN — so enumerate every state
in which `transferActive()` would be true:

| State | What the gate does today | What the term would add |
|---|---|---|
| Owner connected, transfer live | `ble.isConnected()` / `wifiLanSession` already true | Nothing — redundant |
| Owner disconnecting | `serviceBleDisconnectCleanup()` runs at `main.cpp:619`, **before** the gate at :665, and resets all three flags. Deferral needs `epdRefreshInProgress`, refusal needs `ownerStillUp` — both already gate terms | Nothing — redundant |
| Orphaned: state set, transport gone | Gate false → `platformIdle()` → park (nRF) or deep sleep (ESP32) | Holds the gate until the 15-minute watchdog |

The third row is the only distinct behaviour, and it is a regression:

- `workInFlight` true takes the `delay(1)` path. On nRF that is **one tick**,
  below `configEXPECTED_IDLE_TIME_BEFORE_SLEEP` (2), and the core's
  `vApplicationIdleHook` is an empty weak stub — so it does not sleep. The idle
  task spins at 64 MHz and the loop body re-runs ~1000×/s.
- The watchdog measures from START, so the spin lasts *15 minutes minus however
  long the transfer already ran*.
- Cost per event: **~0.9–1.5 mAh** on nRF52840 (≈1.5–2 days of CR2450 standby),
  **~6–12 mAh** on an ESP32-S3 tag with WiFi+BLE up.
- Worse, it removes an existing recovery. On battery ESP32 an orphan is
  *self-healing today*, because deep sleep is a reboot and all transfer state is
  plain RAM. The term converts a self-healing state into a fifteen-minute awake
  one.

So the term buys invariant-hardening at the price of the only states it applies
to. The right response to "this state should not exist" is to remove the state,
not to refuse to sleep while it exists.

### The change

**1. `main.cpp` — one gate term, not two:**

```diff
 const bool workInFlight = bleRxQueuePending() || bleTxQueuePending() ||
                           ble.isConnected() ||
+                          ble.eventPending() ||
                           s_advertisingRestartPending ||
                           epdRefreshInProgress ||
                           wifiLanSession;
```

`eventPending()` closes the real gap: an event raised after `serviceBleEvents()`
ran in this pass is otherwise invisible until the next one, and the pass is about
to park. This defers idle by exactly one pass, deliberately.

**2. `display_service.cpp` — heal the orphan in `checkTransferTimeouts()`:**

```c
// Commit 77c2226 proved a live pipe session always has a hardware half, and
// closed the one path that broke it. This asserts it at runtime rather than
// resting on the proof: any future path that recreates the orphan gets one log
// line and a reset, instead of a session that silently accepts 0x0081 frames
// into torn-down state. Deliberately not a gate term -- a transfer whose
// transport is gone cannot progress, so refusing to sleep on it burns power for
// work that will never happen.
if (pipeState.active && !pipeState.error && !directWriteActive && !partialCtx.active) {
    od_log_error("ERROR: orphaned pipe session (no hardware half) - resetting");
    resetPipeWriteState();
}
```

**3. `display_service.cpp` — drop the `> 0` timestamp guards:**

```c
if (directWriteActive && directWriteStartTime > 0)          // -> drop "&& ... > 0"
if (partialCtx.active && partialCtx.start_time > 0 && ...)  // -> drop "&& ... > 0"
```

The `active` flag is set in straight-line code eleven lines from the `millis()`
stamp with no return between, so it already implies a valid timestamp. The guard
is not merely redundant: a transfer starting in the ~1 ms window where `millis()`
wraps through zero has its watchdog disabled **permanently**. Odds are of order
1 in 10⁹ transfers — this is a free removal of a reasoning burden, not a live
risk, and it matters because item 2 makes the watchdog the backstop for the
orphan assertion.

**4. Split the two questions `transferActive()` currently answers.** It has five
callers asking two different things:

| Caller | Question | Wants |
|---|---|---|
| `workInFlight` (proposed) | is live work in flight? | — *not adding it, see above* |
| `touch_input.cpp:587` | may I poll GT911 over I2C? | live work only |
| `wifi_service.cpp:697` | may I run a full-channel scan? | live work only |
| `display_service.cpp:1921,1925` | would logging this frame spam? | **any** stream, including a dead one still receiving frames |

Amend `transferActive()` to `directWriteActive || partialCtx.active ||
(pipeState.active && !pipeState.error)` for the first three, and give the
log-quieting predicates their own broader test that keeps the errored case quiet.

Without the split, a fatal NACK un-suppresses every remaining in-flight `0x0081`
frame — up to a full window from a compliant client, unbounded from one that
ignores the NACK until END — at ~90 frames/s, two log lines each
(`bleRxQueuePush()` arrival + dispatch banner), evicting the NACK itself from the
ring. That is the opposite of the diagnostic improvement Draft 2 claimed.

Note also that `imageWriteLogQuietFrame()` is called from `bleRxQueuePush()`,
which runs on the **callback task** — so `transferActive()` is already read
cross-task. Keeping the logging predicate separate avoids widening that read to
`pipeState.error`.

**5. Optional, same commit or its own:** make `takeConnectedEvent()` /
`takeDisconnectedEvent()` atomic read-and-clear (`__atomic_exchange_n`). The
current check-then-clear can lose a whole event, not just its payload — and a
lost disconnect means no `s_disconnectCleanupPending` *and* no
`s_advertisingRestartPending`, so the radio never re-arms. Instruction-scale
window, but a one-line fix.

### Proof obligations

| Term | Set by | Cleared by | Cannot latch because |
|---|---|---|---|
| `ble.eventPending()` | stack callbacks | `take*Event()` at the next loop top | The peek clears nothing; the take always runs. Continuous new events are ongoing work, not a latch |
| orphan assertion | n/a — it is the clear | itself | Runs every pass, unconditionally |

No transfer flag enters the gate, so no transfer flag can veto sleep. That is the
point of this revision.

### ESP32 deep sleep

`ble.eventPending()` defers `platformIdle()` by one pass when an event lands
after `serviceBleEvents()`. That is the term's purpose and its entire effect.
Nothing else changes: no transfer state reaches the gate, `lastActivityMs` still
supplies the quiet window, and a deep-sleep wake is a full reboot, so no state
here survives it.

### Test

1. **`eventPending()`.** Raise a connect or disconnect after `serviceBleEvents()`
   has run and before the gate is evaluated; assert the pass takes `delay(1)` and
   the next pass consumes the event. On hardware, the observable is a
   mid-transfer disconnect being serviced one pass later rather than after a
   `CHECK_INTERVAL_MS`.
2. **Orphan assertion.** Fault-inject the orphan (clear `directWriteActive`
   without resetting the pipe), then assert: one `ERROR:` line, pipe state
   cleared, a subsequent `0x0081` frame rejected, and the device idling/sleeping
   normally. Confirm it does *not* fire in ordinary operation — a full transfer,
   a partial transfer, a fatal NACK, and a mid-transfer disconnect should each
   complete with the assertion silent.
3. **Watchdog guards.** Regression only: both watchdogs still fire at 15 minutes.
4. **Log split.** Force a fatal NACK mid-stream with frames still in flight;
   assert the NACK line survives in the ring and the per-frame lines stay
   suppressed.
5. **nRF idle current**, disconnected, before and after. Expected unchanged; a
   delta means something reaches the gate that should not.
6. **Build matrix**, all 11 environments.

---

## Residuals — not in this commit

1. **Duration, not idle.** Both watchdogs measure from START, so a genuinely slow
   15-minute push is aborted mid-flight. Converting to an idle timer (stamp on
   each accepted frame) is a behaviour change deserving its own argument.
2. **A fatally-NACKed pipe session is remembered indefinitely.** `PipeWriteState`
   has no timestamp, so nothing bounds it; harmless, since it is excluded from
   every live-work predicate and bounded in practice by the next
   START/END/disconnect.
3. **Cleanup is dropped, not deferred.** `serviceBleDisconnectCleanup()` clears
   `s_disconnectCleanupPending` *before* the `ownerStillUp` test and returns, so
   when the skip fires that disconnect's teardown never runs. Less severe now
   that no transfer flag vetoes sleep, but still a silent drop.
4. **`sessionOrigin` is never cleared** — stamped at every START, so a refusal log
   line can cite a transfer that ended long ago.
5. **nRF MSD cadence** is still coupled to the idle duration.
6. **A stale source comment** at `display_service.cpp:2771-2772` still says nRF
   dispatches from its callback task; untrue since Phase 3.

## Risk and rollback

| Risk | Likelihood | Mitigation |
|---|---|---|
| Orphan assertion fires in normal operation | Low | Test 2's negative cases; it logs at ERROR, so a false positive is loud rather than silent |
| Dropping the `> 0` guards changes watchdog timing | None | `active` already implies a valid stamp |
| Log split leaves a case unsuppressed | Low | Test 4 |
| `eventPending()` defers sleep unexpectedly | By design, one pass | Test 5 measures the aggregate |

Every item reverts independently. None depends on `77c2226` except the orphan
assertion, which asserts the invariant that commit established.
