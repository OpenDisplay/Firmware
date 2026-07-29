# Plan — terminate orphaned transfers, then teach `workInFlight` about them

**Date:** 2026-07-29
**Branch:** `fix/loop-hang-3`
**Follows:** `4d37d43` *fix(ble): let a transport event interrupt the cooperative idle wait*

Part (b) of the three-part fix for the ~40 s post-disconnect park. Parts (a) and
(c) landed in `4d37d43`.

**Revision note.** The first draft of this plan proposed a single change: amend
`transferActive()` to exclude `pipeState.error` and add it to `workInFlight`.
Review found that unsafe — the 15-minute direct-write watchdog orphans full-PIPE
state in a form the exclusion does not catch, so the amended predicate would
still latch true forever. The work therefore splits into two commits, the first
of which is a real bug fix that stands on its own.

---

## Commit 1 — `fix(pipe): terminate the pipe session when the transfer watchdog fires`

### The bug (present today, independent of any gate change)

`main.cpp` runs two transfer watchdogs:

```cpp
if (directWriteActive && directWriteStartTime > 0) {
    if (directWriteDuration > 900000UL) {      // 15 min
        cleanupDirectWriteState(true);          // clears directWriteActive ONLY
    }
}
checkPartialWriteTimeout();                     // resets pipe only if pipeState.partial
```

`cleanupDirectWriteState()` contains no reference to `pipeState`. A **full**
(non-partial) PIPE transfer whose client stalls past 15 minutes therefore lands
in a split state:

| Flag | After the watchdog |
|---|---|
| `directWriteActive` | `false` — panel torn down, rail cut, touch resumed |
| `partialCtx.active` | `false` |
| `pipeState.active` | **`true`** |
| `pipeState.error` | **`false`** |

The pipe half survives its own hardware half. The `0x0081` DATA handler gates on
`pipeState`, not on `directWriteActive` (`if (!pipeState.active || pipeState.error) return;`),
so the session keeps accepting frames after its panel is gone. The only exits are
a replacement `0x0080` START, an END, or a disconnect.

**The consequence is worse than lost frames.** `cleanupDirectWriteState()` zeroes
`directWriteTotalBytes`, `directWriteBytesWritten` and `directWriteCompressed`.
That is precisely the state the auto-complete guard already warns about — the
existing comment at `display_service.cpp:2820-2824` reads:

> MUST be gated on `!partial`: a partial transfer never touches `directWrite*`
> (both are 0), so `0>=0` would false-fire a FULL refresh on the very first frame

The watchdog manufactures that same zeroed state for a **full** transfer, where
the `!pipeState.partial` guard does not apply. So on an *uncompressed* full pipe,
the next `0x0081` frame trips `directWriteBytesWritten >= directWriteTotalBytes`
as `0 >= 0` and calls `directWriteFinishAndRefresh()`, which issues
`bbepRefresh()` + `waitforrefresh(60)` **against an unpowered panel with no
`epdSessionAcquire()`**. If the floating BUSY line reads busy, that spins
`delay(10)` for up to **60 seconds inside `loop()`** — the exact class of stall
this branch exists to remove.

On a *compressed* full pipe, auto-complete is gated off, so every frame is
accepted and ACKed with its payload silently discarded, and the eventual `0x0082`
END runs the same bogus refresh — reporting success for an image that was never
written.

Scope: this only bites while the owning client stays connected across the 15
minutes, since `serviceBleDisconnectCleanup()` resets pipe state on disconnect.

The asymmetry exists because the two watchdogs were written in different files:
the partial one sits in `display_service.cpp` beside the state it must clear, the
direct one in `main.cpp`, where reaching that state means going through an
accessor rather than seeing it inline. Nothing prevented it — see the correction
under *The change* — but nothing prompted it either.

### The change

**Correction to an earlier draft:** this plan previously claimed the fix *had* to
move into `display_service.cpp` because `pipeState` is file-static and
unreachable from `main.cpp`. That is wrong. `display_service.h` already declares
both `resetPipeWriteState()` (:64) and `pipeWriteActive()` (:66), and `main.cpp`
already calls the former at :404. The minimal fix is two lines in `loop()`:

```c
cleanupDirectWriteState(true);
if (pipeWriteActive()) resetPipeWriteState();
```

Moving the block is therefore a **cohesion** choice, not a necessity — the two
watchdogs belong together beside the state they terminate, and their separation
is why one of them forgot the pipe. Recommended, but the commit message must not
claim it was required.

**`display_service.h`** — replace the `checkPartialWriteTimeout()` declaration:

```c
// Both transfer watchdogs, together: the direct-write timeout must terminate the
// enclosing pipe session, and keeping the two apart is exactly how it came to
// tear down the panel and leave that session running.
void checkTransferTimeouts(void);
```

**`display_service.cpp`** — the direct block moves in, and gains the pipe
teardown:

```c
void checkTransferTimeouts(void) {
    if (directWriteActive && directWriteStartTime > 0 &&
        (millis() - directWriteStartTime) > TRANSFER_WATCHDOG_MS) {
        od_log_error("ERROR: Direct write timeout (%u ms) - cleaning up stuck state",
                     (unsigned)(millis() - directWriteStartTime));
        cleanupDirectWriteState(true);
        // A full PIPE owns this direct-write session as its hardware half. Reset
        // the pipe with it: leaving pipeState.active set orphans a transfer with
        // no panel, and the 0x0081 handler gates on pipeState, so it would keep
        // accepting frames into a torn-down session. Deliberately NOT folded into
        // cleanupDirectWriteState(), which normal END also calls and where the
        // pipe reset is already sequenced separately.
        if (pipeState.active) resetPipeWriteState();
    }
    checkPartialWriteTimeout();   // unchanged; already resets pipe when partial
}
```

**`main.cpp`** — the two calls collapse to one; the 15-minute literal and the log
line move out with the block.

### Why reset rather than NACK

Disconnect cleanup already resets rather than notifies, and after 15 minutes of
silence there is rarely a client to tell. Reset also cannot fail on a dead link,
where `sendPipeNack()` would queue a response that `serviceBleTx()` discards.

### Test

- **Full PIPE watchdog, uncompressed.** Start an *uncompressed* full PIPE
  transfer, send a few frames, stop, **stay connected**. After 15 minutes assert:
  the timeout logs once, the panel powers down, and a subsequent `0x0081` frame is
  rejected rather than processed. Before this commit that frame trips the `0>=0`
  auto-complete and drives `bbepRefresh()` at a dead panel — run it once against
  unfixed code to confirm the test has teeth, and watch for the up-to-60 s
  `waitforrefresh()` stall.
- **Full PIPE watchdog, compressed.** Same, but assert the eventual `0x0082` END
  does not report success for an image that was never written.
- **Pipe-partial watchdog.** Unchanged behaviour; regression only.
- **Normal END.** Confirm the reordered call site did not disturb the ordinary
  completion path.

### Why it stands alone

It fixes a live defect — frames processed into a torn-down session — with no
dependency on the gate change. It is also the prerequisite for Commit 2.

---

## Commit 2 — `fix(ble): count unconsumed events and live transfers as work`

### What is still wrong after `4d37d43`

```cpp
const bool workInFlight = bleRxQueuePending() || bleTxQueuePending() ||
                          ble.isConnected() ||
                          s_advertisingRestartPending ||
                          epdRefreshInProgress ||
                          wifiLanSession;
```

Two blind spots: an event raised after `serviceBleEvents()` ran in this pass, and
a live transfer with the panel powered. Consequence today is bounded — one
`CHECK_INTERVAL_MS` (~100 ms) rather than the original 40 s, because `4d37d43`
makes the wait interruptible — but the gate is still wrong about what constitutes
work, and it governs ESP32 deep sleep.

### The change

**`display_service.cpp`** — `transferActive()` stops reporting dead sessions:

```c
bool transferActive(void) {
    // pipeState.error means the session is dead but deliberately remembered:
    // sendPipeNack() keeps pipeState so the reported ACK position stays
    // consistent for the client, and the panel has already been released. That
    // is bookkeeping, not work. A genuinely live transfer always has
    // directWriteActive (full pipe/direct START -> directWriteActivatePanel) or
    // partialCtx.active (0x76 / pipe-partial START) set, so excluding the errored
    // case never under-reports -- given Commit 1, which stops the watchdog
    // leaving a non-errored pipe behind with neither hardware flag set.
    return directWriteActive || partialCtx.active ||
           (pipeState.active && !pipeState.error);
}
```

**`main.cpp`** — two terms:

```diff
 const bool workInFlight = bleRxQueuePending() || bleTxQueuePending() ||
                           ble.isConnected() ||
+                          ble.eventPending() ||
                           s_advertisingRestartPending ||
                           epdRefreshInProgress ||
+                          transferActive() ||
                           wifiLanSession;
```

### Why amend `transferActive()` rather than add a gate-only predicate

It has four other callers — `touch_input.cpp:587`, `wifi_service.cpp:697`, and
two log-quieting predicates at `display_service.cpp:1898-1903`. Each asks "is a
transfer in flight?" and each is currently told "yes" by a session that is dead
and whose panel has already been released: touch polling stays suspended, WiFi
roam scans stay blocked, and frames the session is silently discarding are
suppressed from the log rather than shown. Reviewed individually, none of the
four is protected by the current behaviour. Amending fixes the meaning once; a
separate narrow predicate would fork the concept and leave the misuse in place.

### Proof obligations

| Term | Set by | Cleared by | Cannot latch because |
|---|---|---|---|
| `ble.eventPending()` | stack callbacks | `take*Event()` at the next loop top | Survives at most to the next pass; the peek clears nothing, the take always runs |
| `directWriteActive` | `directWriteActivatePanel()` only | `cleanupDirectWriteState()` only — END, NACK, replacement START, refresh completion/failure, disconnect cleanup, watchdog | Single set site, single clear site, watchdog backstop |
| `partialCtx.active` | `0x76` START, pipe-partial START | whole-struct `memset` in `cleanup_partial_write_state()` | Watchdog backstop via `checkPartialWriteTimeout()` |
| `pipeState.active && !pipeState.error` | pipe START only | `resetPipeWriteState()` — END, auto-complete, replacement START, disconnect, **and the watchdog as of Commit 1**; or `pipeState.error` going true | Commit 1 closes the only path that produced a non-errored orphan |

### ESP32 deep sleep — corrected claim

The first draft asserted the new terms cannot change sleep *duration* because
neither touches `lastActivityMs`. That was too strong: `workInFlight` bypasses
`platformIdle()` entirely, so any stuck term is an independent sleep veto
regardless of the quiet window.

The accurate claim: **for normal, fully-observed transport teardown**, sleep
behaviour is unchanged, because a live transfer already held the device awake via
`ble.isConnected()` (BLE-owned) or `wifiLanSession` (LAN-owned), and both go
false at the same point the new term does. What the change does add is a veto on
*stale* transfer state — which is precisely why Commit 1 must land first, and why
the "cleanup is dropped, not deferred" residual below is now a safety
consideration rather than documentation debt.

### The `workInFlight` comment

The existing text — *"Every term is transient and most are cleared earlier in
this same pass"* — becomes false and must be rewritten. It must not be replaced
with "`lastActivityMs` is the sole authority on sleep", which is also false while
`workInFlight` short-circuits `platformIdle()`. State instead: every term is
bounded either within the pass or by a terminal transfer path with a watchdog
backstop, and none is a sticky "ever happened" flag.

### Test

1. **Mid-transfer disconnect.** `sleep_timeout_ms` ≈ 40 s. Connect, authenticate,
   start a PIPE transfer, send enough frames to power the panel, kill the link
   before END. Assert `Disconnect reason:` within one pass, transfer state
   cleared, panel down, advertising back up, no departed-client frame dispatched
   afterwards. Repeat for full PIPE, pipe-partial, legacy partial.
   *Timing caveat:* do not assert a hard sub-50 ms bound. A disconnect landing
   during a synchronous refresh cannot be serviced until `waitforrefresh()`
   returns; the assertion is "one pass after the handler returns", not wall-clock.
2. **The latch, fault-injected.** The obvious test — fatal NACK then disconnect —
   is worthless: `serviceBleDisconnectCleanup()` calls `resetPipeWriteState()`
   unconditionally before the gate is evaluated, so it passes with or without the
   amendment. Reproduce instead via the path that skips cleanup: force the
   `ownerStillUp` early return, or inject a lost disconnect event, then evaluate
   the gate. Alternatively use the Commit 1 watchdog path, which now terminates
   cleanly and can be asserted directly.
3. **ESP32 deep-sleep regression.** Battery config: idle → sleeps; connect and
   disconnect with no transfer → sleeps after the re-armed window; mid-transfer
   disconnect → prompt cleanup then sleeps; completed transfer → sleeps.
4. **nRF idle current.** Unchanged expected; this touches the gate, not the wait.
   Any delta means a term is stuck.
5. **Build matrix.** All 11 environments, both commits.

---

## Residuals — deliberately not in either commit

0. **The watchdog is a duration timer, not an idle timer.** `directWriteStartTime`
   is stamped at START and cleared only on completion or cleanup, so the 15-minute
   limit measures the *whole* transfer, not silence. A genuinely healthy but slow
   push — a large panel over a poor link, or a client that throttles — is aborted
   mid-flight, and after Commit 1 that abort now also resets the pipe, which is
   more correct but no less abrupt. Converting it to an idle timer (stamp on each
   accepted frame) is a behaviour change worth its own commit and its own
   argument; flagged here so the choice is deliberate rather than inherited.
1. **A fatally-NACKed pipe session is still remembered indefinitely.**
   `pipeState` has no timestamp field, so there is no watchdog for a session
   whose hardware half was already released by `sendPipeNack()`. Harmless after
   Commit 2, since nothing reads it as work, and bounded in practice by the next
   START/END/disconnect. Adding `start_ms` to `PipeWriteState` would close it.
2. **Event coalescing.** `takeDisconnectedEvent()` does check-then-clear and then
   reads `s_disconnectReason`, so a second same-type event inside that window is
   lost and its payload can attach to the wrong edge. `serviceBleEvents()` also
   processes connect before disconnect regardless of true order. Observed in the
   8.5 h capture as a connect/disconnect/connect burst inside 400 ms.
3. **Cleanup is dropped, not deferred.** `serviceBleDisconnectCleanup()` clears
   `s_disconnectCleanupPending` *before* the `ownerStillUp` test and returns, so
   when the skip fires that disconnect's teardown never runs at all. Now a safety
   consideration, per the deep-sleep correction above.
4. **nRF MSD cadence** is still coupled to the idle duration.

## Risk and rollback

| Risk | Likelihood | Mitigation |
|---|---|---|
| Commit 1 reorders the normal END path | Low | Test 1.3; the moved block is the watchdog only |
| A transfer flag latches through a path not in the table | Low after Commit 1 | Fault-injected test 2; watchdog backstops |
| ESP32 stops deep-sleeping | Low | Test 3; residual 3 is the remaining exposure |
| Amending `transferActive()` changes touch/WiFi behaviour | Low, and desirable | Only for errored sessions, where suspension was already wrong |

Both commits revert independently. Commit 2 depends on Commit 1; Commit 1 depends
on nothing.
