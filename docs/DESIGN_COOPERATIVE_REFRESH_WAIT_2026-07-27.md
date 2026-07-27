# Cooperative refresh wait

**Status: PROPOSED — not implemented.** Written 2026-07-27 on `feat/unify-nrf-esp-phase3`
after a live nRF capture showed a 16 s refresh stalling BLE event servicing long enough
to corrupt reconnect handling. Nothing in this document has been coded; the two bugs it
describes as *observed* were fixed separately in `c4bd4bc`, which treated the symptom.

## The problem

`waitforrefresh()` ([`src/display_service.cpp:747`](../src/display_service.cpp)) polls the
panel BUSY line with a raw `delay(10)`:

```c
for (size_t i = 0; i < (size_t)(timeout * 100); i++){
    delay(10);
    if(i % 50 == 0) od_log_raw(".");
    if(!bbepIsBusy(&bbep)){ ... return true; }
}
```

`delay()` is not `idleDelay()`. For the whole refresh — up to 60 s by the bound, ~16 s
measured on a Spectra 6-colour panel — the loop task does **nothing**:

| Serviced by `idleDelay()` | Serviced during a refresh |
|---|---|
| `processButtonEvents()` / `processTouchInput()` | — |
| `processLedFlash()` | — |
| `buzzerService()` | — |
| `epdSessionTick()` | — |
| `serviceBleTx()` | — |
| (`loop()` only) `serviceBleEvents()` | — |

The refresh is a dead zone, not merely a busy one.

### Observed consequence

nRF, `nrf52840custom-debug`, 2026-07-27:

```
[0194.860] I: === BLE CLIENT DISCONNECTED (nRF) ===      <- flag latched, callback task
[0199.713] I: Refresh took 15.98 seconds                  <- loop task unblocks
[0212.962] I: === BLE CLIENT CONNECTED (nRF) ===          <- next client
[0213.167] D: [BLE][Q:0] URX 0x0080 (12 B): 00 80 ...     <- its first command, queued
[0213.241] I: Disconnect reason: 19                       <- 18 s stale, serviced now
[0213.242] W: Dropped 1 queued command(s) ...             <- ate the NEW client's frame
```

Two defects fell out of that single stall, both fixed in `c4bd4bc`:

1. `bleRxQueueDiscardAll()` discarded the ring at *service* time rather than at
   disconnect time, so a reconnect landing inside the stall lost its first command.
   Fixed with a boundary captured in the disconnect callback.
2. `serviceBleDisconnectCleanup()`'s "owner still up" guard was inside
   `#ifdef OPENDISPLAY_HAS_WIFI`, leaving nRF with no guard at all —
   `resetPipeWriteState()` would have destroyed the new client's session. Fixed by
   hoisting the `ble.isConnected()` half out of the `#ifdef`.

Both fixes make deferred work *correct when serviced late*. Neither reduces the
lateness. Any future deferred-work path inherits the same 16 s exposure.

## Blocking is not required by the hardware

`bbepRefresh()` issues `SSD1608_MASTER_ACTIVATE` and returns. The panel drives the
waveform on its own and reports progress on a single GPIO. The MCU has no work to do
for the duration — `waitforrefresh` is polling a pin. There is no DMA to babysit, no
SPI transaction held open, no timing constraint tighter than the 10 ms poll.

Two properties of the existing code confirm the design was already headed this way.

**The protocol does not couple the ACK to the refresh.** `directWriteFinishAndRefresh`
sends the END ack and force-flushes it *before* starting the refresh
([`src/display_service.cpp:2369-2379`](../src/display_service.cpp)), with a comment
naming the reason — the loop task is the response ring's only drainer. The refresh
outcome is reported afterwards as an independent
`RESP_DIRECT_WRITE_REFRESH_SUCCESS` ([`:2418`](../src/display_service.cpp)). Clients
already accept the result as a later, separate frame.

**`epdRefreshInProgress` is already the right shape.** It is raised around exactly this
window, and both `serviceBleDisconnectCleanup()` and `serviceBleAdvertisingRestart()`
already gate on it ([`src/main.cpp:317`, `:367`](../src/main.cpp)). But it can only ever
be observed as `true` from *inside* the blocking wait, so nothing can currently act on
it. The flag exists for a design that was never realized.

## Option A — cooperative wait (recommended)

Keep the call signature and the straight-line control flow. Replace the `delay(10)`
inside the poll with the servicing block `idleDelay()` already runs.

Factor that block out of `idleDelay()` ([`src/main.cpp:619`](../src/main.cpp)) into a
shared `serviceLoopMaintenance()` and call it from both. Two copies of the list is how
the two directions drift — the same failure mode this branch removed from the RX/TX
logging.

The refresh still owns the loop task; no new commands dispatch; RX stays queued. What
changes is that the 16 s stops being dead.

**Hard invariant: `serviceLoopMaintenance()` must NOT call `serviceBleRx()`.**
`waitforrefresh` is reached from inside a command handler, dispatched by
`serviceBleRx()`. Dispatching from within would make every handler reentrant and
corrupt multi-frame transfer state mid-stream. `idleDelay()` already states this rule in
its comment; the shared helper must carry it.

`idleDelay()`'s early return on `bleRxQueuePending()` is **not** part of the shared
block — it is `idleDelay`-specific (cap command latency) and wrong for the refresh poll,
which must keep waiting on BUSY regardless of queued RX.

Calling `serviceBleEvents()` from the wait is safe and is the point of the change:
`requestFastLink()` touches only the BLE stack, and the two flag consumers already defer
on `epdRefreshInProgress`, so cleanup and advertising correctly stay deferred until the
refresh ends.

**Effect on the observed bug:** the disconnect is consumed within ~10 ms instead of
18 s, so the reconnect race never opens. The `c4bd4bc` boundary remains correct and
still covers the residual window; it simply stops being load-bearing.

**Cost:** BLE notifications and button/touch input now interleave with SPI-idle polling.
No SPI transaction is open during the wait, so there is no bus contention. Battery
builds do more work during a refresh than before — measurable, and worth capturing on
the bench alongside the two nRF baselines still outstanding.

## Option B — true non-blocking state machine

Poll BUSY from `loop()` and split all six `waitforrefresh` call sites into before/after
halves:

| Site | Path |
|---|---|
| [`:543`](../src/display_service.cpp) | boot refresh, bb_epaper |
| [`:1592`](../src/display_service.cpp) | boot refresh, FastEPD |
| [`:2389`](../src/display_service.cpp) | transfer end, FastEPD |
| [`:2398`](../src/display_service.cpp) | transfer end, bb_epaper |
| [`:3210`](../src/display_service.cpp) | partial refresh, skip-reinit panels |
| [`:3213`](../src/display_service.cpp) | partial refresh, normal path |

**The blocking is currently doing double duty as mutual exclusion.** Remove it and a
`DIRECT_WRITE_START` arriving mid-refresh reaches `epdSessionAcquire()` and stomps the
panel. Option B therefore also requires explicit gating for every display-touching
opcode — a new rejection or deferral policy, and a wire-visible one if it NACKs.

What it buys over Option A is command dispatch *during* a refresh. On a single-link
peripheral already mid-transfer that is of limited value and arguably undesirable.

**Not recommended** unless a concrete requirement appears for servicing commands while
the panel draws.

## Sequencing

This rewrites the body of `waitforrefresh()`, which `debug/freeze-fix-phase2`
(`5f3e74c`) already replaced with `waitForPanelIdle()` — a `millis()` deadline plus a
per-driver busy predicate. **Land that branch first, or fold both into one change.**
Doing them independently guarantees a conflict in the same loop body.

Landing phase 2 first is preferable: `waitForPanelIdle()` is already the single wait all
drivers poll through, so Option A becomes a one-line substitution inside it rather than
a change repeated per driver.

One interaction to note: on this branch `fastepd_wait_refresh()`
([`src/display_fastepd.cpp:228`](../src/display_fastepd.cpp)) is a stub that returns
immediately, so FastEPD panels have no wait to make cooperative. Phase 2 replaces it
with a real LUTAFSR poll. Until that lands, Option A affects bb_epaper panels only.

## Acceptance criteria

Bench-only; CI builds but executes nothing.

1. During a full refresh on a Spectra 6-colour panel, a BLE disconnect is logged as
   `Disconnect reason: N` within ~100 ms of the `CLIENT DISCONNECTED` banner — not after
   the refresh completes.
2. Queued responses continue to drain mid-refresh: `[BLE][Q:n] ETX ...` lines appear
   during the dot cadence, and `Q` does not climb monotonically across the refresh.
3. A button press mid-refresh registers within one poll interval.
4. Disconnect + reconnect + first command, entirely inside a refresh: the command
   dispatches, and `Dropped N queued command(s)` reports only the departed client's
   frames.
5. No command dispatches mid-refresh — RX depth may rise, and the drain happens after
   `epdRefreshInProgress` clears. A dispatch banner between `EPD refresh:` and
   `Refresh took` means the reentrancy invariant was broken.
6. Refresh wall-clock is unchanged within noise versus the blocking build.
7. Battery idle current and pipe-write throughput captured against the same reference
   build as the other outstanding nRF baselines.
