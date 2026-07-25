# EPD Panel Power Session & Keep-Alive

Latency reduction for the connect → refresh → disconnect cycle by keeping the
e-paper panel powered and initialized between rapid updates instead of fully
re-initializing it every push.

Implemented on branch `feat/less-latency` (commits `40ef209`..`f8d683e`). Written
against [src/display_service.cpp](../src/display_service.cpp),
[src/main.cpp](../src/main.cpp), and [src/main.h](../src/main.h).

---

## 1. The problem

Every image push used to pay a full cold panel bring-up. Measured on nRF52840
(the `[+Nms]` instrumentation still in `partial_prepare_panel_ram`), a partial
refresh spent **1586 ms** in prep alone, before any pixels moved:

| Step | Cost | What it is |
|---|---|---|
| `pwrmgm(true)` | **899 ms** | `delay(800)` rail settle + `delay(100)` pin settle in [main.cpp](../src/main.cpp) |
| `bbepInitIO` | 72 ms | SPI re-begin + first `pInitFull` |
| `bbepWakeUp` | 40 ms | hardware reset pulse |
| `bbepSendCMDSequence` | 7 ms | partial init sequence |
| `bbepFill(WHITE, PLANE_1)` | 284 ms | whole-frame white fill over per-byte SPI |
| `bbepFill(WHITE, PLANE_0)` | 284 ms | whole-frame white fill |
| **Total** | **1586 ms** | |

**Root cause:** every refresh tail unconditionally ran `bbepSleep(&bbep, 1)`
(SSD16xx deep-sleep mode `0x03` — controller RAM lost) followed by `pwrmgm(false)`
(`SPI.end()`, rail cut). Clients do connect → one update → **disconnect** cycles
seconds apart, so the panel was torn all the way down and rebuilt from cold every
single time — even when the next update was moments away.

---

## 2. What changed (actual code, before → after)

### 2.1 A power state machine replaces the bare `displayPowerState` bool

**Before:** panel power was a single global `bool displayPowerState`, set blindly
inside `pwrmgm()` with no idempotency guard. Every caller open-coded its own
power/init/sleep sequence.

**After:** a three-state machine is the single source of truth. Declared in
[main.h](../src/main.h) next to `displayPowerState` (which is kept synced as a
legacy mirror), with the enum + constant in the shared header
[display_service.h](../src/display_service.h) so every translation unit sees them:

```c
enum PwrMgmState : uint8_t { PWR_OFF = 0, PWR_WARM = 1, PWR_ACTIVE = 2 };
#define EPD_KEEPALIVE_MAX_S 30   // hard cap on power_option.screen_timeout_seconds

volatile uint8_t pwrmgmState      = PWR_OFF;  // single source of truth
uint32_t         pwrmgmOffDeadlineMs = 0;     // keep-alive deadline (valid only in WARM)
volatile uint8_t pwrmgmLock       = 0;        // cross-task try-lock
```

`PWR_OFF == 0` is deliberate: BSS is zeroed at boot and after an ESP32 deep-sleep
wake, so the default state always matches the physical reality of a powered-down
rail.

### 2.2 `pwrmgm()` gained an idempotency guard and owns the OFF↔ACTIVE edges

**Before** ([main.cpp](../src/main.cpp) `pwrmgm`): set `displayPowerState = onoff`
and ran the full ~900 ms power-up (or the `SPI.end()` power-down) **every call**,
even if already in that state.

**After:** two guard lines short-circuit a no-op transition, and the state var is
updated beside the legacy bool:

```c
if (onoff  && pwrmgmState != PWR_OFF) return;   // already powered (WARM or ACTIVE)
if (!onoff && pwrmgmState == PWR_OFF) return;   // already off
displayPowerState = onoff;
pwrmgmState = onoff ? PWR_ACTIVE : PWR_OFF;
if (!onoff) pwrmgmOffDeadlineMs = 0;
```

This makes every legacy caller safe for free: a redundant `pwrmgm(true)` (e.g. the
Seeed boot path that powers up twice) becomes a no-op instead of a wasted 900 ms,
while real transitions — including the boot `true → false → true` rail power-cycle
in `prepareEpdRailForBoot` — always proceed because each call actually flips the
state. `pwrmgm` owns only the **OFF ↔ ACTIVE** rail edges; the **ACTIVE ↔ WARM**
edges belong to the session layer (below).

### 2.3 The panel session layer

Five functions in [display_service.cpp](../src/display_service.cpp) wrap all panel
power/init/teardown. Public entry points are declared in
[display_service.h](../src/display_service.h); Acquire/Release are file-static.

- **`epdSessionAcquire(bool partialInit)` → `bool cold`** — bring the panel up for
  a transfer. From `PWR_OFF` it does the full cold path: `pwrmgm(true)`,
  `bbepInitIO`, `bbepWakeUp`, `bbepSendCMDSequence`. From `PWR_WARM` it cancels the
  keep-alive deadline, goes back to `PWR_ACTIVE`, and (Phase 1) still re-runs
  `bbepWakeUp` + the init sequence — skipping only the ~900 ms rail bring-up and
  `bbepInitIO`. Returns `true` iff it was a cold bring-up.
- **`epdSessionRelease(bool refreshSuccess)`** — finish a transfer. On success with
  keep-alive enabled: arm the deadline (`millis() + window`) and move to
  `PWR_WARM`, leaving the controller **awake** (no `bbepSleep`, so `is_awake`
  stays 1) and the rail/SPI up. On failure or when the window is 0: power fully off.
- **`epdSessionForceOff()`** — power the panel fully down now: sleep the controller
  (`bbepSleep(&bbep,1)` for bb_epaper, or `fastepd_direct_sleep()` +
  `fastepd_mark_hw_deinitialized()` for FastEPD IT8951), then `pwrmgm(false)`. Idempotent
  (`PWR_OFF` → returns immediately).
- **`epdSessionTick()`** — the keep-alive timer poll (§4).
- **`epdSessionIsWarm()`** — `true` when `PWR_WARM`.

### 2.4 Refresh tails route through the session instead of tearing down

The partial and full-frame paths no longer hard-code sleep+power-off:

| Site | Before | After |
|---|---|---|
| `partial_prepare_panel_ram` | `pwrmgm(true)` + `bbepInitIO` + `bbepWakeUp` + init seq | `bool cold = epdSessionAcquire(true)` |
| `partial_write_to_panel` tail | `bbepSleep` + `delay(50)` + `pwrmgm(false)` | `if (ok) epdPlanesPrepared = true; epdSessionRelease(ok)` |
| `directWriteActivatePanel` | force `pwrmgm(false)`+`delay(50)`+`pwrmgm(true)` + init | `epdSessionAcquire(false)` |
| `directWriteFinishAndRefresh` | `bbepSleep` + `delay(50)` | (nothing — release happens in cleanup) |
| `cleanupDirectWriteState` | unconditional `bbepSleep`+`pwrmgm(false)` | `if (PWR_ACTIVE) { refreshDisplay ? ForceOff : Release(true) }` |
| `cleanup_partial_write_state` | `powerDown = active && displayPowerState` | `teardown = active && pwrmgmState == PWR_ACTIVE` → `ForceOff` |

**The ACTIVE-only-teardown invariant** is what makes disconnect handling free: the
cleanup helpers only touch panel power when `pwrmgmState == PWR_ACTIVE` (a transfer
is genuinely in flight). After a *successful* refresh, `epdSessionRelease` has
already moved the panel to `PWR_WARM`, so the post-success cleanup calls — and the
disconnect callbacks that invoke them — become power no-ops and the panel stays
warm. No disconnect-path logic changed; only comments were added
([device_control.cpp](../src/device_control.cpp),
[esp32_ble_callbacks.h](../src/esp32_ble_callbacks.h)).

### 2.5 Full-frame partials skip the two white fills

The two `bbepFill(WHITE)` calls exist to guarantee `PLANE_0 == PLANE_1` *outside*
the refresh rect, so uninitialized controller RAM can't flash noise during
`MASTER_ACTIVATE`. But the partial protocol always streams `plane_size * 2` bytes —
**both** planes, fully, for the rect — or the refresh is refused. When the rect
covers the whole frame there is no "outside the rect," so the fills are provably
redundant even on a cold panel:

```c
bool fullFrame = partialCtx.x == 0 && partialCtx.y == 0 &&
                 partialCtx.width  == globalConfig.displays[0].pixel_width &&
                 partialCtx.height == globalConfig.displays[0].pixel_height;
if (!fullFrame) {
    bbepFill(&bbep, BBEP_WHITE, PLANE_1);
    bbepFill(&bbep, BBEP_WHITE, PLANE_0);
}
```

This alone removes **568 ms** from a full-frame partial (the observed client sends
`raw rect 0,0 800x480`). Sub-rect partials still fill. Widening the skip to warm
sub-rects (using `epdPlanesPrepared`) is deferred to Phase 2b pending per-panel
validation.

### 2.6 Smaller latency trims
- `waitforrefresh` polls BUSY at **10 ms** instead of 100 ms, so a ~0.5 s refresh
  returns up to ~90 ms sooner. The `i == 0` "never went busy" error check still
  holds (BUSY asserts within µs of `MASTER_ACTIVATE`).
- The trailing `delay(200)` in `waitforrefresh` and the `delay(20)` before the
  partial refresh were already removed earlier on the branch.
- BLE fast-link request (2M PHY + 251-octet DLE) re-enabled in
  [device_control.cpp](../src/device_control.cpp) `connect_callback` — attacks the
  transfer phase, self-negotiating.

---

## 3. The state machine

### States

| State | Rail | Controller | SPI | Meaning |
|---|---|---|---|---|
| `PWR_OFF` | off | asleep (RAM lost) | ended | idle / boot default / post deep-sleep-wake |
| `PWR_ACTIVE` | on | awake, initialized | up | a transfer/refresh is in flight |
| `PWR_WARM` | on | awake, initialized | up | keep-alive: powered-idle, deadline armed |

### Transitions

```
                 epdSessionAcquire (cold: pwrmgm(true)+InitIO+wake+initSeq)
        ┌─────────────────────────────────────────────────────────────┐
        │                                                              ▼
   ┌─────────┐                                                   ┌───────────┐
   │ PWR_OFF │                                                   │ PWR_ACTIVE│
   └─────────┘                                                   └───────────┘
        ▲     ▲                                                    │      │
        │     │  epdSessionRelease(success=false || window==0)     │      │
        │     └────────────────────────────────────────────────────┘      │
        │                                                                  │
        │  epdSessionForceOff        epdSessionRelease(success, window>0)  │
        │  (bbepSleep + pwrmgm(false))          arm deadline               │
        │                                                                  ▼
        │                                                            ┌───────────┐
        └────────────────────────────────────────────────────────── │ PWR_WARM  │
             epdSessionTick: deadline reached  → ForceOff            └───────────┘
                                                                          │  ▲
                                          epdSessionAcquire (warm:        │  │
                                          cancel deadline, wake+initSeq)  ▼  │
                                                                       PWR_ACTIVE
```

- **OFF → ACTIVE**: `epdSessionAcquire` (cold). The only path that pays the ~900 ms
  rail bring-up + full init.
- **ACTIVE → WARM**: `epdSessionRelease(true)` with keep-alive enabled. Arms the
  deadline; leaves everything powered.
- **ACTIVE → OFF**: `epdSessionRelease(false)`, `epdSessionRelease` on an
  AXP2101 board (window 0), or any `epdSessionForceOff` (error/disconnect/
  watchdog/deep-sleep/boot).
- **WARM → ACTIVE**: `epdSessionAcquire` (warm). Cancels the deadline; skips the
  rail bring-up and `bbepInitIO`. **This is the fast path** — the whole point.
- **WARM → OFF**: `epdSessionTick` when the deadline passes, or a `ForceOff` from
  any teardown trigger.

### Who calls what

- **Acquire**: `partial_prepare_panel_ram` (partial init), `directWriteActivatePanel`
  (full init).
- **Release**: `partial_write_to_panel` tail, `cleanupDirectWriteState(false)`.
- **ForceOff**: `epdSessionTick` (expiry), `cleanupDirectWriteState(true)` and
  `cleanup_partial_write_state` (error/disconnect/15-min watchdog), `enterDeepSleep`,
  and the three boot tails in `initDisplay`.

---

## 4. The keep-alive timer

A pure `millis()`-poll — no hardware timer, no interrupt. This matches the repo's
existing watchdog idiom and, critically, keeps all panel SPI work on the loop task
(never on a callback/timer task).

### Arming
`epdSessionRelease(true)` (keep-alive enabled) sets:
```c
pwrmgmOffDeadlineMs = millis() + epdKeepAliveWindowMs();
pwrmgmState = PWR_WARM;
```

`epdKeepAliveWindowMs()` sources the window from config rather than a hardcoded
constant:
```c
static uint32_t epdKeepAliveWindowMs(void) {
    uint8_t s = globalConfig.power_option.screen_timeout_seconds;
    for (uint8_t i = 0; i < globalConfig.sensor_count; i++) {
        if (globalConfig.sensors[i].sensor_type == SENSOR_TYPE_AXP2101) {
            if (s != 0) {
                writeSerial("[EPD session] AXP2101 present - keep-alive forced off (screen_timeout_seconds ignored)", true);
            }
            return 0;
        }
    }
    if (s > EPD_KEEPALIVE_MAX_S) s = EPD_KEEPALIVE_MAX_S;
    return (uint32_t)s * 1000;
}
```
- **Source:** `PowerOption.screen_timeout_seconds` (uint8, seconds) — the number of
  seconds the panel stays powered (`PWR_WARM`) after a successful refresh.
- **Clamp:** the value is clamped to `EPD_KEEPALIVE_MAX_S` (30), so the effective
  window is `min(30, configured) × 1000 ms`. Out-of-range values are clamped, not
  rejected.
- **0 = immediate off, and is the default.** When the field is 0, `epdSessionRelease`
  takes the `window == 0` branch and powers the panel straight down after the
  refresh — no keep-alive. Old persisted config blobs and factory defaults carry 0
  in the reserved bytes, so existing devices keep `main`'s shipped immediate-off
  behavior until the field is explicitly set.
- **AXP2101 override (retained):** on boards with an AXP2101 PMIC (warm idle draw
  unmeasured) the window is forced to 0 regardless of config, so the panel powers
  straight down. The override checks the sensor list before the clamp and now
  announces itself — but only when it actually suppresses a non-zero configured
  value:
  ```
  [EPD session] AXP2101 present - keep-alive forced off (screen_timeout_seconds ignored)
  ```
  On an AXP2101 board left at the 0 default nothing is being suppressed, so the log
  stays quiet; with a non-zero config it logs once per release (i.e. once per push).

### Live-disable hardening (config reload)
Because a warm panel arms its deadline from the value in effect *at release time*, a
config write that sets `screen_timeout_seconds = 0` would otherwise not take effect
until the stale deadline (≤30 s) expired. `reloadConfigAfterSave()`
([communication.cpp](../src/communication.cpp)) closes this: after a successful
reload, if keep-alive is now disabled and the panel is still warm, it force-offs the
panel immediately:
```c
if (globalConfig.power_option.screen_timeout_seconds == 0 && epdSessionIsWarm()) {
    epdSessionForceOff();
}
```
The force-off is conditional (only when the new value is 0 and the panel is warm), so
a normal config save on a warm panel is left untouched. A *shortened* non-zero value
simply applies from the next release; the residual (≤30 s) is not re-clamped live.

### Firing
`epdSessionTick()` is called from two places in [main.cpp](../src/main.cpp):
`loop()` (top, every iteration) and inside `idleDelay()`'s wait loop (so a long
blocking `idleDelay(sleep_timeout_ms)` on nRF, or `idleDelay(2000)` on mains ESP32,
still expires the window on time). It is a cheap no-op unless the panel is warm:

```c
void epdSessionTick(void) {
    if (pwrmgmState != PWR_WARM) return;   // fast pre-check
    if (!pwrmgmLockTryTake()) return;      // held by a transfer → skip this pass
    if (pwrmgmState == PWR_WARM && (int32_t)(millis() - pwrmgmOffDeadlineMs) >= 0) {
        epdSessionForceOffLocked();
    }
    pwrmgmLockGive();
}
```

The `(int32_t)(millis() - deadline) >= 0` comparison is wrap-safe across the 49-day
`millis()` rollover. The tick never touches hardware unless it is actually
expiring the window.

### Cancelling
`epdSessionAcquire` (warm path) clears `pwrmgmOffDeadlineMs` and moves to
`PWR_ACTIVE` — so a new transfer arriving inside the window simply cancels the
pending power-down and reuses the warm panel.

### Cross-task safety
On nRF, a BLE write callback (running Acquire on the higher-priority Bluefruit task)
can race the loop task's tick (running ForceOff). A single-byte try-lock guards the
transitions:
- Acquire/Release/ForceOff take the lock (`pwrmgmLockTake`, which **spins with
  `delay(1)`** — a `vTaskDelay` yield, so it can never starve a lower-priority
  holder; a bare busy-spin would deadlock the single core).
- The tick **try-locks** (`pwrmgmLockTryTake`) and skips its pass entirely if a
  transfer holds the lock — so it can never cut the rail mid-init.

---

## 5. Deep-sleep interaction (ESP32)

Deep sleep only fires between connections (a live BLE link keeps `workInFlight`
true, blocking the idle branch). The permutations:

| State at sleep decision | Outcome |
|---|---|
| `PWR_OFF` | `enterDeepSleep`'s `epdSessionForceOff()` is a no-op; sleeps as before |
| `PWR_WARM`, battery | idle-hold (default 10 s) typically expires before the configured keep-alive window → `enterDeepSleep` runs `ForceOff` (below its early-returns), cleanly sleeping the panel before the MCU sleeps. Effective keep-alive = min(keep-alive window, idle-hold) |
| `PWR_WARM`, mains | `enterDeepSleep` early-returns *above* the `ForceOff`; panel stays warm; the tick in `idleDelay(2000)` expires it at the configured keep-alive window (`screen_timeout_seconds`, ≤30 s) |
| `PWR_ACTIVE` | unreachable — a transfer/connection keeps `workInFlight` true |
| wake from deep sleep | RAM zeroed → `PWR_OFF` (enum value 0 correct); rail physically off; first push pays a cold acquire. `displayed_etag` (RTC-persisted) still gates partials |

The `epdSessionForceOff()` in `enterDeepSleep` is placed **after** all early-returns
(mains check, live-link re-check) so the mains case above works and, as a bonus, it
closes a pre-existing hazard where deep sleep never powered the panel down.

---

## 6. Results

| Case | Before | After |
|---|---|---|
| Cold, full-frame partial | 1586 ms | ~1018 ms (fills skipped) |
| Cold, sub-rect partial | 1586 ms | 1586 ms (fills required) |
| **Warm, full-frame partial** | 1586 ms | **~47 ms** (wake + init seq only) |
| Warm, sub-rect partial | 1586 ms | ~615 ms (fills still run) |

Plus: disconnect within the keep-alive window (`screen_timeout_seconds`, ≤30 s)
reconnects onto a warm panel; `waitforrefresh` returns
up to ~90 ms sooner; BLE transfer phase 2–4× faster on peers that honour the
PHY/DLE upgrade.

### Instrumentation
Watch the serial log for `[EPD session]` lines (acquire cold/warm, release
warm-idle, keep-alive expired, force off) and the `[+Nms]` prep breakdown in
`partial_prepare_panel_ram` (which now also logs whether the fills ran or were
skipped for a full-frame rect).

---

## 7. Scope & follow-ups

**Implemented (Phase 1):** the state machine, keep-alive, full-frame fill skip,
`waitforrefresh` tightening, BLE fast-link. All changes are inside `src/` — nothing
under `.pio/libdeps/` (bb_epaper) was modified.

**Deferred (need per-panel hardware validation):**
- **Phase 2a** — warm acquire skips `bbepWakeUp` + resends the init sequence only on
  partial↔full transitions (`epdSessionInitWasPartial` is already tracked). ~47 → ~7 ms.
- **Phase 2b** — skip the fills on warm *sub-rect* partials via `epdPlanesPrepared`
  (already set after a successful partial). Caution: plane consistency after a
  partial is guaranteed on SSD1680-class parts but not universally (UC81xx).
- **Upstream-only** — bb_epaper's nRF per-byte SPI → bulk DMA (~200–280 ms/frame)
  and the 800 ms rail-settle reduction. Out of scope here (libdep / hardware).

**Known residual behavior:** a hard decode error mid-START leaves the panel `PWR_WARM`
(released via `cleanupDirectWriteState(false)`) until the keep-alive window's tick or the next push;
this is safe and self-recovers.
