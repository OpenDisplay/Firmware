# nRF Hardware Watchdog — Implementation Plan

**Date:** 2026-08-01
**Revision:** 2 — rewritten after an adversarial review found four blocking errors in rev 1.
Corrections are marked **[R1]** throughout; §10 lists them.
**Target:** the watchdog itself is `TARGET_NRF` only (nRF52840, Bluefruit / S140 v7.3.0).
The **module is portable** (W-0): a target-neutral `watchdog.h` with two self-gated
implementations, ESP32 stubbed (W-6). ESP32 behaviour is unchanged except that its existing
reset-reason decode relocates out of `main.cpp`.
**Goal:** recover the device when an *unbounded* wait — in `loop()`, or below it in a
vendored driver we cannot instrument — wedges the panel permanently.

---
**STATUS UPDATE (2026-08-03): timeout changed to 120 s, below this plan's D-1 value.**
`OPENDISPLAY_NRF_WDT_S` was dropped from the 300 s this plan derives and validates (§3.2,
§10 D-1) to **120 s**. This is a deliberate, confirmed choice made outside this plan's
analysis, not a correction to it — everything below about the 240 s worst case, the 1.25×
margin at 300 s, and W-2's pre-call feed policy is still accurate background, but the
headline numbers ("300 s", "1.25× margin") no longer describe the shipped value.

**Consequence: the margin this plan relies on is gone.** At 120 s, a healthy
`REFRESH_FULL` on the 7-colour split-buffer panel (§3.1's ~240 s worst case) will trip the
watchdog *mid-refresh* on a device that isn't wedged. T2 (§8) — measuring that panel's real
span — is now a prerequisite for shipping to it, not a confirmation exercise; the "possible
sizing error" residual in §10 is elevated from unlikely to expected. Do not ship this
timeout to a 7-colour split-buffer panel without re-deriving it.

**Also since rev 2:** two idle breadcrumb phases (`IDLE_OFF`/`IDLE_WARM`) replaced the
single shared `OD_WDT_PHASE_IDLE` used at `epdSessionForceOffLocked()`/`epdSessionRelease()`,
and four more (`PWRMGM_AXP2101`/`RAIL`/`PINS`/`WIRE`) were added inside `pwrmgm()` itself
(`main.cpp`) after a watchdog reset landed there — `pwrmgm()` had no breadcrumb coverage in
the original design. All 16 phase values in the 4-bit field are now in use (`watchdog.h`).
See the retained-breadcrumb reset-reason logging fix below for a related gap that was
losing the very reset-reason line this plan's boot log depends on.
---

## 1. What this closes

Two accepted residuals and one new finding converge on the same gap.

**Accepted residual 1 — nRF's unbounded I2C spins.**
[PLAN_PHASE2_BOUND_WAITS_2026-07-26.md](PLAN_PHASE2_BOUND_WAITS_2026-07-26.md) decision
**D-L** found that `Wire_nRF52.cpp:166-181` / `:230-247` spin on TWIM events with no deadline
and no yield, and chose option **(a) accept and document**.

**Accepted residual 2 — the scheduler-starving fault class.**
Decision **D-K** accepted that `millis()` bounds all fail together if the scheduler stalls, on
the grounds that it is "a fault class we cannot recover from anyway." A hardware watchdog is
what makes that class recoverable, so D-K's premise no longer holds once this lands.

**New finding (2026-08-01) — the same shape on SPI.**
`nrfx_spim.c:598` blocks in `while (!nrf_spim_event_check(p_spim, NRF_SPIM_EVENT_END)){}` with
no timeout and no yield, because `SPIClass` initialises nrfx with a NULL handler
(`SPI.cpp:101`). On nRF the Arduino API drives the panel **one byte at a time**
(`arduino_io.inl:214-221`), so the firmware enters that spin ~96,000 times per full refresh.

Underlying all three: no firmware-armed watchdog exists on nRF. See **V1** for the corrected
statement of what exists on ESP32 — rev 1 got that wrong.

## 2. Verification done before writing this plan

Checked against source in this workspace. Rows corrected after review are marked **[R1]**.

| # | Fact | Evidence |
|---|---|---|
| V1 **[R1]** | nRF has **no** watchdog. ESP32's IDF task watchdog **is enabled and initialised** (`CONFIG_ESP_TASK_WDT_EN=y`, `INIT=y`, `TIMEOUT_S=5`) — but **watches nothing that would catch a wedged `loop()`**: S3/classic set only `CHECK_IDLE_TASK_CPU0=y` while `CONFIG_ARDUINO_RUNNING_CORE=1` puts `loopTask` on CPU1; C3/C6 subscribe no idle task at all | `framework-arduinoespressif32-libs/{esp32s3,esp32,esp32c3,esp32c6}/sdkconfig`. Rev 1 claimed "no watchdog on either target" — the *conclusion* for `loop()` holds, the *reason* was wrong |
| V2 **[R1]** | Neither the core nor Bluefruit **operates** `NRF_WDT` | grep over `cores/`+`libraries/` finds only MDK register definitions. Rev 1 said "only MDK defs exist"; `drivers/include/nrfx_wdt.h` is also installed — header only |
| V3 **[R1]** | The nrfx WDT **implementation** is not compiled | no `drivers/src/nrfx_wdt.c`; `NRFX_WDT_ENABLED` absent from `nrfx_config.h`. The *header* `nrfx_wdt.h` does exist. Use `hal/nrf_wdt.h` |
| V4 **[R1]** | ⚠️ **`RESETREAS` is read and cleared by the core before `setup()`**, and exposed via `readResetReason()` | `cores/nRF5/wiring.c:37-40` — `_reset_reason = NRF_POWER->RESETREAS;` then `RESETREAS \|= RESETREAS` (write-1-to-clear); getter at `:72`. **Rev 1's `sd_power_reset_reason_get()` design would read zero forever**, and its "bits accumulate across boots" claim was false |
| V5 | `GPREGRET` (id 0) is already taken by the DFU handshake | [device_control.cpp:868-869](../src/device_control.cpp#L868-L869) |
| V6 **[R1]** | `GPREGRET2` (id 1) is reachable under the SD and unused by app + framework source | `nrf_soc.h:641-666`. **Not fully verified against the installed bootloader image**, which is not in these sources — see risk in W-3 |
| V7 **[R1]** | No `.noinit` section exists in the packaged linker scripts | `grep -l noinit cores/nRF5/linker/*.ld` → none. Rev 1 concluded "package edit or nothing" — **wrong**: PlatformIO supports `board_build.ldscript` (`platforms/nordicnrf52/builder/frameworks/arduino/adafruit.py:190-196`). GPREGRET2 is still preferred, but as a choice, not a necessity |
| V8 **[R1]** | WDT is clocked from LFCLK — but **LFCLK is already started by the core before `setup()`** | `wiring.c:45-57` (`TASKS_LFCLKSTART = 1`). **Rev 1's conclusion that this forces arming after `ble.begin()` was wrong**; arm ordering is now a policy choice (W-3), not a clock constraint |
| V9 | Reload uses a fixed magic and per-register enables | `nrf_wdt.h:56` `NRF_WDT_RR_VALUE 0x6E524635`; `WDT_RREN_RR0_*`, `nrf52840_bitfields.h:17376` |
| V10 | Behaviour in sleep / debug-halt is configurable | `WDT_CONFIG_SLEEP_Pos = 0`, `WDT_CONFIG_HALT_Pos = 3`, `nrf52840_bitfields.h:17385,17391` |
| V11 **[R2]** | **The WDT cannot be stopped once started.** The register block has no `TASKS_STOP` and no `ENABLE`, and `CRV`/`RREN`/`CONFIG` latch at `START`. **Which resets clear it is NOT established** — a power-on reset certainly does; whether a soft/`DOG`/pin reset does is unverified, so the code feeds any watchdog it finds running regardless (W-1) | nRF52840 architectural property; the HAL exposes START and reload, no stop. Constrains W-3 and §5's DFU residual |
| V12 **[R1]** | **Worst-case uninstrumentable span = ~240 s.** The 8.1" Spectra is `BBEP_SPLIT_BUFFER \| BBEP_7COLOR`; its init list holds **4** `BUSY_WAIT` entries, and `REFRESH_FULL` sends the whole sequence to CS1 **and again to CS2** | `bb_ep.inl:3704-3726` (4 × `BUSY_WAIT`), `:4373-4380` (CS1 then CS2), `:3967-3969` (30 s cap for 3/4/7-colour). 8 × 30 s = 240 s inside **one** `bbepRefresh()` call |
| V13 **[R1]** | `enterDFUMode()` **jumps to the bootloader without a system reset** | [device_control.cpp:868-884](../src/device_control.cpp#L868-L884) — `sd_softdevice_disable()`, vector-table move, `bootloader_util_app_start()`. No `NVIC_SystemReset()`. With **V11**, an armed WDT keeps counting into the bootloader |

## 3. The central design problem

A watchdog is easy to get catastrophically wrong here, because this firmware **legitimately
blocks for minutes** on a healthy device.

### 3.1 Blocking-span inventory

| Span | Worst case | Feed possible inside? |
|---|---|---|
| **`bbepRefresh(REFRESH_FULL)` on a 7-colour split-buffer panel** | **~240 s** (**V12**) | **No** — one libdep call |
| `waitforrefresh(60)` | ~126 s — 6000 iterations × ~21 ms (`delay(10)` + `bbepIsBusy`'s own `delay(10)`+`delay(1)`, `bb_ep.inl:3984-3986`). The argument is **not** seconds | **Yes** — [display_service.cpp:857-870](../src/display_service.cpp#L857-L870) |
| `bbepSendCMDSequence` (init only) | N × 5 s (B/W) or N × 30 s (multicolour) | No — libdep |
| `pwrmgm(true)` | 900 ms | Yes |
| `nrfx_spim` / `Wire` spins | **unbounded** (the fault) | No — and deliberately not (§W-2) |

### 3.2 Why 300 s works — but only with a pre-call feed **[R1]**

**Rev 1 claimed 300 s "dominates every span" with >3× headroom, from an assumed N=3 and a
single command sequence. That was wrong** (**V12**): the real worst case is 240 s, and rev 1's
margin was ~1.25×, erasable by any work preceding the call in the same handler invocation.

**D-1 is confirmed at 300 s**, and is made safe by a change to the feed policy rather than to
the timeout: **feed immediately before every call that enters bb_epaper's blocking region**
(W-2). Since the 240 s is a single uninterruptible call, feeding on entry means the watchdog
faces exactly that span and nothing else.

- Longest uncovered span becomes **240 s** — the single worst libdep call, not a sum.
- Margin **60 s (1.25×)**, and everything before the call is irrelevant because the counter is
  freshly reloaded.
- **This margin is thin and must be respected**: any future increase in `BUSY_WAIT` count,
  controller count, or per-wait cap eats directly into it. Recorded as a residual in §5, with
  T2 measuring the real figure on hardware.

## 4. Design

### W-0 — Module shape: portable header, two self-gated implementations

Not an nRF-only module, and the reason is `#ifdef` count in shared code. Every feed site and
breadcrumb stamp lives in a file that compiles for both targets: `loop()`/`idleDelay` in
[main.cpp](../src/main.cpp) (*"One loop body for both targets"*), and `waitforrefresh` plus the
bb_epaper entry points in [display_service.cpp](../src/display_service.cpp). An nRF-only API
would put `#ifdef TARGET_NRF` around ~20 call sites in shared files — what
[PLAN_UNIFY_NRF_ESP32_LOOP_BLE_2026-07-27.md](PLAN_UNIFY_NRF_ESP32_LOOP_BLE_2026-07-27.md)
worked to remove.

Follow the transport pattern, whose reasoning `ble_transport.h` already records: *"Exactly one
implementation is linked per build... a plain class rather than an abstract base: virtual
dispatch would cost a vtable and indirect calls for zero benefit"*, and *"The whole file is
gated on TARGET_NRF, so an ESP32 build compiles it to an empty translation unit — no
build_src_filter changes needed."*

```
src/watchdog.h          portable. No nrf_wdt.h, no esp_task_wdt.h — any TU may include it.
src/watchdog_nrf.cpp    whole file #ifdef TARGET_NRF   — real
src/watchdog_esp32.cpp  whole file #ifdef TARGET_ESP32 — stubs (W-6)
```

Free functions; no state worth exposing.

```c
void odWatchdogBootInit(void);             // decode reset reason, evaluate strike counter
bool odWatchdogInSafeMode(void);           // [R1] main.cpp gates initDisplay() on this
void odWatchdogArm(void);                  // once, before the boot panel path (W-3)
void odWatchdogFeed(void);                 // W-2 feed sites
void odWatchdogBreadcrumb(uint8_t phase);  // panel-phase stamps (W-4)
```

#### Deliberately NOT in the API

| Omitted | Why |
|---|---|
| `stop()` / `disable()` | nRF WDT **cannot be stopped once started** (**V11**); ESP32's TWDT can. Exposing it would advertise a capability one target cannot honour |
| runtime timeout parameter | `CRV` must be written **before** `START` and is immutable after. Compile-time only |
| task registration | ESP32's TWDT is per-task; nRF's reload registers are not an analogue. Contract is **one loop task, one watchdog**, stated in the header |

#### Bonus: this removes `#ifdef`s rather than adding them

The ESP32 reset decode already exists inline in `main.cpp` under `#ifdef TARGET_ESP32`
([:27-44](../src/main.cpp#L27-L44), [:82-84](../src/main.cpp#L82-L84)). Moving it into
`watchdog_esp32.cpp` is a net `#ifdef` reduction in `main.cpp`.

### W-1 — Watchdog configuration (nRF)

Use the Nordic HAL (`nrf_wdt.h`): header-only inline functions already on the include path, no
driver to enable (**V3**), and it supplies `NRF_WDT_RR_VALUE` (**V9**) instead of a magic
constant.

- `CRV = (OPENDISPLAY_NRF_WDT_S × 32768) − 1`. At 300 s: `9,830,399` (`0x0095FFFF`).
- **[R1] Validate the flag at compile time**, since it is a public build knob feeding a
  hardware register: `static_assert` that it is either `0` (disabled) or within
  `[60, 3600]` seconds. The lower bound keeps it above §3.1's spans; the upper stays well
  inside `CRV`'s 32-bit ceiling (~131,072 s). Rev 1's "no overflow guard needed" was valid
  only for the literal 300.
- `RREN = RR0` only — one reload register, one feeder.
- `CONFIG.SLEEP = 1` — keep counting while the CPU sleeps; `idleDelay` feeds every ≤100 ms
  chunk. Without it, a device that hangs while idle is never recovered.
- `CONFIG.HALT = 0` — do not count while halted by a debugger.
- **[R1] Check `RUNSTATUS` before configuring.** There is no way to disarm or reconfigure a
  running WDT: the register block exposes `TASKS_START` and `RR[8]` only — **no `TASKS_STOP`,
  no `ENABLE`** (`nrf52840.h:2044-2062`; contrast SPIM/TWI/UART, which have `ENABLE` at
  `0x500`) — and `CRV`/`RREN`/`CONFIG` are latched at `START`, so later writes are ignored.

  Consequence: if the **bootloader** left the WDT running, our configuration is silently
  discarded and we inherit its timeout — possibly far shorter than 300 s — while believing we
  set our own. `odWatchdogArm()` must read `RUNSTATUS` first, and if the WDT is already
  running, **log loudly and skip configuration** rather than pretend. Feeding must then still
  happen (the inherited dog is real). Confirm behaviour on hardware in T7.

### W-2 — Feed policy

**Principle: only feed from a site whose execution proves forward progress.**

| Site | Why |
|---|---|
| `loop()` top, beside `epdSessionTick()` ([main.cpp:896](../src/main.cpp#L896)) | primary liveness proof |
| `idleDelay()` chunk loop ([main.cpp:1031-1041](../src/main.cpp#L1031-L1041)) | a long idle wait is healthy |
| `waitforrefresh()` poll loop ([display_service.cpp:857](../src/display_service.cpp#L857)) | a 126 s refresh is healthy |
| **[R1] immediately before each bb_epaper entry point** — the **15** call sites in `display_service.cpp` — `bbepRefresh` ×3, `bbepSendCMDSequence` ×3, `bbepWakeUp` ×3, `bbepSleep` ×1, `bbepFill` ×2 (consecutive), and **`bbepInitIO` ×3**. Two counting errors were caught in review: `bbepFill` appears twice, and `bbepInitIO` was omitted entirely even though it sends `pInitFull` internally — twice on a split-buffer panel — making it a ~240 s span in its own right: `bbepWakeUp` ([:365](../src/display_service.cpp#L365), [:476](../src/display_service.cpp#L476), [:500](../src/display_service.cpp#L500)), `bbepSendCMDSequence` ([:366](../src/display_service.cpp#L366), [:479](../src/display_service.cpp#L479), [:503](../src/display_service.cpp#L503)), `bbepSleep` ([:451](../src/display_service.cpp#L451)), `bbepRefresh` ([:562](../src/display_service.cpp#L562), [:2503](../src/display_service.cpp#L2503), [:3340](../src/display_service.cpp#L3340)), `bbepFill` ([:3368](../src/display_service.cpp#L3368)) | **This is what makes 300 s safe** (§3.2). Reloading on entry means the dog faces the single 240 s call, not that call plus everything before it |

**Explicitly NOT fed from:**

- **Any ISR, timer, or SoftDevice callback.** An interrupt-fed watchdog verifies the interrupt
  controller is alive, not the program. Non-negotiable.
- **`nrfx_spim`'s or `Wire`'s spins** — unreachable without forking the package, and *we do not
  want to*: those spins are the fault.

**[R1] Corrected property statement.** Rev 1 claimed "every wait we have bounded feeds the dog;
every wait we have not, does not." That is false — `bbepWaitBusy` is bounded (5/30 s) and gets
no feed. The accurate statement is: **every span we can reach is fed at its boundary; spans we
cannot reach must individually fit inside the timeout.** V12 is the largest such span.

**[R1] A stuck BUSY pin does not trip the watchdog** and is not meant to: `waitforrefresh`
keeps feeding for its 6,000 iterations, returns `false`, and `loop()` resumes. That is correct
behaviour — a failed refresh is an error to report, not a wedge to reset. T4 must therefore
distinguish "BUSY stuck" (no reset expected) from a non-returning call inside one iteration
(reset expected).

### W-3 — Boot-loop containment **[R1] — redesigned**

Rev 1's design was **internally contradictory** and is replaced wholesale. Its faults:

1. It armed the watchdog *after* the boot panel path, so a wedge *in* that path could never
   produce a `DOG` reset — yet it then claimed a "panel-safe-mode escape after 3 of them."
   Strikes could never accumulate from the very failure the escape existed for.
2. It cleared the strike counter on "first successful refresh." Every ordinary boot performs a
   successful boot refresh, so each recurring runtime wedge would clear the previous strike and
   the count would never reach 3.
3. In safe mode no refresh occurs, so "first successful refresh" could never fire and the
   device could never leave safe mode.
4. It put a persistent counter and an overwritten breadcrumb in the same 8-bit register with no
   bit allocation, so a breadcrumb write would destroy the counter.

**Redesigned:**

- **Arm before the boot panel path**, immediately after `odWatchdogBootInit()`. Boot wedges are
  now covered, which is what makes the strike counter meaningful. **V8** removed the clock
  reason for arming late, so nothing prevents this. The `bootdiag` `while (!Serial)` gate at
  [main.cpp:58](../src/main.cpp#L58) still precedes any sane arm point and stays uncovered.
- **Clear the counter on sustained uptime, not on panel success.** Clear once the device has
  run `WDT_HEALTHY_MS` (propose **10 minutes**, ≥2× the timeout) since boot with the watchdog
  armed. This is panel-independent, so it works identically in safe mode — solving faults 2
  and 3 together. Strikes accumulate only when resets come *fast*, which is exactly the
  boot-loop condition safe mode exists for; a device that survives 10 minutes between wedges is
  not boot-looping and should keep retrying the panel.
- **Safe mode is self-exiting.** After 10 healthy minutes in safe mode the counter clears, so
  the next reset boots normally and retries the panel. Worst case is a bounded oscillation —
  3 fast resets, a long safe-mode period, one retry — rather than a permanent brick.
- **[R1] Explicit GPREGRET2 bit allocation** (8 bits, `nrf52840_bitfields.h`):

  | Bits | Field |
  |---|---|
  | 7:6 | validity tag `0b10` — distinguishes a real value from cold-boot garbage |
  | 5:4 | strike counter, 0–3, saturating |
  | 3:0 | breadcrumb phase, 0–15 |

  Breadcrumb writes are read-modify-write over bits 3:0 only —
  `sd_power_gpregret_clr(1, 0x0F)` then `sd_power_gpregret_set(1, phase & 0x0F)`. Two SVCs, not
  atomic; safe because only the loop task writes it. Document that in the header.
- **[R1] Risk (V6):** GPREGRET2 is unused by application and framework source, but the
  installed **bootloader image was not inspected**. If the bootloader writes it, the validity
  tag causes a stale value to be discarded rather than misread — the counter resets, degrading
  containment but not causing a wrong action. Verify on hardware (T7).
- **[R1] No MSD status bit.** Rev 1 promised to surface safe mode in the manufacturer data.
  Status bit 3 is reserved and "must be 0" in `include/opendisplay_structs.h`, which is
  **vendored byte-for-byte** from `opendisplay-protocol`; per CLAUDE.md such a change must
  originate in the canonical repo. Out of scope here — safe mode is reported via the boot log
  only. Surfacing it on the wire is follow-up work in the protocol repo.

### W-4 — Observability

**[R1] Reset-reason decode must use `readResetReason()`, not the peripheral.** Per **V4** the
core has already read and cleared `RESETREAS` before `setup()`. Rev 1's
`sd_power_reset_reason_get()` design would have read zero on every boot and silently reported
"power-on" for every watchdog reset — defeating the entire purpose of step 1. Decode the saved
word's `RESETPIN / DOG / SREQ / LOCKUP / OFF / DIF` bits. No clearing is needed or possible;
the core already did it, which also means bits do **not** accumulate across boots.

**Breadcrumb.** One-byte phase code in GPREGRET2 bits 3:0, stamped at panel-phase transitions
(`IDLE`, `ACQUIRE_COLD`, `ACQUIRE_WARM`, `INIT_SEQ`, `FILL`, `STREAM`, `REFRESH_WAIT`,
`RELEASE`, `FORCE_OFF`) — 9 values, inside the 16 the field allows. Logged next to the reset
reason at boot.

Payoff: a freeze stops being "it wedged somewhere" and becomes
`reset=DOG breadcrumb=INIT_SEQ`, naming the wedged wait directly.

**`TIMEOUT` ISR (D-3).** Fires ~61 µs (2 LFCLK cycles) before the reset — enough to stamp a
final breadcrumb, not enough to write flash or drain a UART. Strictly best-effort; the
boot-side decode is the mechanism we rely on.

### W-5 — Build-flag control

`-DOPENDISPLAY_NRF_WDT_S=300` in `[env:nrf52840custom]`, `0` = disabled, validated per W-1.

- Default **on**. A watchdog that ships disabled is not a watchdog.
- The three other nRF envs inherit via `${env:nrf52840custom.build_flags}` — no separate
  entries. `CONFIG.HALT = 0` keeps breakpoints safe in the debug env (**D-4**).
- `bootdiag` is safe: its `while (!Serial)` gate precedes the arm point (W-3).
- **No ESP32 flag** — `watchdog_esp32.cpp` is stubbed (W-6).

### W-6 — The ESP32 stub **[R1]**

- `odBootReasonLog()` / `odWatchdogBootInit()` — **real**: the existing `resetReasonName()` +
  `esp_reset_reason()` logic relocated from `main.cpp`.
- `odWatchdogArm()` — no-op that logs, **once**, the *accurate* state: the IDF task watchdog
  is enabled at 5 s but **no task that would catch a wedged `loop()` is subscribed** (**V1**).
- `odWatchdogFeed()`, `odWatchdogBreadcrumb()` — empty. `odWatchdogInSafeMode()` returns false.

Rev 1 planned to log "no watchdog armed", which **V1** shows would be false on S3 and classic
ESP32. The accurate message is more useful anyway: it names the specific gap
(`CHECK_IDLE_TASK_CPU0` vs `ARDUINO_RUNNING_CORE=1`) that a future implementation must close,
which is a one-line `esp_task_wdt_add(NULL)` on the loop task plus the feed sites this plan
already wires up.

## 5. What this does not fix

- **It does not fix any unbounded wait.** `nrfx_spim.c:598` and the six `Wire_nRF52` spins are
  untouched. A permanent hang becomes a periodic reset — better and observable, but the device
  still drops its link, loses transfer state, and pays a cold bring-up.
- **Recovery is slow, by choice.** Up to 5 minutes dead before reset; with **D-2**'s threshold
  of 3, up to ~15 minutes to reach safe mode. Right trade for e-paper, where five minutes late
  is invisible but resetting a healthy device mid-refresh is a visible regression.
- **[R1] The 240 s / 300 s margin is thin (1.25×).** Any growth in `BUSY_WAIT` count,
  controller count, or the multicolour cap eats it directly. If a future panel exceeds it,
  healthy devices get reset mid-refresh — the main way this change can do harm. T2 measures the
  real figure; treat V12 as a number to re-check whenever a panel is added.
- **[R1] DFU can take a `DOG` reset — accepted, out of scope.** Per **V13**, `enterDFUMode()`
  jumps to the bootloader without a system reset, and per **V11** the watchdog cannot be
  stopped, so it keeps counting into a bootloader that will not feed it. A DFU session lasting
  >300 s from the jump will be reset. Assessment: the reset re-enters the bootloader (GPREGRET
  id 0 still holds `0xB1`), so the expected outcome is an interrupted transfer the host must
  retry, not a brick — **unless** the reset lands mid-flash-write, which is not analysed here.
  The fix, if it is ever wanted, is to replace the direct jump with `NVIC_SystemReset()`,
  matching the core's own `enterUf2Dfu()` (`wiring.c:76-80`). **[R2] Caveat:** whether a
  running nRF52840 WDT survives a non-power-on reset could not be established from any source
  in this workspace. If it survives, `NVIC_SystemReset()` would **not** help and only a power
  cycle stops it. The implementation is written to be correct either way (W-1's unconditional
  inherit-detection); `RUNSTATUS` logged at boot settles it empirically. Explicitly descoped.

  **[R1] This residual may not exist at all.** Many Nordic/Adafruit bootloaders feed the WDT in
  their main loop precisely because the application may have armed one. The installed
  bootloader is a flashed binary, absent from these sources, so this could not be verified.
  **T9 answers it empirically** — if DFU survives, the residual is void.
- **It says nothing about brownout.** POFCON is not enabled (`grep POFCON src/` → nothing).
- **It does not verify the panel rail power-cycles on reset** (was D-6, descoped).
- **ESP32 gains no watchdog.** The stub's boot log makes that explicit rather than assumed.

## 6. Decisions — all resolved

| ID | Decision | Resolution |
|---|---|---|
| **D-1** | Timeout value | ✅ **300 s**, re-confirmed after **V12** revealed a 240 s worst case. Made safe by W-2's pre-call feed, not by the timeout alone. Margin 1.25× — residual in §5 |
| **D-2** | Panel-safe-mode threshold | ✅ **3 consecutive `DOG` resets**, with the redesigned clear rule in W-3 |
| **D-3** | `TIMEOUT` ISR for a final breadcrumb? | ⚠️ **Deferred — NOT implemented.** Phase-transition breadcrumbs are stamped eagerly at every panel-phase entry, so the retained value is already correct when the reset lands; the ISR would only add ~61 µs of redundancy. Revisit if a real failure shows a phase gap |
| **D-4** | Watchdog in the debug envs? | ✅ **Enabled**, inherited; `bootdiag` included (its Serial gate precedes the arm point) |
| **D-5** | Ship observability before arming? | ✅ **Yes — split.** Step 1 lands W-0/W-4/W-6 with nothing armed |
| **D-6** | Does the panel rail drop on a WDT reset? | ✅ **Descoped** — §5 |
| **D-7 [R1]** | Fix the DFU jump so the watchdog cannot reset a DFU session? | ✅ **No — out of scope by decision.** Recorded as an accepted residual in §5 with its severity assessment |

## 7. Test plan

| # | Test | Expected |
|---|---|---|
| T1 | Normal operation, many push cycles | No reset; `DOG` never appears in the decode |
| T2 **[R1]** | Full refresh on the **slowest supported panel** (7-colour split-buffer if available), cold, full-frame | No reset, and **log the measured span** — this validates V12's 240 s and the real margin against 300 s. The single most important test |
| T3 | Injected infinite loop in `loop()` behind a debug command | Reset within timeout; boot logs `reset=DOG` + breadcrumb |
| T4 **[R1]** | (a) BUSY held asserted through a refresh; (b) non-returning call inside one `waitforrefresh` iteration | (a) **no** reset — `waitforrefresh` returns false and `loop()` resumes; (b) reset. Distinguishing these is the point |
| T5 | Injected wedge in the SPIM path (transfer with SPIM disabled) | Reset with `breadcrumb=STREAM`/`INIT_SEQ` — the target case |
| T6 | Long `idleDelay` at max `sleep_timeout_ms`, battery | No reset — validates `CONFIG.SLEEP=1` + the idle feed |
| T7 **[R1]** | Force 3 fast consecutive `DOG` resets; then let the device run >10 min. Log `RUNSTATUS` at every boot | Safe mode entered, device advertises and is DFU-reachable; counter clears after the healthy window; next reset boots normally. Also confirms (a) GPREGRET2 survives a `DOG` reset and the bootloader does not clobber it (**V6**), and (b) `RUNSTATUS` reads *not running* at boot — i.e. the bootloader did not leave a WDT armed that would silently override our config (W-1) |
| T8 | Debugger breakpoint held >5 min | No reset — validates `CONFIG.HALT=0` |
| T9 **[R1]** | Enter DFU and idle in the bootloader >5 min | **Outcome unknown — that is the point of the test.** If the bootloader feeds the WDT, no reset and the **D-7** residual is void. If it does not, a reset is expected; confirm the device re-enters DFU rather than bricking. Either way, record the result against §5 |
| T10 | `bootdiag`, no USB host, left >5 min | No reset — the Serial gate must stay outside coverage |

## 8. Files touched

| File | Change |
|---|---|
| `src/watchdog.h` | **new** — portable interface, five free functions (W-0). No vendor headers |
| `src/watchdog_nrf.cpp` | **new** — `#ifdef TARGET_NRF`. HAL config, feed, `readResetReason()` decode, GPREGRET2 bit-allocated counter + breadcrumb, safe-mode state |
| `src/watchdog_esp32.cpp` | **new** — `#ifdef TARGET_ESP32`. Real boot decode relocated from `main.cpp`; accurate `Arm()` log; empty feed/breadcrumb (W-6) |
| `src/main.cpp` | feed at `loop()` top and in `idleDelay`; `odWatchdogBootInit()` + `odWatchdogArm()` before the boot panel path; gate `initDisplay()` on `odWatchdogInSafeMode()`. **Net `#ifdef` reduction** — `resetReasonName()` and its call site move out |
| `src/display_service.cpp` | feed in `waitforrefresh` **and before all 15 bb_epaper entry points** (W-2); breadcrumb stamps. No `#ifdef`s |
| `platformio.ini` | `-DOPENDISPLAY_NRF_WDT_S=300` in `[env:nrf52840custom]`; three other nRF envs inherit |
| `docs/TIMER_AND_WATCHDOG_INVENTORY_2026-07-26.md` | §1.1/§1.2 are **wrong** per **V1** — the ESP32 TWDT is enabled, just not watching `loop()`. Correct both |
| `docs/PLAN_PHASE2_BOUND_WAITS_2026-07-26.md` | note D-L/D-K residuals are now recoverable (not fixed) |

## 9. Sequencing

1. **W-0 + W-4 + W-6 — module and observability, nothing armed** (**D-5**). Create the header
   and both implementations; relocate the ESP32 decode; add the nRF `readResetReason()` decode
   and the GPREGRET2 breadcrumb; wire all feed sites so they exist and compile while `Arm()` is
   inert. Zero brick risk. **Independently valuable**: answers whether field units are resetting
   or hanging, and lands the `main.cpp` `#ifdef` cleanup regardless of the rest.
2. **T2 first, then W-1 + W-2** — measure the real worst-case refresh *before* arming, since
   §3.2's margin is only 1.25×. Then arm at 300 s.
3. **W-3** — arm-before-boot-panel-path, GPREGRET2 bit allocation, strike counter with the
   uptime-based clear, safe mode.

Steps 2 and 3 must land together: step 2 alone is the configuration with the boot-loop hazard
W-3 exists to contain.

## 10. Review corrections (rev 1 → rev 2)

Rev 1 was reviewed adversarially; findings were re-verified against source before acceptance.

**Blocking errors, all confirmed:**
1. **V4** — `RESETREAS` is cleared by the core before `setup()`; rev 1's SoftDevice-API design
   would have read zero forever and reported every watchdog reset as a power-on.
2. **V12 / §3.2** — worst case is 240 s (4 `BUSY_WAIT` × 2 controllers × 30 s), not the assumed
   ≤90 s. Rev 1's ">3× headroom" was wrong; 300 s is only safe with W-2's pre-call feed.
3. **W-3** — rev 1's containment was self-contradictory in four distinct ways (§W-3); redesigned.
4. **V13 / D-7** — DFU jumps to the bootloader without a reset, so rev 1's T9 ("the system reset
   clears the WDT") was factually wrong. Now an accepted residual.

**Corrected but non-blocking:** V1 (ESP32 TWDT *is* enabled — conclusion held, reason wrong),
V2/V3 (imprecise inventory), V6 (bootloader not inspected), V7 (`board_build.ldscript` exists),
V8 (LFCLK already started before `setup()`), W-1 (flag needs validation), W-2 (property
statement was false; stuck-BUSY behaviour clarified), W-6 (stub message would have been false).

**Verified correct and unchanged:** V5, V9, V10, V11, and §3.1's ~126 s `waitforrefresh`
arithmetic.
