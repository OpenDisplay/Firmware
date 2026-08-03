#pragma once

#include <stdint.h>

// Portable watchdog + boot-reason interface.
//
// Deliberately includes NO vendor headers -- not nrf_wdt.h, not esp_task_wdt.h --
// so any translation unit can call it without dragging in an SDK. Exactly one
// implementation is linked per build (watchdog_nrf.cpp or watchdog_esp32.cpp),
// each gated whole-file on its target, so the other compiles to an empty
// translation unit and no build_src_filter change is needed. This mirrors
// BleTransport; see src/ble_transport.h for the same reasoning stated at length.
//
// Free functions rather than a class: unlike BleTransport there is no state worth
// exposing, and every caller wants exactly one global watchdog.
//
// WHY THIS IS PORTABLE WHEN THE WATCHDOG IS NOT
// ---------------------------------------------
// Only nRF arms a hardware watchdog today. But every feed site and every
// breadcrumb stamp lives in code that compiles for BOTH targets -- loop() and
// idleDelay() in main.cpp ("One loop body for both targets"), waitforrefresh()
// and the bb_epaper entry points in display_service.cpp. An nRF-only API would
// mean #ifdef TARGET_NRF around ~20 call sites in shared files, which is what
// docs/PLAN_UNIFY_NRF_ESP32_LOOP_BLE_2026-07-27.md worked to remove. A no-op on
// ESP32 costs nothing and keeps the shared body clean.
//
// WHAT IS DELIBERATELY ABSENT
// ---------------------------
//   - stop()/disable(): the nRF52840 WDT CANNOT be stopped once started. Its
//     register block has no TASKS_STOP and no ENABLE (contrast SPIM/TWI/UART,
//     which have ENABLE at 0x500); only a system reset clears it. Exposing a
//     stop() would advertise a capability one target cannot honour.
//   - a runtime timeout parameter: CRV must be written BEFORE the start task and
//     is latched thereafter. Compile-time only, via OPENDISPLAY_NRF_WDT_S.
//   - task registration: ESP32's task watchdog is per-task subscribe/unsubscribe;
//     nRF's reload registers are not an analogue. The contract here is ONE LOOP
//     TASK, ONE WATCHDOG.
//
// See docs/PLAN_NRF_HARDWARE_WATCHDOG_2026-08-01.md.

// --- breadcrumb phases -----------------------------------------------------
// Stamped at panel-phase transitions and retained across a reset, so the boot
// after a watchdog reset can name the wait that wedged instead of reporting only
// that something did. Values must fit 4 bits (0-15); see the GPREGRET2 layout in
// watchdog_nrf.cpp.
enum OdWatchdogPhase : uint8_t {
    OD_WDT_PHASE_IDLE         = 0,   // pre-session: stamped once at boot, before
                                     // the first epdSessionAcquire/Release/ForceOff
    OD_WDT_PHASE_ACQUIRE_COLD = 1,
    OD_WDT_PHASE_ACQUIRE_WARM = 2,
    OD_WDT_PHASE_INIT_SEQ     = 3,
    OD_WDT_PHASE_FILL         = 4,
    OD_WDT_PHASE_STREAM       = 5,
    OD_WDT_PHASE_REFRESH_WAIT = 6,
    OD_WDT_PHASE_RELEASE      = 7,
    OD_WDT_PHASE_FORCE_OFF    = 8,
    OD_WDT_PHASE_BOOT_REFRESH = 9,
    // pwrmgmState is plain RAM, not retained across a reset, so without a
    // distinct phase per idle sub-state a freeze during either one reports the
    // same generic OD_WDT_PHASE_IDLE and the two are indistinguishable after the
    // fact. These name which idle state the session was actually left in.
    OD_WDT_PHASE_IDLE_OFF     = 10,  // session fully powered down (PWR_OFF)
    OD_WDT_PHASE_IDLE_WARM    = 11,  // panel kept awake for keep-alive (PWR_WARM)
    // pwrmgm(true)'s rail bring-up sequence, uninstrumented until the 2026-08-03
    // freeze (reset ~120s after ACQUIRE_COLD, never reaching INIT_SEQ -- the wedge
    // was somewhere inside pwrmgm() itself). These name which of its four
    // sub-steps was entered last. Uses the last 4 of the 16 available phase values.
    OD_WDT_PHASE_PWRMGM_AXP2101 = 12,  // before initAXP2101() (I2C PMIC bring-up)
    OD_WDT_PHASE_PWRMGM_RAIL    = 13,  // before pwr_pin HIGH + delay(800)
    OD_WDT_PHASE_PWRMGM_PINS    = 14,  // before panel GPIO setup + delay(100/200)
    OD_WDT_PHASE_PWRMGM_WIRE    = 15,  // before initOrRestoreWireForOpenDisplay()
    OD_WDT_PHASE__MAX         = 15,
};

// Decode and log why we booted, and evaluate the consecutive-reset strike
// counter. Call ONCE, early in setup(), after od_log_init() so the line is
// actually emitted. Must run before odWatchdogArm().
void odWatchdogBootInit(void);

// True when the strike counter tripped and this boot must skip all panel work.
// Always false until W-3 lands.
bool odWatchdogInSafeMode(void);

// Arm the hardware watchdog. Irreversible on nRF. Call ONCE, after
// odWatchdogBootInit() and before the boot panel path. No-op where no hardware
// watchdog is used.
void odWatchdogArm(void);

// Prove forward progress. Cheap enough to call in a tight poll loop; a no-op
// when nothing is armed.
//
// FEED ONLY FROM SITES WHOSE EXECUTION PROVES THE PROGRAM IS ALIVE -- loop(),
// cooperative waits that return, and immediately before entering a bounded
// library call. NEVER from an ISR, timer or stack callback: an interrupt-fed
// watchdog verifies that the interrupt controller is running, not that the
// program is, which is the classic way to build a watchdog that never fires.
void odWatchdogFeed(void);

// Record the panel phase we are about to enter. Retained across reset.
void odWatchdogBreadcrumb(uint8_t phase);
