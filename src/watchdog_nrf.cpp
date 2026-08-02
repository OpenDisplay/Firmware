// nRF52840 implementation of the portable watchdog interface.
//
// The whole file is gated on TARGET_NRF, so an ESP32 build compiles it to an
// empty translation unit -- no build_src_filter changes needed across the CI
// environments. Nothing outside this file names an nRF WDT type.
//
// See docs/PLAN_NRF_HARDWARE_WATCHDOG_2026-08-01.md.

#include "watchdog.h"

#ifdef TARGET_NRF

#include <Arduino.h>
#include <nrf.h>
#include <nrf_soc.h>
#include <nrf_sdm.h>
#include <hal/nrf_wdt.h>
#include "od_log.h"

// ---------------------------------------------------------------------------
// Reset reason
// ---------------------------------------------------------------------------
//
// CRITICAL: do NOT read NRF_POWER->RESETREAS here, and do NOT call
// sd_power_reset_reason_get(). The Arduino core has ALREADY read and cleared the
// register before setup() runs:
//
//     cores/nRF5/wiring.c:37-40
//         _reset_reason = NRF_POWER->RESETREAS;
//         NRF_POWER->RESETREAS |= NRF_POWER->RESETREAS;   // write-1-to-clear
//
// and exposes the saved word through readResetReason() (wiring.h:32). Reading the
// peripheral ourselves returns zero on every boot, which would silently report
// every watchdog reset as a power-on -- defeating the entire point of this module.
//
// It also means the bits do NOT accumulate across boots: the core clears them each
// time, so no clearing is needed or possible on our side.

static uint32_t s_resetReason = 0;

static void logResetReason(uint32_t r) {
    // RESETREAS is a bitfield, not an enum: several causes can be latched at once.
    // Print every set bit rather than the first match.
    //
    // ALL eight nRF52840 causes are listed. An earlier revision omitted VBUS, NFC
    // and LPCOMP, which made any of them print as "POWERON" because the fallback
    // keyed on "nothing matched" rather than on r == 0.
    static const struct { uint32_t msk; const char* name; } kinds[] = {
        { POWER_RESETREAS_RESETPIN_Msk, "RESETPIN" },
        { POWER_RESETREAS_DOG_Msk,      "DOG"      },
        { POWER_RESETREAS_SREQ_Msk,     "SREQ"     },
        { POWER_RESETREAS_LOCKUP_Msk,   "LOCKUP"   },
        { POWER_RESETREAS_OFF_Msk,      "OFF"      },
        { POWER_RESETREAS_LPCOMP_Msk,   "LPCOMP"   },
        { POWER_RESETREAS_DIF_Msk,      "DIF"      },
        { POWER_RESETREAS_NFC_Msk,      "NFC"      },
        { POWER_RESETREAS_VBUS_Msk,     "VBUS"     },
    };
    if (r == 0) {
        // A cold start latches nothing. This is the ONLY power-on signature.
        od_log_info("[WDT] reset reason: POWERON (0x00000000)");
        return;
    }
    char buf[96];
    size_t n = 0;
    uint32_t seen = 0;
    buf[0] = '\0';
    for (unsigned i = 0; i < sizeof(kinds) / sizeof(kinds[0]); i++) {
        if (!(r & kinds[i].msk)) continue;
        seen |= kinds[i].msk;
        int w = snprintf(buf + n, sizeof(buf) - n, "%s%s", n ? "|" : "", kinds[i].name);
        // snprintf returns the length it WOULD have written; clamp so a truncating
        // write cannot push n past the buffer.
        if (w <= 0) break;
        n += (size_t)w;
        if (n >= sizeof(buf) - 1) { n = sizeof(buf) - 1; break; }
    }
    // Nonzero with no recognised bit is a real anomaly, not a power-on. Say so.
    if (r & ~seen) {
        snprintf(buf + n, sizeof(buf) - n, "%sUNKNOWN", n ? "|" : "");
    }
    od_log_info("[WDT] reset reason: %s (0x%08lX)", buf, (unsigned long)r);
}

// ---------------------------------------------------------------------------
// Retained state: GPREGRET2
// ---------------------------------------------------------------------------
//
// GPREGRET (id 0) is NOT available -- device_control.cpp:868-869 uses it for the
// DFU handshake (0xB1). GPREGRET2 (id 1) is free and retained across a watchdog or
// soft reset (cleared only by power-on/brownout).
//
// It is 8 bits, and it has to carry two things, so the layout is explicit:
//
//     bit  7 6 | 5 4 | 3 2 1 0
//          tag | cnt | phase
//
//   7:6  validity tag, always 0b10. Distinguishes a value we wrote from
//        cold-boot garbage or another writer; a bad tag means "discard".
//   5:4  consecutive-DOG strike counter, 0-3, saturating (W-3, not yet used).
//   3:0  breadcrumb phase, OdWatchdogPhase.
//
// Without this allocation a breadcrumb write would destroy the strike counter.
//
// *** WHY THERE ARE TWO ACCESS PATHS ***
//
// sd_power_gpregret_{get,set,clr} are numbered from SOC_SVC_BASE_NOT_AVAILABLE
// (nrf_soc.h:65,164-166) -- "SVCs that are not available when the SoftDevice is
// disabled". odWatchdogBootInit() runs early in setup(), whereas ble.begin()
// enables the SoftDevice much later, so at boot those SVCs cannot be used.
//
// When the SoftDevice is DISABLED, POWER is not a protected peripheral and
// NRF_POWER->GPREGRET2 is directly readable/writable -- which is also cheaper and
// atomic (one store, versus a get + clr + set SVC sequence).
// When the SoftDevice is ENABLED, POWER is protected and the SVCs are mandatory.
//
// sd_softdevice_is_enabled() is numbered from SDM_SVC_BASE (nrf_sdm.h:89,195),
// which IS available either way, so it is a safe discriminator.

#define OD_WDT_G2_TAG_MASK   0xC0u
#define OD_WDT_G2_TAG_VALUE  0x80u   /* 0b10 << 6 */
#define OD_WDT_G2_CNT_MASK   0x30u
#define OD_WDT_G2_CNT_SHIFT  4
#define OD_WDT_G2_PHASE_MASK 0x0Fu

static bool sdEnabled(void) {
    uint8_t on = 0;
    if (sd_softdevice_is_enabled(&on) != NRF_SUCCESS) return false;
    return on != 0;
}

// Both accessors report failure rather than swallowing it: a silently dead
// breadcrumb would be worse than no breadcrumb, because the boot log would still
// print a (stale or zero) phase and invite a wrong conclusion.
static bool g2Read(uint8_t* out) {
    if (!sdEnabled()) {
        *out = (uint8_t)(NRF_POWER->GPREGRET2 & 0xFFu);
        return true;
    }
    uint32_t v = 0;
    uint32_t rc = sd_power_gpregret_get(1, &v);
    if (rc != NRF_SUCCESS) return false;
    *out = (uint8_t)(v & 0xFFu);
    return true;
}

static bool g2Write(uint8_t v) {
    if (!sdEnabled()) {
        NRF_POWER->GPREGRET2 = v;   // single atomic store; no SVC, no clr/set race
        return true;
    }
    // The SoftDevice offers no store, only masked clear and set.
    uint32_t rc = sd_power_gpregret_clr(1, 0xFFu);
    if (rc != NRF_SUCCESS) return false;
    if (v) rc = sd_power_gpregret_set(1, v);
    return rc == NRF_SUCCESS;
}

static bool g2UpdateField(uint8_t mask, uint8_t value) {
    uint8_t cur = 0;
    if (!g2Read(&cur)) return false;
    if ((cur & OD_WDT_G2_TAG_MASK) != OD_WDT_G2_TAG_VALUE) {
        cur = OD_WDT_G2_TAG_VALUE;                   // stale/garbage: reinitialise
    }
    return g2Write((uint8_t)((cur & (uint8_t)~mask) | (value & mask)));
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

// --- W-3: boot-loop containment ------------------------------------------
//
// A watchdog that resets a device which wedges during BOOT turns one hang into an
// endless reset cycle: the device never advertises, is unreachable over BLE/DFU,
// and flattens its battery faster than if it had simply hung. That is strictly
// worse than the bug it is meant to fix, so the strike counter exists to escape it.
//
// Strikes accumulate ONLY when resets arrive fast. The counter is cleared after
// OD_WDT_HEALTHY_MS of continuous uptime, which is deliberately NOT tied to panel
// success:
//
//   - Clearing on "first successful refresh" would never accumulate, because every
//     ordinary boot performs a successful boot refresh and would wipe the previous
//     strike.
//   - It would also make safe mode permanent, because safe mode performs no
//     refresh and so could never satisfy the clear condition.
//
// An uptime rule solves both at once and is panel-independent, so it behaves
// identically in safe mode. A device that survives ten minutes between wedges is
// not boot-looping and should keep retrying the panel.
#define OD_WDT_SAFE_MODE_STRIKES 3u
#define OD_WDT_HEALTHY_MS        (10UL * 60UL * 1000UL)   /* >= 2x the timeout */

static bool     s_safeMode      = false;
static bool     s_strikesToClear = false;
static uint32_t s_bootMs        = 0;

static void strikesSet(uint8_t n) {
    if (n > 3) n = 3;
    (void)g2UpdateField(OD_WDT_G2_CNT_MASK, (uint8_t)(n << OD_WDT_G2_CNT_SHIFT));
}

void odWatchdogBootInit(void) {
    s_bootMs = millis();
    s_resetReason = readResetReason();
    logResetReason(s_resetReason);

    const bool wasDog = (s_resetReason & POWER_RESETREAS_DOG_Msk) != 0;
    uint8_t strikes = 0;

    uint8_t g2 = 0;
    if (!g2Read(&g2)) {
        od_log_warn("[WDT] GPREGRET2 unreadable - breadcrumb and strike count unavailable");
    } else if ((g2 & OD_WDT_G2_TAG_MASK) == OD_WDT_G2_TAG_VALUE) {
        strikes = (uint8_t)((g2 & OD_WDT_G2_CNT_MASK) >> OD_WDT_G2_CNT_SHIFT);
        od_log_info("[WDT] breadcrumb from previous run: phase=%u strikes=%u",
                    (unsigned)(g2 & OD_WDT_G2_PHASE_MASK), (unsigned)strikes);
    } else {
        od_log_info("[WDT] no retained breadcrumb (cold start or first boot)");
        if (!g2Write(OD_WDT_G2_TAG_VALUE)) {
            od_log_warn("[WDT] GPREGRET2 unwritable - breadcrumbs disabled");
        }
    }

    if (wasDog) {
        if (strikes < 3) strikes++;
        strikesSet(strikes);
        od_log_warn("[WDT] previous boot ended in a watchdog reset (strike %u/%u)",
                    (unsigned)strikes, (unsigned)OD_WDT_SAFE_MODE_STRIKES);
    } else if (strikes != 0) {
        // Any non-DOG reset means the fast-reset cycle was broken by something
        // else (power cycle, pin reset, deliberate reboot). Start clean.
        strikes = 0;
        strikesSet(0);
    }

    s_safeMode = (strikes >= OD_WDT_SAFE_MODE_STRIKES);
    if (s_safeMode) {
        od_log_error("[WDT] SAFE MODE: %u consecutive watchdog resets - skipping ALL panel "
                     "work this boot so the device stays reachable over BLE/DFU. "
                     "Clears after %lu s of healthy uptime.",
                     (unsigned)strikes, (unsigned long)(OD_WDT_HEALTHY_MS / 1000UL));
    }
    s_strikesToClear = (strikes != 0);

    odWatchdogBreadcrumb(OD_WDT_PHASE_IDLE);
}

// Called from the loop-top feed. Clears the strike counter once the device has
// demonstrably survived, so safe mode is self-exiting and a slow-recurring fault
// never accumulates toward it.
static void strikesClearIfHealthy(void) {
    if (!s_strikesToClear) return;
    if ((uint32_t)(millis() - s_bootMs) < OD_WDT_HEALTHY_MS) return;
    s_strikesToClear = false;
    strikesSet(0);
    od_log_info("[WDT] %lu s of healthy uptime - strike counter cleared",
                (unsigned long)(OD_WDT_HEALTHY_MS / 1000UL));
}

bool odWatchdogInSafeMode(void) {
    return s_safeMode;
}

// ---------------------------------------------------------------------------
// The watchdog itself
// ---------------------------------------------------------------------------
//
// Timeout is compile-time ONLY: CRV must be written before the start task and is
// latched thereafter, so there is no runtime knob to expose.

#ifndef OPENDISPLAY_NRF_WDT_S
#define OPENDISPLAY_NRF_WDT_S 300
#endif

#if OPENDISPLAY_NRF_WDT_S != 0
// Lower bound: must exceed every legitimate blocking span in the firmware -- the
// largest is a ~240 s bbepRefresh() on a 7-colour split-buffer panel (plan V12).
// Upper bound: CRV is 32 bits at 32768 Hz, so ~131072 s is representable; 3600 s
// keeps a wide margin and rejects a mistyped value that would silently disable
// recovery for hours.
static_assert(OPENDISPLAY_NRF_WDT_S >= 60 && OPENDISPLAY_NRF_WDT_S <= 3600,
              "OPENDISPLAY_NRF_WDT_S must be 0 (disabled) or 60..3600 seconds");
#endif

static bool s_inherited = false;   // a watchdog we found running, not one we started
#if OPENDISPLAY_NRF_WDT_S != 0
static bool s_armed = false;
#endif

void odWatchdogArm(void) {
    // Inherit-detection runs on EVERY build, including OPENDISPLAY_NRF_WDT_S=0.
    //
    // Whether a running nRF52840 WDT survives a non-power-on reset (soft reset,
    // DOG reset, pin reset) is NOT established by any source available in this
    // workspace, and the two possibilities have very different consequences:
    //
    //   - If it does NOT survive, this branch simply never fires and costs a
    //     single register read at boot.
    //   - If it DOES survive, then a build with the watchdog disabled -- reached by
    //     DFU, a reflash, or NVIC_SystemReset from a build that had it enabled --
    //     would inherit a live watchdog it never feeds, and reset forever until
    //     someone physically removes power. That is a brick.
    //
    // Feeding whatever we find is correct under both, so do that rather than pick.
    // T7 logs RUNSTATUS at boot and will settle the question empirically.
    if (nrf_wdt_started(NRF_WDT)) {
        od_log_warn("[WDT] ALREADY RUNNING at boot (not started by this call). "
                    "CRV=%lu (%lus) RREN=0x%lX CONFIG=0x%lX - cannot be stopped or "
                    "reconfigured; feeding it as-is.",
                    (unsigned long)NRF_WDT->CRV,
                    (unsigned long)((NRF_WDT->CRV + 1UL) / 32768UL),
                    (unsigned long)NRF_WDT->RREN,
                    (unsigned long)NRF_WDT->CONFIG);
        s_inherited = true;
        odWatchdogFeed();
        return;
    }
#if OPENDISPLAY_NRF_WDT_S == 0
    od_log_warn("[WDT] disabled at build time (OPENDISPLAY_NRF_WDT_S=0)");
#else
    // Order is load-bearing: CRV, RREN and CONFIG all latch at TASKS_START.
    nrf_wdt_reload_value_set(NRF_WDT, ((uint32_t)OPENDISPLAY_NRF_WDT_S * 32768UL) - 1UL);
    nrf_wdt_reload_request_enable(NRF_WDT, NRF_WDT_RR0);   // one reload register,
                                                           // one feeder, one task
    // RUN_SLEEP: keep counting while the CPU sleeps (idleDelay's delay() chunks),
    // but pause while halted by a debugger, so a breakpoint is not a reset.
    nrf_wdt_behaviour_set(NRF_WDT, NRF_WDT_BEHAVIOUR_RUN_SLEEP);

    s_armed = true;
    odWatchdogFeed();            // start the first period from a known state
    nrf_wdt_task_trigger(NRF_WDT, NRF_WDT_TASK_START);   // irreversible
    od_log_info("[WDT] armed: %us", (unsigned)OPENDISPLAY_NRF_WDT_S);
#endif
}

void odWatchdogFeed(void) {
    strikesClearIfHealthy();
#if OPENDISPLAY_NRF_WDT_S != 0
    if (!s_armed && !s_inherited) return;
#else
    if (!s_inherited) return;   // nothing of ours is armed; only feed an inherited dog
#endif
    // Every ENABLED reload register must be written before the counter reloads --
    // RREN is an AND, not an OR. Our own arm path enables RR0 alone, but an
    // INHERITED watchdog (one the bootloader started; see odWatchdogArm) may have
    // any subset enabled, and feeding only RR0 would then never reload it. Walk
    // RREN instead of assuming.
    //
    // nrf_wdt_reload_request_set writes NRF_WDT_RR_VALUE (0x6E524635); the magic
    // value is why a wild pointer or stray memset cannot accidentally pet the dog.
    for (uint8_t i = 0; i <= (uint8_t)NRF_WDT_RR7; i++) {
        nrf_wdt_rr_register_t rr = (nrf_wdt_rr_register_t)i;
        if (nrf_wdt_reload_request_is_enabled(NRF_WDT, rr)) {
            nrf_wdt_reload_request_set(NRF_WDT, rr);
        }
    }
}

void odWatchdogBreadcrumb(uint8_t phase) {
    // Skip the register work when the phase has not actually changed. A stamp
    // costs up to three SVCs once the SoftDevice is enabled (get + clr + set), so
    // without this the streaming stamps -- which sit on per-frame and per-row
    // paths -- would be far too expensive to place where they are most useful.
    //
    // Kept in sync with the register by odWatchdogBootInit(), which stamps IDLE
    // explicitly after establishing the tag, so the cache never starts out lying.
    static uint8_t s_lastPhase = 0xFF;
    static bool s_failLogged = false;
    phase &= OD_WDT_G2_PHASE_MASK;
    if (phase == s_lastPhase) return;
    // Advance the cache ONLY on success. Updating it first would suppress the
    // retry after a failed write, leaving the cache claiming a phase the register
    // never received -- and a failed clr+set can even leave the byte cleared.
    if (!g2UpdateField(OD_WDT_G2_PHASE_MASK, phase)) {
        if (!s_failLogged) {          // latched: this sits on per-frame paths
            s_failLogged = true;
            od_log_warn("[WDT] GPREGRET2 write failed - breadcrumb may be stale");
        }
        return;
    }
    s_lastPhase = phase;
}

#endif  // TARGET_NRF
