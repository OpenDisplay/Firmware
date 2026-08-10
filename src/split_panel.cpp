#include "split_panel.h"

#include <Arduino.h>
#include <bb_epaper.h>
#include <string.h>
#include "structs.h"
#include "od_log.h"
#include "watchdog.h"

extern struct GlobalConfig globalConfig;
extern BBEPDISP bbep;

// bb_epaper's public header forward-declares only bbepWriteCmd/bbepWriteData/
// bbepCMD2; everything else is declared by its callers (display_service.cpp does
// the same). These are compiled into the library from bb_ep.inl / arduino_io.inl.
void bbepSetCS2(BBEPDISP *pBBEP, uint8_t cs);
void bbepInitIO(BBEPDISP *pBBEP, uint8_t u8DC, uint8_t u8RST, uint8_t u8BUSY, uint8_t u8CS,
                uint8_t u8MOSI, uint8_t u8SCK, uint32_t u32Speed);
void bbepWaitBusy(BBEPDISP *pBBEP);
bool bbepIsBusy(BBEPDISP *pBBEP);

// Tripwires. splitPanelUsed() deliberately does NOT track a library define, so
// these assert that the pieces we build on still exist. If bb_epaper reworks the
// split-buffer model again this fails to compile on every target, rather than
// quietly producing firmware that cannot drive the panel — which is exactly how
// the BBEP_T133A01-gated predecessor of this file disappeared.
static_assert((int)EP133_SPECTRA_1200x1600 > 0,
              "bb_epaper: EP133_SPECTRA_1200x1600 missing/renamed (was EP133A_* in the limengdu fork)");
static_assert(BBEP_SPLIT_BUFFER == 0x0400,
              "bb_epaper: BBEP_SPLIT_BUFFER changed; split-panel dispatch must be revisited");
static_assert(CMD_CS_NONE == 0xf8 && CMD_CS1 == 0xfb,
              "bb_epaper: cs_mode opcodes changed; the manual chip-select sequence must be revisited");

static bool s_geometryOk = false;
static bool s_streamOpen = false;
static bool s_onLeftHalf = true;
static uint32_t s_halfWritten = 0;   // bytes into the half currently selected
static bool s_faulted = false;

// Spectra6 accepts colour codes 0-3, 5 and 6 per nibble; 4 and 7-15 are undefined
// and a panel fed one can latch an unintended waveform. bb_epaper sanitises at
// DRAW time via u8Colors_spectra, but this firmware never draws — it forwards
// host-packed bytes — so the clamp has to happen on the way to the wire.
static uint8_t s_nibbleLut[256];
static bool s_lutReady = false;

static void buildNibbleLut(void) {
    if (s_lutReady) return;
    for (int i = 0; i < 256; i++) {
        const uint8_t hi = (uint8_t)((i >> 4) & 0x0f);
        const uint8_t lo = (uint8_t)(i & 0x0f);
        const uint8_t hiOk = (hi <= 3 || hi == 5 || hi == 6) ? hi : 0;
        const uint8_t loOk = (lo <= 3 || lo == 5 || lo == 6) ? lo : 0;
        s_nibbleLut[i] = (uint8_t)((hiOk << 4) | loOk);
    }
    s_lutReady = true;
}

// One controller's share of the frame: half-width rows at 4bpp, full height.
static inline uint32_t halfPlaneBytes(void) {
    return ((uint32_t)bbep.native_width / 4u) * (uint32_t)bbep.native_height;
}

bool splitPanelUsed(void) {
    return (bbep.iFlags & BBEP_SPLIT_BUFFER) != 0;
}

uint8_t splitPanelCs2Pin(void) {
    uint8_t p = globalConfig.displays[0].cs_pin_2;
    if (p == 0 || p == 0xFF) return 2;
    return p;
}

bool splitPanelGeometryOk(void) {
    return s_geometryOk;
}

void splitPanelConfigureGeometry(void) {
    s_geometryOk = false;
    s_streamOpen = false;
    s_onLeftHalf = true;
    s_halfWritten = 0;
    s_faulted = false;
    if (!splitPanelUsed()) return;

    const DisplayConfig& d = globalConfig.displays[0];
    if (d.pixel_width != bbep.native_width || d.pixel_height != bbep.native_height ||
        d.color_scheme != OD_COLOR_SCHEME_BWGBRY_SPLIT) {
        od_log_error("ERROR: split panel requires a %dx%d bwgbry_split (8) display config",
                     bbep.native_width, bbep.native_height);
        return;
    }
    s_geometryOk = true;
}

void splitPanelInitIo(void) {
    const DisplayConfig& d = globalConfig.displays[0];
    // CS2 before initIO: bbepInitIO() finishes by sending the panel's init
    // sequence, whose CMD_CS1_CS2 opcodes need iCS2Pin already assigned or the
    // shared registers reach only the primary controller.
    bbepSetCS2(&bbep, splitPanelCs2Pin());
    odWatchdogBreadcrumb(OD_WDT_PHASE_INIT_SEQ);
    odWatchdogFeed();   // bbepInitIO sends pInitFull internally (~240 s worst case)
    bbepInitIO(&bbep, d.dc_pin, d.reset_pin, d.busy_pin, d.cs_pin, d.data_pin, d.clk_pin, 8000000);
}

// bbepWriteCmd()/bbepWriteCmdData() open with `if (!is_awake) bbepWakeUp()`, and
// bbepWakeUp() toggles RST -- a hardware reset that would wipe the controller RAM
// we are part-way through filling. By the time any of this runs the panel really
// is awake (bbepInitIO -> bbepSendCMDSequence set the flag), so a zero here can
// only be stale bookkeeping; assert it rather than let it reset us mid-frame.
static inline void assumeAwake(void) { bbep.is_awake = 1; }

// Select one controller, or neither. cs_mode must already be CMD_CS_NONE, which
// is what stops the library toggling CS underneath us on every write.
static inline void selectControllers(bool cs1, bool cs2) {
    digitalWrite(bbep.iCSPin,  cs1 ? LOW : HIGH);   // upstream: iCSPin IS CS1 (iCS1Pin was deleted)
    digitalWrite(bbep.iCS2Pin, cs2 ? LOW : HIGH);
}

bool splitPanelBeginFrame(void) {
    if (s_streamOpen) splitPanelCloseFrame();
    s_onLeftHalf = true;
    s_halfWritten = 0;
    s_faulted = false;
    if (!splitPanelUsed() || !s_geometryOk) return false;

    // Never start blasting a half-plane into a controller that is still finishing
    // the previous refresh.
    if (bbepIsBusy(&bbep)) bbepWaitBusy(&bbep);

    assumeAwake();
    bbep.cs_mode = CMD_CS_NONE;
    selectControllers(true, false);
    bbepWriteCmd(&bbep, UC8151_DTM1);   // 0x10; also leaves DC HIGH == data mode
    s_streamOpen = true;
    return true;
}

// Cross to the second controller: releasing CS1 ends its burst, and the second
// DTM opens the right half-plane.
static bool advanceToRightHalf(void) {
    if (!s_streamOpen || !s_onLeftHalf) return false;
    selectControllers(false, true);
    assumeAwake();
    bbepWriteCmd(&bbep, UC8151_DTM1);
    s_onLeftHalf = false;
    s_halfWritten = 0;
    return true;
}

void splitPanelSinkBytes(const uint8_t* data, uint32_t len) {
    if (!data || len == 0) return;
    if (!s_streamOpen) { s_faulted = true; return; }
    buildNibbleLut();

    const uint32_t half = halfPlaneBytes();
    // 512 B: one bbepWriteData -> SPI.transferBytes per chunk. Large enough to
    // amortise the per-call overhead across a ~480 KB half-plane, and it swallows
    // the boot screen's 300-byte half-rows whole.
    uint8_t scratch[512];

    while (len > 0) {
        if (s_halfWritten >= half) {
            if (s_onLeftHalf) {
                if (!advanceToRightHalf()) { s_faulted = true; return; }
            } else {
                // More image than the panel has pixels. The bufferless shim this
                // replaces dropped these silently and still reported success.
                od_log_warn("Split panel: %lu excess image bytes dropped", (unsigned long)len);
                s_faulted = true;
                return;
            }
        }
        // uint32_t throughout: the predecessor computed this run length in a
        // uint16_t, so any chunk of 65536 bytes truncated to zero and spun forever.
        uint32_t run = half - s_halfWritten;
        if (run > len) run = len;
        if (run > sizeof(scratch)) run = (uint32_t)sizeof(scratch);

        for (uint32_t i = 0; i < run; i++) scratch[i] = s_nibbleLut[data[i]];
        bbepWriteData(&bbep, scratch, (int)run);   // CS_NONE: pure byte burst, no CS, no DC

        data += run;
        len -= run;
        s_halfWritten += run;
    }
}

bool splitPanelCloseFrame(void) {
    if (!s_streamOpen) return false;
    selectControllers(false, false);
    // Unconditional restore, matching bbepWriteImage4bppDual's epilogue: handing
    // CMD_CS_NONE back to library code that expects to drive CS itself means its
    // writes reach no controller, silently.
    bbep.cs_mode = CMD_CS1;
    s_streamOpen = false;

    const bool complete = !s_onLeftHalf && s_halfWritten >= halfPlaneBytes();
    if (s_faulted) {
        od_log_error("ERROR: split panel frame faulted; refusing to refresh");
        return false;
    }
    if (!complete) {
        // Short frame. The caller's byte-count check catches a client that
        // under-declares its total, but not a frame cut short by a geometry
        // mismatch or a dropped transport.
        od_log_error("ERROR: split panel frame short (half %d, %lu of %lu bytes)",
                     s_onLeftHalf ? 0 : 1, (unsigned long)s_halfWritten,
                     (unsigned long)halfPlaneBytes());
        return false;
    }
    return true;
}

void splitPanelPowerOff(void) {
    if (!splitPanelUsed()) return;
    // POFF to both controllers, then wait for the HV rails to bleed down before
    // anything considers cutting VDD.
    bbep.cs_mode = CMD_CS1_CS2;
    bbepCMD2(&bbep, UC8151_POFF, 0x00);
    bbepWaitBusy(&bbep);
    bbep.cs_mode = CMD_CS1;
}
