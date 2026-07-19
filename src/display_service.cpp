#include "display_service.h"

#include <Arduino.h>
#include <bb_epaper.h>
#include <string.h>
#include <Wire.h>
#include "structs.h"
#include "buzzer_control.h"
#include "sensor_sht40.h"
#include "sensor_bq27220.h"
#include "communication.h"
#include "encryption.h"
#include "boot_screen.h"
#include "touch_input.h"
#include "uzlib.h"
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
#include "display_seeed_gfx.h"
#endif

#ifdef TARGET_NRF
extern "C" {
#include "nrf_soc.h"
}
#include <bluefruit.h>
#include "ble_init.h"
#include "nrf.h"
#endif

#ifdef TARGET_ESP32
#include "ble_init.h"   // NimBLE-Arduino + BLE* aliases
#include "wifi_service.h"
#include <SPI.h>
#endif

extern BBEPDISP bbep;
extern struct GlobalConfig globalConfig;
extern uint8_t msd_payload[16];
extern uint8_t dynamicreturndata[11];
extern uint8_t rebootFlag;
extern uint8_t activeLedInstance;
extern bool connectionRequested;
extern uint8_t mloopcounter;
extern bool displayPowerState;
// EPD panel power state machine — variables DEFINED in main.h TU; enum +
// EPD_KEEPALIVE_MAX_S live in display_service.h.
extern volatile uint8_t pwrmgmState;
extern uint32_t pwrmgmOffDeadlineMs;
extern volatile uint8_t pwrmgmLock;
extern uint32_t directWriteStartTime;
extern uint32_t directWriteCompressedReceived;
extern uint8_t directWriteRefreshMode;
extern uint32_t directWriteTotalBytes;
extern uint16_t directWriteHeight;
extern uint16_t directWriteWidth;
extern uint32_t directWriteDecompressedTotal;
extern uint32_t directWriteBytesWritten;
extern bool directWritePlane2;
extern bool directWriteBitplanes;
extern bool directWriteCompressed;
extern bool directWriteActive;
extern uint8_t decompressionChunk[OPENDISPLAY_DECOMPRESSION_CHUNK_SIZE];
volatile bool epdRefreshInProgress = false;

extern uint32_t displayed_etag;

// 0x76 partial-write error codes come from the canonical opendisplay_protocol.h;
// use OD_ERR_PARTIAL_* directly at the call sites rather than shadowing them here
// (the OD_ERR_PIPE_START_* family reuses the same byte values with DIFFERENT
// meanings, so a local copy is a drift/mix-up hazard). For reference:
//   OD_ERR_PARTIAL_ETAG_MISMATCH  0x01   old_etag != displayed etag
//   OD_ERR_PARTIAL_RECT_OOB       0x03   rectangle out of panel bounds
//   OD_ERR_PARTIAL_RECT_ALIGN     0x04   x / width not a multiple of 8
//   OD_ERR_PARTIAL_FLAGS          0x05   bad / unsupported flags
//   OD_ERR_PARTIAL_STREAM         0x06   stream / length error
//   OD_ERR_PARTIAL_UNSUPPORTED    0x07   partial write unsupported (e.g. not 1bpp)

// TODO(protocol): the canonical header defines no partial-write flag constant;
// the 0x76 path reuses the bit0=compressed convention. Add an OD_PARTIAL_FLAG_*
// (or reuse a shared flag) upstream in opendisplay-protocol, then drop this local.
static const uint8_t PARTIAL_FLAG_COMPRESSED = 0x01u;
static const uint8_t PARTIAL_ALLOWED_FLAGS = PARTIAL_FLAG_COMPRESSED;

struct PartialStreamContext {
    bool active;
    bool compressed;
    uint8_t flags;
    uint32_t new_etag;
    uint16_t x;
    uint16_t y;
    uint16_t width;
    uint16_t height;
    uint32_t expected_stream_size;
    uint32_t plane_size;
    uint32_t bytes_received;
    uint32_t bytes_written;
    uint8_t current_plane;
    uint32_t start_time;
};

#ifdef TARGET_ESP32
extern BLEAdvertisementData* advertisementData;
extern BLEServer* pServer;
extern BLEService* pService;
#endif

void pwrmgm(bool onoff);
String getChipIdHex();
void writeSerial(String message, bool newLine);
void flushLog();
void bbepInitIO(BBEPDISP *pBBEP, uint8_t u8DC, uint8_t u8RST, uint8_t u8BUSY, uint8_t u8CS, uint8_t u8MOSI, uint8_t u8SCK, uint32_t u32Speed);
void bbepWakeUp(BBEPDISP *pBBEP);
void bbepSendCMDSequence(BBEPDISP *pBBEP, const uint8_t *pSeq);
void bbepRefresh(BBEPDISP *pBBEP, int iMode);
void bbepSleep(BBEPDISP *pBBEP, int iMode);
void bbepSetAddrWindow(BBEPDISP *pBBEP, int x, int y, int cx, int cy);
void bbepStartWrite(BBEPDISP *pBBEP, int iPlane);
void bbepWriteData(BBEPDISP *pBBEP, uint8_t *pData, int iLen);
void bbepFill(BBEPDISP *pBBEP, unsigned char ucColor, int iPlane);
void bbepWriteCmd(BBEPDISP *pBBEP, uint8_t cmd);
void bbepCMD2(BBEPDISP *pBBEP, uint8_t cmd1, uint8_t cmd2);
void bbepWaitBusy(BBEPDISP *pBBEP);
bool bbepIsBusy(BBEPDISP *pBBEP);
#ifdef BBEP_T133A01
void bbepSetCS2(BBEPDISP *pBBEP, uint8_t cs);
void bbepWriteCmdData(BBEPDISP *pBBEP, uint8_t cmd, const uint8_t *pData, int iLen);
void bbepStartDataStream(BBEPDISP *pBBEP, uint8_t cmd);
void bbepWriteDataStreamByte(BBEPDISP *pBBEP, uint8_t data);
void bbepEndDataStream(BBEPDISP *pBBEP);
#endif
void flashLed(uint8_t color, uint8_t brightness);
bool waitforrefresh(int timeout);

#ifdef TARGET_NRF
static bool nrfVbusPresent() {
    //ignored for now
    //return (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
    return false;
}
#else
static bool nrfVbusPresent() { return true; }
#endif

static void epdBsPinLowIfNrf() {
#ifdef TARGET_NRF
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);
#endif
}

// Battery boot: power-cycle the panel rail once. pwrmgm(true) already waits ~900 ms
// per enable; extra delays here are only for rail discharge between off/on.
static void prepareEpdRailForBoot() {
    epdBsPinLowIfNrf();
    pwrmgm(true);
#ifdef TARGET_NRF
    if (!nrfVbusPresent()) {
        delay(50);
        pwrmgm(false);
        delay(50);
        epdBsPinLowIfNrf();
        pwrmgm(true);
    }
#endif
}

#ifdef BBEP_T133A01
// CS2 must be set before bbepInitIO() so dual-chip init reaches both controllers.
static void e1004InitPanel(void) {
    const DisplayConfig& d = globalConfig.displays[0];
    bbepSetCS2(&bbep, e1004_cs2_pin());
    bbepInitIO(&bbep, d.dc_pin, d.reset_pin, d.busy_pin, d.cs_pin, d.data_pin, d.clk_pin, 8000000);
}

// Half-panel DTM for bwgbry_split: CS held for left half, then right (no FB).
static bool e1004GeometryOk = false;
static bool e1004StreamOpen = false;
static bool e1004OnLeftHalf = true;
static uint32_t e1004HalfBytesWritten = 0;
static uint32_t e1004HalfPlaneBytes = 0;

void e1004_end_plane(void);

static uint8_t e1004_panel_byte(uint8_t packed) {
    auto nibble = [](uint8_t c) -> uint8_t {
        c &= 0x0f;
        switch (c) {
        case 0x00: case 0x01: case 0x02: case 0x03: case 0x05: case 0x06:
            return c;
        default:
            return 0x00;
        }
    };
    return (uint8_t)((nibble(packed >> 4) << 4) | nibble(packed));
}

static void e1004_ccset_both(void) {
    uint8_t data = 0x01;
    digitalWrite(bbep.iCS2Pin, LOW);
    bbep.iCSPin = bbep.iCS1Pin;
    bbepWriteCmdData(&bbep, 0xe0, &data, 1);
    digitalWrite(bbep.iCS2Pin, HIGH);
    bbep.iCSPin = bbep.iCS1Pin;
    bbepWaitBusy(&bbep);
    delay(10);
}

static uint32_t e1004_half_plane_bytes(void) {
    return ((uint32_t)bbep.native_width / 4u) * (uint32_t)bbep.native_height;
}

bool e1004_begin_plane(void) {
    if (!e1004_panel_used() || !e1004GeometryOk) return false;
    if (e1004StreamOpen) e1004_end_plane();
    e1004_ccset_both();
    e1004HalfPlaneBytes = e1004_half_plane_bytes();
    e1004HalfBytesWritten = 0;
    e1004OnLeftHalf = true;
    bbep.iCSPin = bbep.iCS1Pin;
    bbepStartDataStream(&bbep, UC8151_DTM1);
    e1004StreamOpen = true;
    return true;
}

bool e1004_advance_to_cs2(void) {
    if (!e1004StreamOpen || !e1004OnLeftHalf) return false;
    bbepEndDataStream(&bbep);
    e1004OnLeftHalf = false;
    e1004HalfBytesWritten = 0;
    bbep.iCSPin = bbep.iCS2Pin;
    bbepStartDataStream(&bbep, UC8151_DTM1);
    return true;
}

void e1004_end_plane(void) {
    if (!e1004StreamOpen) return;
    bbepEndDataStream(&bbep);
    bbep.iCSPin = bbep.iCS1Pin;
    e1004StreamOpen = false;
    e1004OnLeftHalf = true;
    e1004HalfBytesWritten = 0;
}

void e1004_write_stream_bytes(const uint8_t* data, uint16_t len) {
    if (!e1004StreamOpen || !data || len == 0) return;
    uint8_t scratch[128];
    uint16_t off = 0;
    while (off < len) {
        uint16_t n = (uint16_t)(len - off);
        if (n > sizeof(scratch)) n = sizeof(scratch);
        for (uint16_t i = 0; i < n; i++) scratch[i] = e1004_panel_byte(data[off + i]);
        SPI.writeBytes(scratch, n);
        off = (uint16_t)(off + n);
    }
    e1004HalfBytesWritten += len;
}

static void e1004_sink_bytes(uint8_t* data, uint32_t len) {
    while (len > 0 && e1004StreamOpen) {
        if (e1004HalfPlaneBytes == 0) return;
        uint32_t space = e1004HalfPlaneBytes - e1004HalfBytesWritten;
        if (space == 0) {
            if (e1004OnLeftHalf) {
                if (!e1004_advance_to_cs2()) return;
                continue;
            }
            return;
        }
        uint16_t take = (len < space) ? (uint16_t)len : (uint16_t)((space > 0xFFFFu) ? 0xFFFFu : space);
        e1004_write_stream_bytes(data, take);
        data += take;
        len -= take;
        if (e1004OnLeftHalf && e1004HalfBytesWritten >= e1004HalfPlaneBytes) {
            if (!e1004_advance_to_cs2()) return;
        }
    }
}
#else
bool e1004_begin_plane(void) { return false; }
bool e1004_advance_to_cs2(void) { return false; }
void e1004_end_plane(void) {}
void e1004_write_stream_bytes(const uint8_t* data, uint16_t len) { (void)data; (void)len; }
#endif

// bb_epaper 71f6e70 replaced EP397/EP426 full-init RAM windows with SET_ORIENTATION
// (flip180=0 → 0x11=0x02 on 800-wide) while part inits and our partial helpers still
// use the pre-change windows. Re-apply those so full and partial share one map.
static void epdAlignCustomPartialRamMode(void) {
    uint8_t uc[4];
    if (bbep.type == EP397_800x480 || bbep.type == EP397_800x480_4GRAY) {
        bbepCMD2(&bbep, SSD1608_DATA_MODE, 0x01);
        bbepWriteCmd(&bbep, SSD1608_SET_RAMXPOS);
        uc[0] = 0x00; uc[1] = 0x00; uc[2] = 0x1f; uc[3] = 0x03;
        bbepWriteData(&bbep, uc, 4);
        bbepWriteCmd(&bbep, SSD1608_SET_RAMYPOS);
        uc[0] = 0xdf; uc[1] = 0x01; uc[2] = 0x00; uc[3] = 0x00;
        bbepWriteData(&bbep, uc, 4);
        bbepWriteCmd(&bbep, SSD1608_SET_RAMXCOUNT);
        uc[0] = 0x00; uc[1] = 0x00;
        bbepWriteData(&bbep, uc, 2);
        bbepWriteCmd(&bbep, SSD1608_SET_RAMYCOUNT);
        uc[0] = 0x00; uc[1] = 0x00;
        bbepWriteData(&bbep, uc, 2);
    } else if (bbep.type == EP426_800x480 || bbep.type == EP426_800x480_4GRAY) {
        bbepCMD2(&bbep, SSD1608_DATA_MODE, 0x02);
        bbepWriteCmd(&bbep, SSD1608_SET_RAMXPOS);
        uc[0] = 0x1f; uc[1] = 0x03; uc[2] = 0x00; uc[3] = 0x00;
        bbepWriteData(&bbep, uc, 4);
        bbepWriteCmd(&bbep, SSD1608_SET_RAMYPOS);
        uc[0] = 0x00; uc[1] = 0x00; uc[2] = 0xdf; uc[3] = 0x01;
        bbepWriteData(&bbep, uc, 4);
        bbepWriteCmd(&bbep, SSD1608_SET_RAMXCOUNT);
        uc[0] = 0x1f; uc[1] = 0x03;
        bbepWriteData(&bbep, uc, 2);
        bbepWriteCmd(&bbep, SSD1608_SET_RAMYCOUNT);
        uc[0] = 0x00; uc[1] = 0x00;
        bbepWriteData(&bbep, uc, 2);
    }
}

static void initBbepPanelSession() {
    const DisplayConfig& d = globalConfig.displays[0];
#ifdef BBEP_T133A01
    if (e1004_panel_used()) {
        e1004InitPanel();
        delay(200);
        return;
    }
#endif
    bbepInitIO(&bbep, d.dc_pin, d.reset_pin, d.busy_pin, d.cs_pin, d.data_pin, d.clk_pin, 8000000);
    bbepWakeUp(&bbep);
    bbepSendCMDSequence(&bbep, bbep.pInitFull);
    epdAlignCustomPartialRamMode();
    delay(200);
}

// ---------------------------------------------------------------------------
// EPD panel power session (keep-alive) — see the state-machine design.
// pwrmgm() owns the OFF<->(ACTIVE) rail transitions and is the sole rail actuator;
// these helpers own the ACTIVE<->WARM transitions plus the keep-alive timer.
// ---------------------------------------------------------------------------

// Which init sequence is loaded in the controller (partial vs full). Panel-init
// bookkeeping, not power state — stays file-static here (Phase 2a uses it).
static bool epdSessionInitWasPartial = false;
// Phase 2b plane-consistency flag: true after a successful partial refresh leaves
// both controller planes consistent. Cleared on ForceOff / full-frame acquire.
// Not consulted for fill-skip in Phase 1 (full-frame skip is unconditional-safe).
static bool epdPlanesPrepared = false;

static bool epdSessionUsesSeeed(void) {
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
    return seeed_driver_used();
#else
    return false;
#endif
}

// Keep-alive window from config: screen_timeout_seconds, clamped to EPD_KEEPALIVE_MAX_S;
// 0 (also the old-blob/factory default) -> Release powers the panel straight down.
// Forced to 0 on AXP2101 boards regardless of config (PMIC warm idle draw unmeasured) —
// announced on the log whenever the override suppresses a non-zero configured value.
static uint32_t epdKeepAliveWindowMs(void) {
    uint8_t s = globalConfig.power_option.screen_timeout_seconds;
    for (uint8_t i = 0; i < globalConfig.sensor_count; i++) {
        if (globalConfig.sensors[i].sensor_type == OD_SENSOR_TYPE_AXP2101) {
            if (s != 0) {
                writeSerial("[EPD session] AXP2101 present - keep-alive forced off (screen_timeout_seconds ignored)", true);
            }
            return 0;
        }
    }
    if (s > EPD_KEEPALIVE_MAX_S) s = EPD_KEEPALIVE_MAX_S;
    return (uint32_t)s * 1000;
}

// Cross-task try-lock. On nRF the Bluefruit write-callback task and the loop()
// task can both touch the session (a transfer begins Acquire on one while the
// keep-alive tick fires ForceOff on the other). Acquire/Release/ForceOff take it;
// the tick TRY-locks and skips its pass if held, so it can never rail-cut mid-init.
static void pwrmgmLockTake(void) {
    // MUST yield while waiting: on nRF this runs on the Bluefruit callback task,
    // which outranks the loop task holding the lock during the tick's ForceOff
    // (SPI ops + delay(50)). A bare busy-spin starves the lower-priority holder
    // forever on the single core (priority-inversion livelock); delay(1) is
    // vTaskDelay, which blocks the spinner so the holder can finish and release.
    while (__atomic_exchange_n(&pwrmgmLock, 1, __ATOMIC_ACQUIRE)) { delay(1); }
}
static bool pwrmgmLockTryTake(void) {
    return __atomic_exchange_n(&pwrmgmLock, 1, __ATOMIC_ACQUIRE) == 0;
}
static void pwrmgmLockGive(void) {
    __atomic_store_n(&pwrmgmLock, 0, __ATOMIC_RELEASE);
}

// Lock-held core (callers must hold pwrmgmLock). Split out so Release/Tick can
// power off without re-taking the non-recursive lock.
static void epdSessionForceOffLocked(void) {
    if (pwrmgmState == PWR_OFF) return;   // idempotent
    writeSerial("[EPD session] force off", true);
    if (epdSessionUsesSeeed()) {
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
        seeed_gfx_direct_sleep();
        // Rail is about to drop: force the next push to fully re-init the TCON
        // rather than wake() a power-cycled IT8951 (garbled refresh otherwise).
        seeed_gfx_mark_hw_deinitialized();
#endif
    } else {
        bbepSleep(&bbep, 1);
        delay(50);
    }
    pwrmgm(false);   // -> PWR_OFF, clears deadline
    epdPlanesPrepared = false;
}

// Bring the panel up for a transfer/refresh. Returns true iff it was COLD (rail
// was off) — callers may need to (re)open the address window regardless.
static bool epdSessionAcquire(bool partialInit) {
    pwrmgmLockTake();
    bool cold;
    if (pwrmgmState == PWR_OFF) {
        writeSerial("[EPD session] acquire: COLD bring-up", true);
        pwrmgm(true);   // -> PWR_ACTIVE (guarded; real transition)
        if (!epdSessionUsesSeeed()) {
            const DisplayConfig& d = globalConfig.displays[0];
#ifdef BBEP_T133A01
            if (e1004_panel_used()) {
                e1004InitPanel();
                epdSessionInitWasPartial = false;
            } else
#endif
            {
                bbepInitIO(&bbep, d.dc_pin, d.reset_pin, d.busy_pin, d.cs_pin, d.data_pin, d.clk_pin, 8000000);
                bbepWakeUp(&bbep);
                const uint8_t* initSeq = partialInit ? (bbep.pInitPart ? bbep.pInitPart : bbep.pInitFull)
                                                     : bbep.pInitFull;
                bbepSendCMDSequence(&bbep, initSeq);
                epdAlignCustomPartialRamMode();
                epdSessionInitWasPartial = partialInit;
            }
        }
        cold = true;
    } else {
        // WARM re-acquire (or, defensively, an already-ACTIVE re-entry).
        writeSerial(pwrmgmState == PWR_ACTIVE ? "[EPD session] acquire: already ACTIVE (defensive)"
                                              : "[EPD session] acquire: WARM re-acquire", true);
        pwrmgmState = PWR_ACTIVE;
        pwrmgmOffDeadlineMs = 0;   // cancel keep-alive
        // Phase 1: full re-init on warm re-acquire (HW reset => registers identical
        // to cold, safest). Phase 2a will skip bbepWakeUp + resend only on change.
        if (!epdSessionUsesSeeed()) {
#ifdef BBEP_T133A01
            if (e1004_panel_used()) {
                epdSessionInitWasPartial = false;
            } else
#endif
            {
                bbepWakeUp(&bbep);
                const uint8_t* initSeq = partialInit ? (bbep.pInitPart ? bbep.pInitPart : bbep.pInitFull)
                                                     : bbep.pInitFull;
                bbepSendCMDSequence(&bbep, initSeq);
                epdAlignCustomPartialRamMode();
                epdSessionInitWasPartial = partialInit;
            }
        }
        cold = false;
    }
    pwrmgmLockGive();
    return cold;
}

// Finish a transfer/refresh. On success (and when keep-alive is enabled) the panel
// stays powered + AWAKE and enters PWR_WARM with an armed deadline; otherwise it is
// powered fully down now.
static void epdSessionRelease(bool refreshSuccess) {
    pwrmgmLockTake();
    if (pwrmgmState == PWR_OFF) { pwrmgmLockGive(); return; }   // nothing to release
    uint32_t window = epdKeepAliveWindowMs();
    if (window == 0 || !refreshSuccess) {
        writeSerial(refreshSuccess ? "[EPD session] release: keep-alive disabled, powering off"
                                   : "[EPD session] release: refresh failed, powering off", true);
        epdSessionForceOffLocked();
    } else {
        pwrmgmState = PWR_WARM;
        pwrmgmOffDeadlineMs = millis() + window;
        // Controller stays AWAKE (no bbepSleep; is_awake stays 1); rail/SPI stay up.
        writeSerial("[EPD session] release: panel warm-idle, off in " + String(window) + " ms", true);
    }
    pwrmgmLockGive();
}

void epdSessionForceOff(void) {
    pwrmgmLockTake();
    epdSessionForceOffLocked();
    pwrmgmLockGive();
}

void epdSessionTick(void) {
    if (pwrmgmState != PWR_WARM) return;   // fast pre-check (only WARM arms the timer)
    if (!pwrmgmLockTryTake()) return;      // held by a transfer -> skip this pass
    // Re-check under the lock: a transfer may have moved us out of WARM meanwhile.
    if (pwrmgmState == PWR_WARM && (int32_t)(millis() - pwrmgmOffDeadlineMs) >= 0) {
        writeSerial("[EPD session] keep-alive expired — powering panel off", true);
        epdSessionForceOffLocked();
    }
    pwrmgmLockGive();
}

bool epdSessionIsWarm(void) {
    return pwrmgmState == PWR_WARM;
}

static bool refreshBootScreenFull() {
    if (!writeBootScreenWithQr()) {
        writeSerial("Boot screen render failed", true);
        return false;
    }
    writeSerial("EPD refresh: FULL (boot)", true);
    touchSuspendForEpdRefresh();
    bbepRefresh(&bbep, REFRESH_FULL);
    return waitforrefresh(60);
}

static void cleanup_partial_write_state(void);
static bool panel_skips_bbep_set_addr_window(void);
static void partial_set_addr_window(BBEPDISP *pBBEP, int x, int y, int cx, int cy);
static bool partial_consume_bytes(uint8_t* data, uint32_t len);
static void partial_prepare_panel_ram(void);
static bool partial_write_to_panel(int refreshMode);
static bool partial_write_stream_bytes(uint8_t* data, uint32_t len);
static bool zlib_stream_to_direct_write(const uint8_t* data, uint32_t len, bool final);
static bool zlib_stream_to_partial_write(const uint8_t* data, uint32_t len, bool final);
static uint32_t calc_controller_plane_bytes(uint16_t width, uint16_t height);
static uint32_t parse_be_u32(const uint8_t* data);
static void send_direct_write_nack(uint8_t opcode, uint8_t error, bool cleanupState);
static PartialStreamContext partialCtx = {};

// Direct-write session-setup helpers (shared by legacy 0x70 START and PIPE 0x80 START)
// and the shared END/refresh tail (shared by legacy 0x72 END, PIPE 0x82 END, and
// both auto-complete paths). Declared here; defined below near the direct-write handlers.
static void directWriteComputeGeometry(bool compressed);
static void directWriteActivatePanel(void);
static void directWriteFinishAndRefresh(uint8_t* data, uint16_t len, uint8_t endOpcode);

// PIPE_WRITE (0x0080-0x0082) sliding-window receive state + reorder queue. Declared
// early so the quiet-logging predicates below can consult pipeState.active. The
// reorder array is a file static (not in the struct) so both targets pay it once.
static PipeWriteState pipeState = {};
static PipeReorderSlot pipeReorder[PIPE_REORDER_SLOTS];

void checkPartialWriteTimeout(void) {
    if (partialCtx.active && partialCtx.start_time > 0 &&
        (millis() - partialCtx.start_time) > 900000UL) {
        writeSerial("ERROR: Partial write timeout - cleaning up stuck state", true);
        cleanup_partial_write_state();
        // A pipe-partial transfer shares partialCtx: also clear pipeState so a zombie
        // pipeState.active can't misroute later 0x0081 frames into the dead partialCtx.
        if (pipeState.partial) resetPipeWriteState();
    }
}

// Disconnect hook: a partial session (0x76 or pipe-partial) powers the panel via
// partial_prepare_panel_ram but never sets directWriteActive, so the disconnect
// handlers' cleanupDirectWriteState gate misses it and the panel would stay
// powered until the 15-min watchdog. cleanup_partial_write_state is file-static;
// this wrapper gives the BLE callbacks a safe no-op-when-idle entry point.
void cleanupPartialWriteOnDisconnect(void) {
    if (partialCtx.active) cleanup_partial_write_state();
}

#define AXP2101_SLAVE_ADDRESS 0x34
#define AXP2101_REG_POWER_STATUS 0x00
#define AXP2101_REG_DC_ONOFF_DVM_CTRL 0x80
#define AXP2101_REG_LDO_ONOFF_CTRL0 0x90
#define AXP2101_REG_DC_VOL0_CTRL 0x82
#define AXP2101_REG_LDO_VOL2_CTRL 0x94
#define AXP2101_REG_LDO_VOL3_CTRL 0x95
#define AXP2101_REG_POWER_WAKEUP_CTL 0x26
#define AXP2101_REG_ADC_CHANNEL_CTRL 0x30
#define AXP2101_REG_ADC_DATA_BAT_VOL_H 0x34
#define AXP2101_REG_ADC_DATA_VBUS_VOL_H 0x36
#define AXP2101_REG_ADC_DATA_SYS_VOL_H 0x38
#define AXP2101_REG_BAT_PERCENT_DATA 0xA4
#define AXP2101_REG_PWRON_STATUS 0x20
#define AXP2101_REG_IRQ_ENABLE1 0x40
#define AXP2101_REG_IRQ_ENABLE2 0x41
#define AXP2101_REG_IRQ_ENABLE3 0x42
#define AXP2101_REG_IRQ_ENABLE4 0x43
#define AXP2101_REG_IRQ_STATUS1 0x44
#define AXP2101_REG_IRQ_STATUS2 0x45
#define AXP2101_REG_IRQ_STATUS3 0x46
#define AXP2101_REG_IRQ_STATUS4 0x47
#define AXP2101_REG_LDO_ONOFF_CTRL1 0x91
#define FONT_BASE_WIDTH 8
#define FONT_BASE_HEIGHT 8
#define FONT_SMALL_THRESHOLD 264

extern const uint8_t writelineFont[] PROGMEM;
extern uint8_t staticWhiteRow[680];
extern uint8_t staticRowBuffer[BOOT_ROW_BUFFER_SIZE];
extern char staticLineBuffer[256];

int bbepSetPanelType(BBEPDISP *pBBEP, int iPanel);
void bbepSetRotation(BBEPDISP *pBBEP, int iRotation);

int mapEpd(int id){
    switch(id) {
        case 0x0000: return EP_PANEL_UNDEFINED;
        case 0x0001: return EP42_400x300;
        case 0x0002: return EP42B_400x300;
        case 0x0003: return EP213_122x250;
        case 0x0004: return EP213B_122x250;
        case 0x0005: return EP293_128x296;
        case 0x0006: return EP294_128x296;
        case 0x0007: return EP295_128x296;
        case 0x0008: return EP295_128x296_4GRAY;
        case 0x0009: return EP266_152x296;
        case 0x000A: return EP102_80x128;
        case 0x000B: return EP27B_176x264;
        case 0x000C: return EP29R_128x296;
        case 0x000D: return EP122_192x176;
        case 0x000E: return EP154R_152x152;
        case 0x000F: return EP42R_400x300;
        case 0x0010: return EP42R2_400x300;
        case 0x0011: return EP37_240x416;
        case 0x0012: return EP37B_240x416;
        case 0x0013: return EP213_104x212;
        case 0x0014: return EP75_800x480;
        case 0x0015: return EP75_800x480_4GRAY;
        case 0x0016: return EP75_800x480_4GRAY_V2;
        case 0x0017: return EP29_128x296;
        case 0x0018: return EP29_128x296_4GRAY;
        case 0x0019: return EP213R_122x250;
        case 0x001A: return EP154_200x200;
        case 0x001B: return EP154B_200x200;
        case 0x001C: return EP266YR_184x360;
        case 0x001D: return EP29YR_128x296;
        case 0x001E: return EP29YR_168x384;
        case 0x001F: return EP583_648x480;
        case 0x0020: return EP296_128x296;
        case 0x0021: return EP26R_152x296;
        case 0x0022: return EP73_800x480;
        case 0x0023: return EP73_SPECTRA_800x480;
        case 0x0024: return EP74R_640x384;
        case 0x0025: return EP583R_600x448;
        case 0x0026: return EP75R_800x480;
        case 0x0027: return EP426_800x480;
        case 0x0028: return EP426_800x480_4GRAY;
        case 0x0029: return EP29R2_128x296;
        case 0x002A: return EP41_640x400;
        case 0x002B: return EP81_SPECTRA_1024x576;
        case 0x002C: return EP7_960x640;
        case 0x002D: return EP213R2_122x250;
        case 0x002E: return EP29Z_128x296;
        case 0x002F: return EP29Z_128x296_4GRAY;
        case 0x0030: return EP213Z_122x250;
        case 0x0031: return EP213Z_122x250_4GRAY;
        case 0x0032: return EP154Z_152x152;
        case 0x0033: return EP579_792x272;
        case 0x0034: return EP213YR_122x250;
        case 0x0035: return EP37YR_240x416;
        case 0x0036: return EP35YR_184x384;
        case 0x0037: return EP397YR_800x480;
        case 0x0038: return EP154YR_200x200;
        case 0x0039: return EP266YR2_184x360;
        case 0x003A: return EP42YR_400x300;
        case 0x003B: return EP75_800x480_GEN2;
        case 0x003C: return EP75_800x480_4GRAY_GEN2;
        case 0x003D: return EP215YR_160x296;
        case 0x003E: return EP1085_1360x480;
        case 0x003F: return EP31_240x320;
        case 0x0040: return EP75YR_800x480;
        case 0x0041: return EP_PANEL_UNDEFINED;
#ifdef BBEP_T133A01
        case OD_PANEL_IC_EP133A_SPECTRA_1200X1600: return EP133A_SPECTRA_1200x1600; // 0x0042, Seeed reTerminal E1004
#else
        case 0x0042: return EP_PANEL_UNDEFINED;
#endif
        case 0x0043: return EP154_200x200_4GRAY;
        case 0x0044: return EP42B_400x300_4GRAY;
        case 0x0045: return EP397_800x480;
        case 0x0046: return EP397_800x480_4GRAY;
        case 0x0047: return EP368_792x528;
        case 0x0048: return EP368_792x528_4GRAY;
        case 0x0049: return EP213ZZ_122x250;
        case 0x004A: return EP40_SPECTRA_400x600;
        case 0x004B: return EP27_176x264;
        case 0x004C: return EP27_176x264_4GRAY;
        default: return EP_PANEL_UNDEFINED;
    }
}

bool seeed_driver_used(void) {
#if !defined(TARGET_ESP32) || !defined(OPENDISPLAY_SEEED_GFX)
    return false;
#else
    if (globalConfig.display_count < 1) return false;
    const struct DisplayConfig& d = globalConfig.displays[0];
    if (d.panel_ic_type != OD_PANEL_IC_ED103TC2_1872X1404 &&
        d.panel_ic_type != OD_PANEL_IC_ED103TC2_1872X1404_4GRAY) return false;
    if (d.display_technology != 0 && d.display_technology != 1) return false;
    return true;
#endif
}

bool e1004_panel_used(void) {
#ifdef BBEP_T133A01
    if (globalConfig.display_count < 1) return false;
    return globalConfig.displays[0].panel_ic_type == OD_PANEL_IC_EP133A_SPECTRA_1200X1600;
#else
    return false;
#endif
}

// cs_pin_2; 0 or 0xFF defaults to GPIO2.
uint8_t e1004_cs2_pin(void) {
    uint8_t p = globalConfig.displays[0].cs_pin_2;
    if (p == 0 || p == 0xFF) return 2;
    return p;
}

bool waitforrefresh(int timeout){
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
    if (seeed_driver_used()) return seeed_gfx_wait_refresh(timeout);
#endif
    if (e1004_panel_used() && !bbepIsBusy(&bbep)) {
        // bbepRefresh already waited; idle here means refresh finished.
        writeSerial("Refresh completed inside bb_epaper", true);
        return true;
    }
    // Poll at 10 ms (was 100 ms) so a ~0.5 s refresh returns up to ~90 ms sooner.
    // BUSY asserts within µs of MASTER_ACTIVATE, so the i==0 "never went busy"
    // error check stays valid at a 10 ms first poll. Loop bound scales x10
    // (timeout*100 iterations of 10 ms); dot cadence every 50 iters keeps ~0.5 s/dot.
    for (size_t i = 0; i < (size_t)(timeout * 100); i++){
        delay(10);
        if(i % 50 == 0) writeSerial(".", false);
        if(!bbepIsBusy(&bbep)){
            if(i == 0){
                writeSerial("ERROR: Epaper not busy after refresh command - refresh may not have started", true);
                return false;
            }
            writeSerial(".", true);
            writeSerial("Refresh took ", false);
            writeSerial((String)((float)i / 100), false);
            writeSerial(" seconds", true);
//            delay(200);   // EXTRA DELAY HERE IS UNNEEDED AND JUST SLOWS THINGS DOWN
            return true;
        }
    }
    writeSerial("Refresh timed out", true);
    return false;
}

#ifdef TARGET_ESP32
static bool s_wire_open_display_ready = false;
static int8_t s_wire_sda_pin = -1;
static int8_t s_wire_scl_pin = -1;
static uint32_t s_wire_clock_hz = 0;

static bool wireBeginForOpenDisplay(int sda, int scl, uint32_t hz) {
    // Do not pinMode() before begin on ESP32 — periman must hand pins to the I2C driver.
    if (Wire.begin(sda, scl, hz)) {
        Wire.setClock(hz);
        s_wire_sda_pin = (int8_t)sda;
        s_wire_scl_pin = (int8_t)scl;
        s_wire_clock_hz = hz;
        s_wire_open_display_ready = true;
        return true;
    }
    if (hz > 100000u && Wire.begin(sda, scl, 100000u)) {
        writeSerial("NOTE: I2C fallback to 100kHz (SDA=GPIO" + String(sda) + " SCL=GPIO" + String(scl) + ")", true);
        Wire.setClock(100000u);
        s_wire_sda_pin = (int8_t)sda;
        s_wire_scl_pin = (int8_t)scl;
        s_wire_clock_hz = 100000u;
        s_wire_open_display_ready = true;
        return true;
    }
    writeSerial("ERROR: Wire.begin failed (SDA=GPIO" + String(sda) + " SCL=GPIO" + String(scl) + ")", true);
    return false;
}
#endif

static bool i2cDataBusValid(uint8_t bus_id) {
    if (bus_id >= globalConfig.data_bus_count) {
        return false;
    }
    const struct DataBus& bus = globalConfig.data_buses[bus_id];
    return bus.bus_type == 0x01 && bus.pin_1 != 0xFF && bus.pin_2 != 0xFF;
}

bool openDisplayI2cBusConfigured(void) {
    for (uint8_t i = 0; i < globalConfig.data_bus_count; i++) {
        if (i2cDataBusValid(i)) {
            return true;
        }
    }
    return false;
}

void invalidateOpenDisplayWire(void) {
#ifdef TARGET_ESP32
    if (s_wire_open_display_ready) {
        Wire.end();
    }
    s_wire_open_display_ready = false;
#endif
}

bool initOrRestoreWireForBus(uint8_t bus_id) {
#ifdef TARGET_ESP32
    if (bus_id == 0xFF) {
        bus_id = 0;
    }
    if (!i2cDataBusValid(bus_id)) {
        return false;
    }
    const struct DataBus& bus = globalConfig.data_buses[bus_id];
    uint32_t hz = bus.bus_speed_hz ? bus.bus_speed_hz : 100000u;
    int sda = (int)bus.pin_2;
    int scl = (int)bus.pin_1;
    if (s_wire_open_display_ready && s_wire_sda_pin == sda && s_wire_scl_pin == scl) {
        return true;
    }
    if (s_wire_open_display_ready) {
        Wire.end();
        s_wire_open_display_ready = false;
    }
    if (!wireBeginForOpenDisplay(sda, scl, hz)) {
        s_wire_open_display_ready = false;
        return false;
    }
    return true;
#else
    (void)bus_id;
    initOrRestoreWireForOpenDisplay();
    return true;
#endif
}

void initOrRestoreWireForOpenDisplay(void) {
#ifdef TARGET_ESP32
    if (globalConfig.data_bus_count > 0 && i2cDataBusValid(0)) {
        (void)initOrRestoreWireForBus(0);
        return;
    }
    if (!s_wire_open_display_ready) {
        if (Wire.begin()) {
            s_wire_open_display_ready = true;
        }
    }
#else
    if (!openDisplayI2cBusConfigured()) {
        return;
    }
    if (i2cDataBusValid(0)) {
        const struct DataBus& bus = globalConfig.data_buses[0];
        pinMode(bus.pin_1, (bus.pullups & 0x01) ? INPUT_PULLUP : INPUT);
        pinMode(bus.pin_2, (bus.pullups & 0x02) ? INPUT_PULLUP : INPUT);
    }
    Wire.begin();
    if (i2cDataBusValid(0) && globalConfig.data_buses[0].bus_speed_hz > 0) {
        Wire.setClock(globalConfig.data_buses[0].bus_speed_hz);
    }
#endif
}

void initDataBuses(){
    writeSerial("=== Initializing Data Buses ===", true);
    if(globalConfig.data_bus_count == 0){
        writeSerial("No data buses configured", true);
        return;
    }
    for(uint8_t i = 0; i < globalConfig.data_bus_count; i++){
        struct DataBus* bus = &globalConfig.data_buses[i];
        if(bus->bus_type == 0x01){ // I2C bus
            writeSerial("Initializing I2C bus " + String(i) + " (instance " + String(bus->instance_number) + ")", true);
            if(bus->pin_1 == 0xFF || bus->pin_2 == 0xFF){
                writeSerial("ERROR: Invalid I2C pins for bus " + String(i) + " (SCL=" + String(bus->pin_1) + ", SDA=" + String(bus->pin_2) + ")", true);
                continue;
            }
            uint32_t busSpeed = (bus->bus_speed_hz > 0) ? bus->bus_speed_hz : 100000;
            if(i == 0){
                #ifdef TARGET_ESP32
                initOrRestoreWireForOpenDisplay();
                #endif
                #ifdef TARGET_NRF
                pinMode(bus->pin_1, INPUT);
                pinMode(bus->pin_2, INPUT);
                if(bus->pullups & 0x01){
                    pinMode(bus->pin_1, INPUT_PULLUP);
                }
                if(bus->pullups & 0x02){
                    pinMode(bus->pin_2, INPUT_PULLUP);
                }
                Wire.begin(); // Uses default I2C pins
                Wire.setClock(busSpeed);
                writeSerial("NOTE: nRF52840 using default I2C pins (config pins: SCL=" + String(bus->pin_1) + ", SDA=" + String(bus->pin_2) + ")", true);
                #endif
                writeSerial("I2C bus " + String(i) + " initialized: SCL=pin" + String(bus->pin_1) + ", SDA=pin" + String(bus->pin_2) + ", Speed=" + String(busSpeed) + "Hz", true);
            } else {
                writeSerial("I2C bus " + String(i) + " configured (init on demand): SCL=pin" + String(bus->pin_1) +
                    ", SDA=pin" + String(bus->pin_2) + ", Speed=" + String(busSpeed) + "Hz", true);
            }
        }
        else if(bus->bus_type == 0x02){
            writeSerial("SPI bus " + String(i) + " detected (not yet implemented)", true);
            writeSerial("  Instance: " + String(bus->instance_number), true);
        }
        else{
            writeSerial("WARNING: Unknown bus type 0x" + String(bus->bus_type, HEX) + " for bus " + String(i), true);
        }
    }
    writeSerial("=== Data Bus Initialization Complete ===", true);
}

void initio(){
    writeSerial("[initio] >> LEDs", true); flushLog();
    if(globalConfig.led_count > 0){
        for (uint8_t i = 0; i < globalConfig.led_count; i++) {
            struct LedConfig* led = &globalConfig.leds[i];
            bool invertRed = (led->led_flags & 0x01) != 0;
            bool invertGreen = (led->led_flags & 0x02) != 0;
            bool invertBlue = (led->led_flags & 0x04) != 0;
            bool invertLed4 = (led->led_flags & 0x08) != 0;
                if (led->led_1_r != 0xFF) {
                    pinMode(led->led_1_r, OUTPUT);
                    digitalWrite(led->led_1_r, invertRed ? HIGH : LOW);
                }
                if (led->led_2_g != 0xFF) {
                    pinMode(led->led_2_g, OUTPUT);
                    digitalWrite(led->led_2_g, invertGreen ? HIGH : LOW);
                }
                if (led->led_3_b != 0xFF) {
                    pinMode(led->led_3_b, OUTPUT);
                    digitalWrite(led->led_3_b, invertBlue ? HIGH : LOW);
                }
                if (led->led_4 != 0xFF) {
                    pinMode(led->led_4, OUTPUT);
                    digitalWrite(led->led_4, invertLed4 ? HIGH : LOW);
                }
        }
        for (uint8_t i = 0; i < globalConfig.led_count; i++) {
            if (globalConfig.leds[i].led_type == 0) {
                activeLedInstance = i;
#ifdef TARGET_NRF
                if (nrfVbusPresent())
#endif
                {
                    flashLed(0xE0, 15);
                    flashLed(0x1C, 15);
                    flashLed(0x03, 15);
                    flashLed(0xFF, 15);
                }
            }
        }
    }
    writeSerial("[initio] >> initPassiveBuzzers", true); flushLog();
    initPassiveBuzzers();
    writeSerial("[initio] >> pwr_pin", true); flushLog();
    if(globalConfig.system_config.pwr_pin != 0xFF){
    pinMode(globalConfig.system_config.pwr_pin, OUTPUT);
    digitalWrite(globalConfig.system_config.pwr_pin, LOW);
    }
    else{
        writeSerial("Power pin not set", true);
    }
    writeSerial("[initio] >> initDataBuses", true); flushLog();
    initDataBuses();
    writeSerial("[initio] >> initSensors", true); flushLog();
    initSensors();
    writeSerial("[initio] << done", true); flushLog();
}

void scanI2CDevices(){
    writeSerial("=== Scanning I2C Bus for Devices ===", true);
    initOrRestoreWireForOpenDisplay();
    uint8_t deviceCount = 0;
    uint8_t foundDevices[128];
    for(uint8_t address = 0x08; address < 0x78; address++){
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();
        if(error == 0){
            foundDevices[deviceCount] = address;
            deviceCount++;
            writeSerial("I2C device found at address 0x" + String(address, HEX) + " (" + String(address) + ")", true);
        }
        else if(error == 4){
            writeSerial("ERROR: Unknown error at address 0x" + String(address, HEX), true);
        }
    }
    if(deviceCount == 0){
        writeSerial("No I2C devices found on bus", true);
    } else {
        writeSerial("Found " + String(deviceCount) + " I2C device(s)", true);
        writeSerial("Device addresses: ", true);
        String addrList = "";
        for(uint8_t i = 0; i < deviceCount; i++){
            if(i > 0) addrList += ", ";
            addrList += "0x" + String(foundDevices[i], HEX);
        }
        writeSerial(addrList, true);
    }
    writeSerial("=== I2C Scan Complete ===", true);
}

void initSensors(){
    writeSerial("=== Initializing Sensors ===", true);
    if(globalConfig.sensor_count == 0){
        writeSerial("No sensors configured", true);
        return;
    }
    for(uint8_t i = 0; i < globalConfig.sensor_count; i++){
        struct SensorData* sensor = &globalConfig.sensors[i];
        writeSerial("Initializing sensor " + String(i) + " (instance " + String(sensor->instance_number) + ")", true);
        writeSerial("  Type: 0x" + String(sensor->sensor_type, HEX), true);
        writeSerial("  Bus ID: " + String(sensor->bus_id), true);
        if(sensor->sensor_type == OD_SENSOR_TYPE_AXP2101){
            writeSerial("  Detected AXP2101 PMIC sensor", true);
        }
        else if(sensor->sensor_type == OD_SENSOR_TYPE_TEMPERATURE){
            writeSerial("  Temperature sensor (initialization not implemented)", true);
        }
        else if(sensor->sensor_type == OD_SENSOR_TYPE_HUMIDITY){
            writeSerial("  Humidity sensor (initialization not implemented)", true);
        }
        else if(sensor->sensor_type == OD_SENSOR_TYPE_SHT40){
            writeSerial("  SHT40 (I2C + MSD slot)", true);
        }
        else if(sensor->sensor_type == OD_SENSOR_TYPE_BQ27220){
            writeSerial("  BQ27220 fuel gauge (MSD voltage + optional dynamic SOC/status bytes)", true);
        }
        else{
            writeSerial("  Unknown sensor type 0x" + String(sensor->sensor_type, HEX), true);
        }
    }
    initSht40Sensors();
    initBq27220Sensors();
    writeSerial("=== Sensor Initialization Complete ===", true);
}

void initAXP2101(uint8_t busId){
    pinMode(21, OUTPUT);
    digitalWrite(21, LOW);
    delay(100);
    digitalWrite(21, HIGH);
    writeSerial("=== Initializing AXP2101 PMIC ===", true);
    if(busId >= globalConfig.data_bus_count){
        writeSerial("ERROR: Invalid bus ID " + String(busId) + " (only " + String(globalConfig.data_bus_count) + " buses configured)", true);
        return;
    }
    struct DataBus* bus = &globalConfig.data_buses[busId];
    if(bus->bus_type != 0x01){
        writeSerial("ERROR: Bus " + String(busId) + " is not an I2C bus", true);
        return;
    }
    if(!initOrRestoreWireForBus(busId)){
        writeSerial("ERROR: Failed to (re)init I2C bus " + String(busId) + " for AXP2101", true);
        return;
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    uint8_t error = Wire.endTransmission();
    if(error != 0){
        writeSerial("ERROR: AXP2101 not found at address 0x" + String(AXP2101_SLAVE_ADDRESS, HEX) + " (error: " + String(error) + ")", true);
        return;
    }
    writeSerial("AXP2101 detected at address 0x" + String(AXP2101_SLAVE_ADDRESS, HEX), true);
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_POWER_STATUS);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            uint8_t status = Wire.read();
            writeSerial("Power status: 0x" + String(status, HEX), true);
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_DC_VOL0_CTRL);
    Wire.write(0x12);
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("DCDC1 voltage set to 3.3V", true);
    } else {
        writeSerial("ERROR: Failed to set DCDC1 voltage", true);
    }
    delay(10);
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_DC_ONOFF_DVM_CTRL);
    error = Wire.endTransmission();
    uint8_t dcEnable = 0x00;
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            dcEnable = Wire.read();
        }
    }
    dcEnable |= 0x01;
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_DC_ONOFF_DVM_CTRL);
    Wire.write(dcEnable);
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("DCDC1 enabled (3.3V)", true);
    } else {
        writeSerial("ERROR: Failed to enable DCDC1", true);
    }
    delay(10);
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_ONOFF_CTRL0);
    error = Wire.endTransmission();
    uint8_t aldoEnable = 0x00;
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            aldoEnable = Wire.read();
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_VOL2_CTRL);
    error = Wire.endTransmission();
    uint8_t aldo3VolReg = 0x00;
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            aldo3VolReg = Wire.read();
        }
    }
    aldo3VolReg = (aldo3VolReg & 0xE0) | 0x1C;
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_VOL2_CTRL);
    Wire.write(aldo3VolReg);
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("ALDO3 voltage set to 3.3V", true);
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_VOL3_CTRL);
    error = Wire.endTransmission();
    uint8_t aldo4VolReg = 0x00;
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            aldo4VolReg = Wire.read();
        }
    }
    aldo4VolReg = (aldo4VolReg & 0xE0) | 0x1C;
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_VOL3_CTRL);
    Wire.write(aldo4VolReg);
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("ALDO4 voltage set to 3.3V", true);
    }
    aldoEnable |= 0x0C;
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_ONOFF_CTRL0);
    Wire.write(aldoEnable);
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("ALDO3 and ALDO4 enabled (3.3V)", true);
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_POWER_WAKEUP_CTL);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            uint8_t wakeupCtl = Wire.read();
            writeSerial("Wakeup control: 0x" + String(wakeupCtl, HEX), true);
            if(wakeupCtl & 0x01){
                writeSerial("Wakeup already enabled", true);
            } else {
                Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
                Wire.write(AXP2101_REG_POWER_WAKEUP_CTL);
                Wire.write(wakeupCtl | 0x01);
                error = Wire.endTransmission();
                if(error == 0){
                    writeSerial("Wakeup enabled", true);
                }
            }
        }
    }
    writeSerial("=== AXP2101 PMIC Initialization Complete ===", true);
}

void readAXP2101Data(){
    writeSerial("=== Reading AXP2101 PMIC Data ===", true);
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    uint8_t error = Wire.endTransmission();
    if(error != 0){
        writeSerial("ERROR: AXP2101 not found at address 0x" + String(AXP2101_SLAVE_ADDRESS, HEX), true);
        return;
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_ADC_CHANNEL_CTRL);
    Wire.write(0xFF);
    error = Wire.endTransmission();
    delay(10);
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_POWER_STATUS);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)2);
        if(Wire.available() >= 2){
            uint8_t status1 = Wire.read();
            uint8_t status2 = Wire.read();
            writeSerial("Power Status 1: 0x" + String(status1, HEX), true);
            writeSerial("Power Status 2: 0x" + String(status2, HEX), true);
            bool batteryPresent = (status1 & 0x20) != 0;
            bool charging = (status1 & 0x04) != 0;
            bool vbusPresent = (status1 & 0x08) != 0;
            writeSerial("Battery Present: " + String(batteryPresent ? "Yes" : "No"), true);
            writeSerial("Charging: " + String(charging ? "Yes" : "No"), true);
            writeSerial("VBUS Present: " + String(vbusPresent ? "Yes" : "No"), true);
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_PWRON_STATUS);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            uint8_t pwronStatus = Wire.read();
            writeSerial("Power On Status: 0x" + String(pwronStatus, HEX), true);
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_ADC_DATA_BAT_VOL_H);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)2);
        if(Wire.available() >= 2){
            uint8_t batVolH = Wire.read();
            uint8_t batVolL = Wire.read();
            uint16_t batVolRaw = ((uint16_t)batVolH << 4) | (batVolL & 0x0F);
            float batVoltage = batVolRaw * 0.5;
            writeSerial("Battery Voltage: " + String(batVoltage, 1) + " mV (" + String(batVoltage / 1000.0, 2) + " V)", true);
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_ADC_DATA_VBUS_VOL_H);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)2);
        if(Wire.available() >= 2){
            uint8_t vbusVolH = Wire.read();
            uint8_t vbusVolL = Wire.read();
            uint16_t vbusVolRaw = ((uint16_t)vbusVolH << 4) | (vbusVolL & 0x0F);
            float vbusVoltage = vbusVolRaw * 1.7;
            writeSerial("VBUS Voltage: " + String(vbusVoltage, 1) + " mV (" + String(vbusVoltage / 1000.0, 2) + " V)", true);
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_ADC_DATA_SYS_VOL_H);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)2);
        if(Wire.available() >= 2){
            uint8_t sysVolH = Wire.read();
            uint8_t sysVolL = Wire.read();
            uint16_t sysVolRaw = ((uint16_t)sysVolH << 4) | (sysVolL & 0x0F);
            float sysVoltage = sysVolRaw * 1.4;
            writeSerial("System Voltage: " + String(sysVoltage, 1) + " mV (" + String(sysVoltage / 1000.0, 2) + " V)", true);
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_BAT_PERCENT_DATA);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            uint8_t batPercent = Wire.read();
            if(batPercent <= 100){
                writeSerial("Battery Percentage: " + String(batPercent) + "%", true);
            } else {
                writeSerial("Battery Percentage: Not available (fuel gauge may be disabled)", true);
            }
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_DC_ONOFF_DVM_CTRL);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            uint8_t dcEnable = Wire.read();
            writeSerial("DC Enable Status: 0x" + String(dcEnable, HEX), true);
            writeSerial("  DCDC1: " + String((dcEnable & 0x01) ? "ON" : "OFF"), true);
            writeSerial("  DCDC2: " + String((dcEnable & 0x02) ? "ON" : "OFF"), true);
            writeSerial("  DCDC3: " + String((dcEnable & 0x04) ? "ON" : "OFF"), true);
            writeSerial("  DCDC4: " + String((dcEnable & 0x08) ? "ON" : "OFF"), true);
            writeSerial("  DCDC5: " + String((dcEnable & 0x10) ? "ON" : "OFF"), true);
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_ONOFF_CTRL0);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            uint8_t aldoEnable = Wire.read();
            writeSerial("ALDO Enable Status: 0x" + String(aldoEnable, HEX), true);
            writeSerial("  ALDO1: " + String((aldoEnable & 0x01) ? "ON" : "OFF"), true);
            writeSerial("  ALDO2: " + String((aldoEnable & 0x02) ? "ON" : "OFF"), true);
            writeSerial("  ALDO3: " + String((aldoEnable & 0x04) ? "ON" : "OFF"), true);
            writeSerial("  ALDO4: " + String((aldoEnable & 0x08) ? "ON" : "OFF"), true);
        }
    }
    writeSerial("=== AXP2101 Data Read Complete ===", true);
}

void powerDownAXP2101(){
    writeSerial("=== Powering Down AXP2101 PMIC Rails ===", true);
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    uint8_t error = Wire.endTransmission();
    if(error != 0){
        writeSerial("ERROR: AXP2101 not found at address 0x" + String(AXP2101_SLAVE_ADDRESS, HEX) + " (error: " + String(error) + ")", true);
        return;
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_IRQ_ENABLE1);
    Wire.write(0x00);
    error = Wire.endTransmission();
    if(error == 0){
        Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
        Wire.write(AXP2101_REG_IRQ_ENABLE2);
        Wire.write(0x00);
        error = Wire.endTransmission();
    }
    if(error == 0){
        Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
        Wire.write(AXP2101_REG_IRQ_ENABLE3);
        Wire.write(0x00);
        error = Wire.endTransmission();
    }
    if(error == 0){
        Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
        Wire.write(AXP2101_REG_IRQ_ENABLE4);
        Wire.write(0x00);
        error = Wire.endTransmission();
    }
    if(error == 0){
        Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
        Wire.write(AXP2101_REG_IRQ_STATUS1);
        Wire.write(0xFF);
        error = Wire.endTransmission();
    }
    if(error == 0){
        Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
        Wire.write(AXP2101_REG_IRQ_STATUS2);
        Wire.write(0xFF);
        error = Wire.endTransmission();
    }
    if(error == 0){
        Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
        Wire.write(AXP2101_REG_IRQ_STATUS3);
        Wire.write(0xFF);
        error = Wire.endTransmission();
    }
    if(error == 0){
        Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
        Wire.write(AXP2101_REG_IRQ_STATUS4);
        Wire.write(0xFF);
        error = Wire.endTransmission();
        if(error == 0){
            writeSerial("All IRQs disabled and status cleared", true);
        }
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_DC_ONOFF_DVM_CTRL);
    error = Wire.endTransmission();
    uint8_t dcEnable = 0x00;
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            dcEnable = Wire.read();
        }
    }
    dcEnable &= 0x01;
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_DC_ONOFF_DVM_CTRL);
    Wire.write(dcEnable);
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("DC2-5 disabled (DC1 kept enabled)", true);
    } else {
        writeSerial("ERROR: Failed to disable DC2-5", true);
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_ONOFF_CTRL1);
    Wire.write(0x00);
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("BLDO1-2, CPUSLDO, DLDO1-2 disabled", true);
    } else {
        writeSerial("ERROR: Failed to disable BLDO/CPUSLDO/DLDO rails", true);
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_ONOFF_CTRL0);
    error = Wire.endTransmission();
    uint8_t aldoEnable = 0x00;
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            aldoEnable = Wire.read();
        }
    }
    aldoEnable &= ~0x0F;
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_LDO_ONOFF_CTRL0);
    Wire.write(aldoEnable);
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("ALDO1-4 disabled", true);
    } else {
        writeSerial("ERROR: Failed to disable ALDO rails", true);
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_POWER_WAKEUP_CTL);
    error = Wire.endTransmission();
    uint8_t wakeupCtrl = 0x00;
    if(error == 0){
        Wire.requestFrom(AXP2101_SLAVE_ADDRESS, (uint8_t)1);
        if(Wire.available()){
            wakeupCtrl = Wire.read();
        }
    }
    if(!(wakeupCtrl & 0x04)) {
        wakeupCtrl |= 0x04;
    }
    if(wakeupCtrl & 0x08) {
        wakeupCtrl &= ~0x08;
    }
    if(!(wakeupCtrl & 0x10)) {
        wakeupCtrl |= 0x10;
    }
    wakeupCtrl |= 0x80;
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_POWER_WAKEUP_CTL);
    Wire.write(wakeupCtrl);
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("AXP2101 wake-up configured and sleep mode enabled", true);
    } else {
        writeSerial("ERROR: Failed to configure AXP2101 sleep mode", true);
    }
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    Wire.write(AXP2101_REG_ADC_CHANNEL_CTRL);
    Wire.write(0x00);
    error = Wire.endTransmission();
    if(error == 0){
        writeSerial("All ADC channels disabled", true);
    } else {
        writeSerial("ERROR: Failed to disable ADC channels", true);
    }
    writeSerial("=== AXP2101 PMIC Rails Powered Down ===", true);
}

static void renderChar_4BPP(uint8_t* rowBuffer, const uint8_t* fontData, int fontRow, int charIdx, int startX, int charWidth, int pitch, int fontScale) {
    for (int col = 0; col < charWidth; col += fontScale) {
        uint8_t fontByte;
        int fontCol = col / fontScale;
        if (fontCol == 0 || fontCol > 7) {
            fontByte = 0x00;
        } else {
            fontByte = fontData[fontCol - 1];
        }
        uint8_t pixelBit = (fontByte >> fontRow) & 0x01;
        uint8_t pixelNibble = (pixelBit == 1) ? 0x0 : 0xF;
        for (int s = 0; s < fontScale; s++) {
            int pixelX = startX + charIdx * charWidth + col + s;
            if (pixelX >= globalConfig.displays[0].pixel_width) break;
            int bytePos = pixelX / 2;
            if (bytePos >= pitch) break;
            if ((pixelX % 2) == 0) {
                rowBuffer[bytePos] = (rowBuffer[bytePos] & 0x0F) | (pixelNibble << 4);
            } else {
                rowBuffer[bytePos] = (rowBuffer[bytePos] & 0xF0) | pixelNibble;
            }
        }
    }
}

static void renderChar_2BPP(uint8_t* rowBuffer, const uint8_t* fontData, int fontRow, int charIdx, int startX, int charWidth, int pitch, uint8_t colorScheme, int fontScale) {
    uint8_t whiteCode = (colorScheme == OD_COLOR_SCHEME_GRAY4) ? 0x03 : 0x01;
    int pixelsPerByte = 4;
    for (int col = 0; col < charWidth; col += pixelsPerByte) {
        uint8_t pixelByte = 0;
        for (int p = 0; p < pixelsPerByte; p++) {
            int pixelX = startX + charIdx * charWidth + col + p;
            if (pixelX >= globalConfig.displays[0].pixel_width) break;
            uint8_t fontByte;
            int fontCol = (col + p) / fontScale;
            if (fontCol == 0 || fontCol > 7) {
                fontByte = 0x00;
            } else {
                fontByte = fontData[fontCol - 1];
            }
            uint8_t pixelBit = (fontByte >> fontRow) & 0x01;
            uint8_t pixelValue = (pixelBit == 1) ? 0x00 : whiteCode;
            pixelByte |= (pixelValue << (6 - p * 2));
        }
        int bytePos = (startX + charIdx * charWidth + col) / 4;
        if (bytePos < pitch) {
            rowBuffer[bytePos] = pixelByte;
        }
    }
}

static void renderChar_1BPP(uint8_t* rowBuffer, const uint8_t* fontData, int fontRow, int charIdx, int startX, int charWidth, int pitch, int fontScale) {
    for (int col = 0; col < charWidth; col += fontScale) {
        uint8_t fontByte;
        int fontCol = col / fontScale;
        if (fontCol == 0 || fontCol > 7) {
            fontByte = 0x00;
        } else {
            fontByte = fontData[fontCol - 1];
        }
        uint8_t pixelBit = (fontByte >> fontRow) & 0x01;
        for (int s = 0; s < fontScale; s++) {
            int pixelX = startX + charIdx * charWidth + col + s;
            if (pixelX >= globalConfig.displays[0].pixel_width) break;
            int bytePos = pixelX / 8;
            int bitPos = 7 - (pixelX % 8);
            if (bytePos < pitch) {
                if (pixelBit == 1) {
                    rowBuffer[bytePos] &= ~(1 << bitPos);
                }
            }
        }
    }
}

void initDisplay(){
    writeSerial("=== Initializing Display ===", true);
    if(globalConfig.display_count > 0){
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
    if (seeed_driver_used()) {
        pwrmgm(true);
        writeSerial("Display: Seeed_GFX (panel_ic " + String(globalConfig.displays[0].panel_ic_type) + ", " +
                    String(globalConfig.displays[0].pixel_width) + "x" + String(globalConfig.displays[0].pixel_height) + ", " +
                    String(getBitsPerPixel()) + " bpp)", true);
        seeed_gfx_epaper_begin();
        if (opnd_seeed_tcon_busy_timeout_occurred()) {
            writeSerial("Seeed_GFX init failed (TCON busy timeout) — skipping boot refresh", true);
            // begin() sets seeed_gfx_hw_initialized=true even on TCON timeout; clear it
            // so the next push takes the full begin() path. Raw pwrmgm(false) (not
            // ForceOff): don't send sleep to a TCON that just timed out on BUSY.
            seeed_gfx_mark_hw_deinitialized();
            pwrmgm(false);
            return;
        }
        writeSerial(String("Height: ") + String(globalConfig.displays[0].pixel_height), true);
        writeSerial(String("Width: ") + String(globalConfig.displays[0].pixel_width), true);
        if (! (globalConfig.displays[0].transmission_modes & OD_TRANSMISSION_MODE_CLEAR_ON_BOOT)){
            writeBootScreenWithQr();
            writeSerial("EPD refresh: FULL (boot, Seeed)", true);
            touchSuspendForEpdRefresh();
            seeed_gfx_full_update();
            waitforrefresh(60);
            // Boot ends PWR_OFF (no keep-alive at boot). ForceOff sleeps the TCON
            // (+ clears the Seeed hw-init flag, Commit 5) and cuts the rail.
            epdSessionForceOff();
            touchResumeAfterEpdRefresh();
        } else {
            // CLEAR_ON_BOOT: begin() succeeded (hw flag true) but the rail is being
            // cut — ForceOff sleeps the TCON, clears the hw flag, and cuts the rail.
            epdSessionForceOff();
        }
    } else
#endif
    {
        prepareEpdRailForBoot();
        memset(&bbep, 0, sizeof(BBEPDISP));
        int panelType = mapEpd(globalConfig.displays[0].panel_ic_type);
        bbepSetPanelType(&bbep, panelType);
        int rotation = globalConfig.displays[0].rotation * 90;
#ifdef BBEP_T133A01
        e1004GeometryOk = false;
        if (e1004_panel_used()) {
            rotation = 0;  // host bakes rotation into packed image
            if (globalConfig.displays[0].pixel_width != bbep.native_width ||
                globalConfig.displays[0].pixel_height != bbep.native_height ||
                globalConfig.displays[0].color_scheme != OD_COLOR_SCHEME_BWGBRY_SPLIT) {
                writeSerial("ERROR: E1004 requires a 1200x1600 bwgbry_split (8) display config", true);
            } else {
                e1004GeometryOk = true;
            }
        }
#endif
        bbepSetRotation(&bbep, rotation);
        writeSerial(String("Height: ") + String(globalConfig.displays[0].pixel_height), true);
        writeSerial(String("Width: ") + String(globalConfig.displays[0].pixel_width), true);
        initBbepPanelSession();
        if (! (globalConfig.displays[0].transmission_modes & OD_TRANSMISSION_MODE_CLEAR_ON_BOOT)){
            bool bootOk = refreshBootScreenFull();
            if (!bootOk && !nrfVbusPresent()) {
                writeSerial("Boot refresh failed on battery — re-powering panel and retrying", true);
                touchResumeAfterEpdRefresh();
                pwrmgm(false);
                delay(200);
                prepareEpdRailForBoot();
                initBbepPanelSession();
                bootOk = refreshBootScreenFull();
            }
            if (!bootOk) {
                writeSerial("Boot screen refresh did not complete", true);
            }
            // Boot ends PWR_OFF (no keep-alive at boot). pwrmgm(true) in boot set
            // PWR_ACTIVE, so ForceOff sleeps the controller + cuts the rail cleanly.
            epdSessionForceOff();
            touchResumeAfterEpdRefresh();
        } else {
            // CLEAR_ON_BOOT: initBbepPanelSession left the controller awake —
            // ForceOff sleeps it before the rail cut (raw pwrmgm(false) skipped that).
            epdSessionForceOff();
        }
    }
    }
    else{
        writeSerial("No display found", true);
    }
}


int getplane() {
    uint8_t colorScheme = globalConfig.displays[0].color_scheme;
    if (colorScheme == OD_COLOR_SCHEME_MONO || colorScheme == OD_COLOR_SCHEME_GRAY16) return PLANE_0;
    if (colorScheme == OD_COLOR_SCHEME_BWR || colorScheme == OD_COLOR_SCHEME_BWY) return PLANE_0;
    if (colorScheme == OD_COLOR_SCHEME_GRAY4) return PLANE_1;
    return PLANE_1;
}

int getBitsPerPixel() {
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
    if (globalConfig.display_count > 0 &&
        globalConfig.displays[0].panel_ic_type == OD_PANEL_IC_ED103TC2_1872X1404_4GRAY) {
        return 4;
    }
#endif
    if (globalConfig.displays[0].color_scheme == OD_COLOR_SCHEME_BWGBRY ||
        globalConfig.displays[0].color_scheme == OD_COLOR_SCHEME_BWGBRY_SPLIT) return 4;
    if (globalConfig.displays[0].color_scheme == OD_COLOR_SCHEME_BWRY) return 2;
    if (globalConfig.displays[0].color_scheme == OD_COLOR_SCHEME_GRAY4) return 2;
    return 1;
}

static float readBatteryVoltageUncached() {
    if (bq27220IsConfigured()) {
        float gaugeV = bq27220BatteryVoltageVolts();
        if (gaugeV >= 0.0f) {
            return gaugeV;
        }
    }
    if (globalConfig.power_option.battery_sense_pin == 0xFF) return -1.0;
    uint8_t sensePin = globalConfig.power_option.battery_sense_pin;
    uint8_t enablePin = globalConfig.power_option.battery_sense_enable_pin;
    uint16_t scalingFactor = globalConfig.power_option.voltage_scaling_factor;
    pinMode(sensePin, INPUT);
    if (enablePin != 0xFF) {
        pinMode(enablePin, OUTPUT);
        digitalWrite(enablePin, HIGH);
        delay(10);
    }
    const int numSamples = 10;
    uint32_t adcSum = 0;
    for (int i = 0; i < numSamples; i++) {
        adcSum += analogRead(sensePin);
        delay(2);
    }
    uint32_t adcAverage = adcSum / numSamples;
    if (enablePin != 0xFF) {
        digitalWrite(enablePin, LOW);
    }
    if (scalingFactor > 0) return (adcAverage * scalingFactor) / (100000.0);
    return -1.0;
}

static constexpr uint32_t kBatteryVoltageTtlMs = 30000u;
float readBatteryVoltage() {
    static uint32_t lastReadMs = 0;
    static float cachedVoltage = -1.0f;
    static bool haveReading = false;
    if (haveReading && (uint32_t)(millis() - lastReadMs) < kBatteryVoltageTtlMs) {
        return cachedVoltage;
    }
    cachedVoltage = readBatteryVoltageUncached();
    lastReadMs = millis();
    haveReading = true;
    return cachedVoltage;
}

float readChipTemperature() {
#ifdef TARGET_ESP32
    return temperatureRead();
#elif defined(TARGET_NRF)
    int32_t tempRaw = 0;
    uint32_t err_code = sd_temp_get(&tempRaw);
    if (err_code == 0) return tempRaw * 0.25f;
    return -999.0;
#else
    return -999.0;
#endif
}

void updatemsdata(){
    // writeSerial("updatemsdata() called (mloopcounter: " + String(mloopcounter) + ")", true);
    pollSht40SensorsForMsd();
    pollBq27220ForMsd();
    float batteryVoltage = readBatteryVoltage();
    float chipTemperature = readChipTemperature();
    uint16_t batteryVoltage10mv = 0;
    if (batteryVoltage >= 0.0f) {
        uint16_t batteryVoltageMv = (uint16_t)(batteryVoltage * 1000.0f);
        batteryVoltage10mv = batteryVoltageMv / 10;
        if (batteryVoltage10mv > 511) {
            batteryVoltage10mv = 511;
        }
    }
    int16_t tempEncoded = (int16_t)((chipTemperature + 40.0f) * 2.0f);
    if (tempEncoded < 0) tempEncoded = 0;
    else if (tempEncoded > 255) tempEncoded = 255;
    uint8_t temperatureByte = (uint8_t)tempEncoded;
    uint8_t batteryVoltageLowByte = (uint8_t)(batteryVoltage10mv & 0xFF);
    uint8_t statusByte = (((batteryVoltage10mv >> 8) & 0x01) ? OD_MSD_STATUS_BATTERY_VOLTAGE_BIT8 : 0) |
                         (rebootFlag ? OD_MSD_STATUS_REBOOT_FLAG : 0) |
                         (connectionRequested ? OD_MSD_STATUS_CONNECTION_REQUESTED : 0) |
                         (((uint8_t)(mloopcounter << OD_MSD_STATUS_MAIN_LOOP_COUNTER_SHIFT)) & OD_MSD_STATUS_MAIN_LOOP_COUNTER_MASK);
    // Build the 16-byte advertisement via the canonical wire struct (all little-endian),
    // then copy into the global msd_payload[16] that the BLE adv APIs below consume.
    struct MsdAdvertisement m;
    memset(&m, 0, sizeof m);
    m.company_id = 0x2446;
    memcpy(m.dynamic, dynamicreturndata, sizeof m.dynamic);
    m.chip_temperature = temperatureByte;
    m.battery_voltage_low = batteryVoltageLowByte;
    m.status = statusByte;
    memcpy(msd_payload, &m, sizeof m);
#ifdef TARGET_NRF
    static uint8_t prev_msd_payload_nrf[16] = {0xFF};
    if (memcmp(prev_msd_payload_nrf, msd_payload, 16) == 0) {
        mloopcounter++;
        mloopcounter &= 0x0F;
        return;
    }
    memcpy(prev_msd_payload_nrf, msd_payload, 16);
    Bluefruit.Advertising.clearData();
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addName();
    Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, msd_payload, 16);
    ble_nrf_apply_adv_interval();
    Bluefruit.Advertising.setFastTimeout(1);
    Bluefruit.Advertising.stop();
    Bluefruit.Advertising.start(0);
#endif
#ifdef TARGET_ESP32
    if (advertisementData != nullptr) {
        static uint8_t prev_msd_payload[16] = {0xFF};
        if (memcmp(prev_msd_payload, msd_payload, 16) == 0) {
            mloopcounter++;
            mloopcounter &= 0x0F;
            return;
        }
        memcpy(prev_msd_payload, msd_payload, 16);
        advertisementData->setManufacturerData(msd_payload, 16);
        BLEAdvertising *pAdvertising = (pServer != nullptr) ? pServer->getAdvertising() : BLEDevice::getAdvertising();
        // Only rebuild+restart advertising while disconnected. The former connected
        // branch rebuilt *advertisementData but never pushed it via
        // setAdvertisementData(), so it was dead work — dropped.
        if (pAdvertising != nullptr && !(pServer != nullptr && pServer->getConnectedCount() > 0)) {
            pAdvertising->stop();
            BLEAdvertisementData freshAdvertisementData;
            static String savedDeviceName = "";
            if (savedDeviceName.length() == 0) savedDeviceName = "OD" + getChipIdHex();
            freshAdvertisementData.setName(savedDeviceName.c_str());
            freshAdvertisementData.setFlags(0x06);
            freshAdvertisementData.setManufacturerData(msd_payload, 16);
            *advertisementData = freshAdvertisementData;
            // setAdvertisementData() must be the last data call before start():
            // enableScanResponse()/setPreferredParams() reset NimBLE's custom-data
            // flag and would make start() drop this manufacturer-data payload.
            pAdvertising->setAdvertisementData(freshAdvertisementData);
            delay(50);
            pAdvertising->start();
        }
    }
    opendisplay_mdns_update_msd_txt();
#endif
    mloopcounter++;
    mloopcounter &= 0x0F;
}

// --- Quiet image-write logging ---------------------------------------------
// An image push arrives as a 0x70 start, many 0x71 data frames, and a 0x72 end.
// Logging every frame + its ack floods the UART (~1 MB of text for a 1.3 MB
// image) and, once the TX buffer fills, throttles the transfer itself. Instead
// we log the first frame in full, a 5%-step percentage meter thereafter, and
// the final frame + chunk total at completion. imageWriteLogQuiet{Cmd,Ack}()
// let communication.cpp suppress the per-frame command/ack spam accordingly.
static uint32_t imgLogTotalBytes;    // expected payload for this stream
static uint32_t imgLogChunks;        // 0x71 frames seen this stream
static uint8_t  imgLogLastStep;      // last 5% step printed (pct/5)
static uint16_t imgLogLastLen;       // length of most recent frame
static uint8_t  imgLogLastHead[16];  // first bytes of most recent frame
static uint8_t  imgLogLastHeadLen;   // valid bytes in imgLogLastHead
static uint32_t imgLogStartMs;       // millis() at stream start (for throughput)

static String imgLogHex(const uint8_t* buf, uint8_t n) {
    String s;
    for (uint8_t i = 0; i < n; i++) {
        if (i > 0) s += " ";
        if (buf[i] < 16) s += "0";
        s += String(buf[i], HEX);
    }
    return s;
}

static void imageWriteLogReset(void) {
    imgLogTotalBytes = 0;
    imgLogChunks = 0;
    imgLogLastStep = 0;
    imgLogLastLen = 0;
    imgLogLastHeadLen = 0;
    imgLogStartMs = 0;
}

static void imageWriteLogStart(uint32_t totalBytes) {
    imgLogTotalBytes = totalBytes;
    imgLogStartMs = millis();
    writeSerial("DW start: " + String(totalBytes) + " bytes expected", true);
}

static void imageWriteLogChunk(const uint8_t* data, uint16_t len) {
    imgLogChunks++;
    imgLogLastLen = len;
    imgLogLastHeadLen = (len < sizeof(imgLogLastHead)) ? (uint8_t)len : (uint8_t)sizeof(imgLogLastHead);
    memcpy(imgLogLastHead, data, imgLogLastHeadLen);
    if (imgLogChunks == 1) {
        writeSerial("DW frame 1: " + String(len) + " bytes: " + imgLogHex(imgLogLastHead, imgLogLastHeadLen), true);
        if (len > 0 && imgLogTotalBytes > 0) {
            uint32_t est = (imgLogTotalBytes + len - 1) / len;
            writeSerial("DW expecting ~" + String(est) + " chunks", true);
        }
    }
}

static void imageWriteLogProgress(uint32_t written, uint32_t total) {
    if (total == 0) return;
    uint32_t pct = (uint64_t)written * 100u / total;
    if (pct >= 100) return;                 // completion summary covers 100%
    uint8_t step = (uint8_t)(pct / 5u);
    if (step <= imgLogLastStep) return;
    imgLogLastStep = step;
    writeSerial("DW " + String(pct) + "% (" + String(imgLogChunks) + " chunks, " +
                String(written) + "/" + String(total) + " bytes)", true);
}

static void imageWriteLogFinish(uint32_t written, uint32_t total) {
    writeSerial("DW final frame " + String(imgLogChunks) + ": " + String(imgLogLastLen) +
                " bytes: " + imgLogHex(imgLogLastHead, imgLogLastHeadLen), true);
    uint32_t elapsedMs = millis() - imgLogStartMs;   // unsigned wrap-safe over one stream
    String rate = "n/a";
    if (elapsedMs > 0) rate = String((float)written / 1.024f / (float)elapsedMs, 1);  // bytes/ms /1.024 = KB/s
    writeSerial("DW complete: " + String(imgLogChunks) + " chunks, " +
                String(written) + "/" + String(total) + " bytes, " +
                String(elapsedMs / 1000.0f, 2) + " s, " + rate + " KB/s", true);
}

bool imageWriteLogQuietCmd(void) {
    return (directWriteActive || partialCtx.active || pipeState.active) && imgLogChunks >= 1;
}

bool imageWriteLogQuietAck(void) {
    return (directWriteActive || partialCtx.active || pipeState.active) && imgLogChunks >= 2;
}

// True when this raw frame is a mid-stream image-write data chunk (command
// header 0x0071, unencrypted) whose per-frame BLE-receive/queue logging should
// be suppressed. Lets the receive callback and queue drain in the other files
// silence their spam without duplicating the stream-state check.
bool imageWriteLogQuietFrame(const uint8_t* data, uint16_t len) {
    return len >= 2 && data[0] == 0x00 &&
           (data[1] == 0x71 || data[1] == 0x81) && imageWriteLogQuietCmd();
}
// ---------------------------------------------------------------------------

// Consume one compressed direct-write payload into the panel controller. Returns
// false on overflow guard or decompress/write failure; the CALLER owns cleanup and
// ACK/NACK emission (legacy 0x71 caller keeps its byte-identical acks; PIPE reuses
// the bool core without acking per frame). Does NOT advance directWriteCompressedReceived.
bool handleDirectWriteCompressedData(uint8_t* data, uint16_t len) {
    if (len > UINT32_MAX - directWriteCompressedReceived) {
        return false;
    }
    if (!zlib_stream_to_direct_write(data, len, false)) {
        return false;
    }
    return true;
}

// True when the active display uses the bb_epaper 4-gray scheme (two 1-bit
// controller planes). The Seeed driver path has its own 4bpp handling.
static inline bool directWriteIsGray4(void) {
    return (globalConfig.displays[0].color_scheme == OD_COLOR_SCHEME_GRAY4)
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
        && !seeed_driver_used()
#endif
        ;
}

// Two-plane uploads (4-gray and BWR/BWY) arrive as two pre-split, row-padded 1-bit
// controller planes concatenated (plane0 then plane1). 4-gray is already gray-coded
// host-side (py-opendisplay applies the panel's gray LUT, matching bbepSetPixel4Gray:
// plane0 <- stored bit0, plane1 <- stored bit1); BWR/BWY send plane0 = BW (palette
// index 1) then plane1 = accent (index 2). Both share the identical byte layout, so
// stream the bytes to the panel, switching from PLANE_0 to PLANE_1 at the single-plane
// boundary - no on-device de-interleave or 2bpp frame buffer. directWriteBytesWritten
// is the running total across both planes, so the compressed and uncompressed paths
// share this one plane-split implementation.
static void streamGray4Bytes(const uint8_t* buf, uint32_t len) {
    const uint32_t planeBytes = (((uint32_t)directWriteWidth + 7u) / 8u) * directWriteHeight;
    uint32_t off = 0;
    while (off < len && directWriteBytesWritten < 2u * planeBytes) {
        if (directWriteBytesWritten == 0u) {
            bbepSetAddrWindow(&bbep, 0, 0, directWriteWidth, directWriteHeight);
            bbepStartWrite(&bbep, PLANE_0);
        } else if (directWriteBytesWritten == planeBytes) {
            bbepSetAddrWindow(&bbep, 0, 0, directWriteWidth, directWriteHeight);
            bbepStartWrite(&bbep, PLANE_1);
        }
        const uint32_t limit = (directWriteBytesWritten < planeBytes) ? planeBytes : 2u * planeBytes;
        uint32_t take = len - off;
        if (directWriteBytesWritten + take > limit) take = limit - directWriteBytesWritten;
        bbepWriteData(&bbep, (uint8_t*)(buf + off), (int)take);
        off += take;
        directWriteBytesWritten += take;
    }
}

static void directWriteSinkBytes(uint8_t* data, uint32_t len) {
#ifdef BBEP_T133A01
    if (e1004_panel_used()) {
        if (e1004GeometryOk) e1004_sink_bytes(data, len);
    } else
#endif
    {
        bbepWriteData(&bbep, data, (int)len);
    }
    directWriteBytesWritten += len;
}

static bool directWriteTouchSuspended = false;

void cleanupDirectWriteState(bool refreshDisplay) {
    directWriteActive = false;
    directWriteCompressed = false;
    directWriteBitplanes = false;
    directWritePlane2 = false;
    directWriteBytesWritten = 0;
    directWriteCompressedReceived = 0;
    directWriteDecompressedTotal = 0;
    directWriteWidth = 0;
    directWriteHeight = 0;
    directWriteTotalBytes = 0;
    directWriteRefreshMode = 0;
    directWriteStartTime = 0;
    // Panel power acts only while a transfer/refresh is actually in flight
    // (PWR_ACTIVE). refreshDisplay==true is a terminal teardown (disconnect,
    // 15-min timeout, mid-stream error) -> power fully off. refreshDisplay==false
    // is the post-refresh path from directWriteFinishAndRefresh -> release to WARM
    // so keep-alive holds the rail for the next push.
    if (pwrmgmState == PWR_ACTIVE) {
        if (refreshDisplay) epdSessionForceOff();
        else                epdSessionRelease(true);
    }
#ifdef BBEP_T133A01
    if (e1004_panel_used()) e1004_end_plane();
#endif
    if (directWriteTouchSuspended) {
        touchResumeAfterEpdRefresh();
        directWriteTouchSuspended = false;
    }
}

// Computes the panel geometry and total controller byte count for a direct-write
// session and records the compressed flag. Sets directWrite{Compressed,Bitplanes,
// Plane2,Width,Height,TotalBytes}. No panel I/O, no acks. Shared by 0x70 and 0x80.
static void directWriteComputeGeometry(bool compressed) {
    uint8_t colorScheme = globalConfig.displays[0].color_scheme;
    directWriteBitplanes = (colorScheme == OD_COLOR_SCHEME_BWR || colorScheme == OD_COLOR_SCHEME_BWY);
    directWritePlane2 = false;
    directWriteCompressed = compressed;
    directWriteWidth = globalConfig.displays[0].pixel_width;
    directWriteHeight = globalConfig.displays[0].pixel_height;
    if (directWriteBitplanes) directWriteTotalBytes = 2u * (((uint32_t)directWriteWidth + 7u) / 8u) * directWriteHeight;
    else {
        // Panel RAM is row-padded: each row occupies ceil(w / pixelsPerByte) bytes, and the
        // Python sender row-pads every plane (np.packbits(axis=1)). Size FLAT and we under-count
        // on width-not-divisible-by-8 panels (e.g. 122-wide EP213), auto-completing before the
        // bottom rows are written. Size row-padded to match sender + the gray4/bitplane paths.
        uint32_t w = (uint32_t)directWriteWidth;
        uint32_t h = (uint32_t)directWriteHeight;
        int bitsPerPixel = getBitsPerPixel();
        if (bitsPerPixel == 4) directWriteTotalBytes = ((w + 1u) / 2u) * h;       // ceil(w/2) bytes/row
        else if (bitsPerPixel == 2) directWriteTotalBytes = ((w + 3u) / 4u) * h;  // ceil(w/4) bytes/row
        else directWriteTotalBytes = calc_controller_plane_bytes(directWriteWidth, directWriteHeight); // ceil(w/8) bytes/row
    }
    // 4-gray arrives as two concatenated 1bpp planes (plane0 ++ plane1), streamed to
    // PLANE_0/PLANE_1. Both compressed and uncompressed transports feed bytes through
    // streamGray4Bytes as chunks arrive.
    const bool gray4 = directWriteIsGray4();
    if (gray4) directWriteTotalBytes = 2u * (((uint32_t)directWriteWidth + 7u) / 8u) * directWriteHeight;
}

// Powers/initializes the panel, opens the full address window, and (compressed)
// resets the zlib streamer. directWriteDecompressedTotal must already be set for
// compressed. Shared by 0x70 and 0x80. No header parsing, no inline data, no acks.
static void directWriteActivatePanel(void) {
    directWriteActive = true;
    directWriteBytesWritten = 0;
    directWriteStartTime = millis();
    imageWriteLogStart(directWriteTotalBytes);
    // Full-frame write: acquire the session with the FULL init sequence. A warm
    // re-acquire skips the ~900 ms rail bring-up + bbepInitIO (replaces the old
    // force power-cycle, which under keep-alive would fire on every push). A
    // full-frame direct write does not preserve partial plane consistency.
    epdSessionAcquire(false);
    epdPlanesPrepared = false;
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
    if (seeed_driver_used()) {
        seeed_gfx_direct_write_reset();
    } else
#endif
#ifdef BBEP_T133A01
    if (e1004_panel_used()) {
        if (!e1004_begin_plane()) {
            writeSerial("ERROR: E1004 dual-CS plane open failed", true);
        }
    } else
#endif
    {
        bbepSetAddrWindow(&bbep, 0, 0, globalConfig.displays[0].pixel_width, globalConfig.displays[0].pixel_height);
        bbepStartWrite(&bbep, directWriteBitplanes ? PLANE_0 : getplane());
    }
    if (directWriteCompressed) {
        od_zlib_stream_reset(directWriteDecompressedTotal);
    }
}

void handleDirectWriteStart(uint8_t* data, uint16_t len) {
    if (partialCtx.active) cleanup_partial_write_state();
    if (directWriteActive) {
        cleanupDirectWriteState(false);
    }
    resetPipeWriteState();
    imageWriteLogReset();
    touchSuspendForEpdRefresh();
    directWriteTouchSuspended = true;
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
    if (seeed_driver_used()) {
        seeed_gfx_prepare_hardware();
    }
#endif
    bool compressed = (len >= 4);
    directWriteComputeGeometry(compressed);
    if (compressed) {
        memcpy(&directWriteDecompressedTotal, data, 4);
        if (directWriteDecompressedTotal != directWriteTotalBytes) {
            cleanupDirectWriteState(false);
            uint8_t errorResponse[] = {RESP_NACK, RESP_DIRECT_WRITE_START_ACK};
            sendResponse(errorResponse, sizeof(errorResponse));
            return;
        }
    }
    directWriteActivatePanel();
    if (compressed && len > 4) {
        uint32_t compressedDataLen = len - 4;
        if (!zlib_stream_to_direct_write(data + 4, compressedDataLen, false)) {
            cleanupDirectWriteState(false);
            uint8_t errorResponse[] = {RESP_NACK, RESP_DIRECT_WRITE_START_ACK};
            sendResponse(errorResponse, sizeof(errorResponse));
            return;
        }
        directWriteCompressedReceived = compressedDataLen;
    }
    uint8_t ackResponse[] = {RESP_ACK, RESP_DIRECT_WRITE_START_ACK};
    sendResponse(ackResponse, sizeof(ackResponse));
}

void handlePartialWriteStart(uint8_t* data, uint16_t len) {
    if (directWriteActive) cleanupDirectWriteState(false);
    if (partialCtx.active) cleanup_partial_write_state();
    resetPipeWriteState();
    imageWriteLogReset();

    if (len < sizeof(struct PartialWriteStartHeader)) {
        send_direct_write_nack(0x76, OD_ERR_PARTIAL_STREAM, false);
        return;
    }

    // Layout is struct PartialWriteStartHeader (17 B). It is ALL big-endian, unlike
    // the little-endian PipePartialExt twin, so we parse by hand (the shifts ARE the
    // byte-swap) rather than overlaying the struct on this LE MCU.
    uint8_t flags     = data[0];
    uint32_t oldEtag  = parse_be_u32(data + 1);
    uint32_t newEtag  = parse_be_u32(data + 5);
    uint16_t rectX    = ((uint16_t)data[9]  << 8) | data[10];
    uint16_t rectY    = ((uint16_t)data[11] << 8) | data[12];
    uint16_t rectW    = ((uint16_t)data[13] << 8) | data[14];
    uint16_t rectH    = ((uint16_t)data[15] << 8) | data[16];

    if ((flags & ~PARTIAL_ALLOWED_FLAGS) != 0) {
        send_direct_write_nack(0x76, OD_ERR_PARTIAL_FLAGS, false);
        return;
    }

    if (oldEtag == 0 || oldEtag != displayed_etag || newEtag == 0) {
        send_direct_write_nack(0x76, OD_ERR_PARTIAL_ETAG_MISMATCH, false);
        return;
    }

    uint16_t dispW = globalConfig.displays[0].pixel_width;
    uint16_t dispH = globalConfig.displays[0].pixel_height;
    if (getBitsPerPixel() != 1 || e1004_panel_used()) {
        // bb_epaper partial refresh support is effectively non-existent for
        // 2bpp+ panels, and physical panels may not support that mode either.
        // This protocol uses two 1bpp controller planes as old/new image memory.
        send_direct_write_nack(0x76, OD_ERR_PARTIAL_UNSUPPORTED, false);
        return;
    }

    if (rectW == 0 || rectH == 0 ||
        (uint32_t)rectX + rectW > dispW ||
        (uint32_t)rectY + rectH > dispH) {
        send_direct_write_nack(0x76, OD_ERR_PARTIAL_RECT_OOB, false);
        return;
    }

    if ((rectX & 7u) != 0 || (rectW & 7u) != 0) {
        send_direct_write_nack(0x76, OD_ERR_PARTIAL_RECT_ALIGN, false);
        return;
    }

    uint32_t planeBytes = calc_controller_plane_bytes(rectW, rectH);
    uint32_t expectedLogicalSize = planeBytes * 2u;

    // TODO(protocol): 0x76 (partial-write) has no RESP_* mirror in the canonical
    // header, so the opcode-echo byte in these frames — and in the send_direct_write_nack(0x76, ...)
    // calls below — stays a raw literal. Add RESP_PARTIAL_WRITE_START upstream in
    // opendisplay-protocol, then replace the raw 0x76 here.
    if (expectedLogicalSize == 0) {
        uint8_t errResponse[] = {RESP_NACK, 0x76};
        sendResponse(errResponse, sizeof(errResponse));
        return;
    }

    memset(&partialCtx, 0, sizeof(partialCtx));
    partialCtx.active = true;
    partialCtx.compressed = (flags & PARTIAL_FLAG_COMPRESSED) != 0;
    partialCtx.flags = flags;
    partialCtx.new_etag = newEtag;
    partialCtx.x = rectX;
    partialCtx.y = rectY;
    partialCtx.width = rectW;
    partialCtx.height = rectH;
    partialCtx.expected_stream_size = expectedLogicalSize;
    partialCtx.plane_size = planeBytes;
    partialCtx.current_plane = 0xFF;
    partialCtx.start_time = millis();
    imageWriteLogStart(expectedLogicalSize);

    partial_prepare_panel_ram();
    if (partialCtx.compressed) od_zlib_stream_reset(expectedLogicalSize);

    // Process optional initial stream bytes before ACK
    if (len > 17) {
        uint16_t initLen = len - 17;
        if (!partial_consume_bytes(data + 17, (uint32_t)initLen)) {
            send_direct_write_nack(0x76, OD_ERR_PARTIAL_STREAM, true);
            return;
        }
    }

    uint8_t ackResponse[] = {RESP_ACK, 0x76};
    sendResponse(ackResponse, sizeof(ackResponse));
}

void handleDirectWriteData(uint8_t* data, uint16_t len) {
    // A pipe transfer (0x0080-0x0082) owns the panel session — and a pipe-partial
    // one owns partialCtx. A stray legacy 0x71 must not feed that context out of
    // band from the sliding-window seq accounting. Silent-discard mirrors how the
    // pipe path treats frames after a fatal error.
    if (pipeState.active) return;
    if (partialCtx.active) {
        if (len == 0) return;
        imageWriteLogChunk(data, len);
        if (!partial_consume_bytes(data, (uint32_t)len)) {
            send_direct_write_nack(RESP_DIRECT_WRITE_DATA_ACK, OD_ERR_PARTIAL_STREAM, true);
            return;
        }
        imageWriteLogProgress(partialCtx.bytes_received, partialCtx.expected_stream_size);
        uint8_t ackResponse[] = {RESP_ACK, RESP_DIRECT_WRITE_DATA_ACK};
        sendResponse(ackResponse, sizeof(ackResponse));
        return;
    }
    if (!directWriteActive || len == 0) return;
    imageWriteLogChunk(data, len);
    if (directWriteCompressed) {
        if (!handleDirectWriteCompressedData(data, len)) {
            cleanupDirectWriteState(true);
            uint8_t errorResponse[] = {RESP_NACK, RESP_DIRECT_WRITE_DATA_ACK};
            sendResponse(errorResponse, sizeof(errorResponse));
        } else {
            directWriteCompressedReceived += len;
            imageWriteLogProgress(directWriteBytesWritten, directWriteTotalBytes);
            uint8_t ackResponse[] = {RESP_ACK, RESP_DIRECT_WRITE_DATA_ACK};
            sendResponse(ackResponse, sizeof(ackResponse));
        }
        return;
    }
    uint32_t remainingBytes = (directWriteBytesWritten < directWriteTotalBytes) ? (directWriteTotalBytes - directWriteBytesWritten) : 0;
    uint16_t bytesToWrite = (len > remainingBytes) ? remainingBytes : len;
    if (bytesToWrite > 0) {
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
        if (seeed_driver_used()) {
            seeed_gfx_direct_write_chunk(data, bytesToWrite);
            directWriteBytesWritten += bytesToWrite;
        } else
#endif
        if (directWriteIsGray4() || directWriteBitplanes) {
            streamGray4Bytes(data, bytesToWrite);  // advances directWriteBytesWritten, splits planes
        } else {
            directWriteSinkBytes(data, bytesToWrite);
        }
    }
    imageWriteLogProgress(directWriteBytesWritten, directWriteTotalBytes);
    if (directWriteBytesWritten >= directWriteTotalBytes) {
        handleDirectWriteEnd(nullptr, 0);
    } else {
        uint8_t ackResponse[] = {RESP_ACK, RESP_DIRECT_WRITE_DATA_ACK};
        sendResponse(ackResponse, sizeof(ackResponse));
    }
}

void handleDirectWriteEnd(uint8_t* data, uint16_t len) {
    // Same guard as handleDirectWriteData: a stray legacy 0x72 mid-pipe must not
    // finalize/refresh a pipe-owned session (partial would commit new_etag==0 and
    // leave pipeState zombied; full-frame would refresh with pipeState still active).
    if (pipeState.active) return;
    if (partialCtx.active) {
        if (data != nullptr && len > 1) {
            send_direct_write_nack(RESP_DIRECT_WRITE_END_ACK, OD_ERR_PARTIAL_STREAM, true);
            return;
        }
        if (partialCtx.compressed) {
            if (partialCtx.bytes_received == 0 || !zlib_stream_to_partial_write(nullptr, 0, true)) {
                send_direct_write_nack(RESP_DIRECT_WRITE_END_ACK, OD_ERR_PARTIAL_STREAM, true);
                return;
            }
        } else if (partialCtx.bytes_written != partialCtx.expected_stream_size) {
            send_direct_write_nack(RESP_DIRECT_WRITE_END_ACK, OD_ERR_PARTIAL_STREAM, true);
            return;
        }
        imageWriteLogFinish(partialCtx.bytes_received, partialCtx.expected_stream_size);
        uint8_t ackResponse[] = {RESP_ACK, RESP_DIRECT_WRITE_END_ACK};
        sendResponse(ackResponse, sizeof(ackResponse));
        int refreshMode = REFRESH_PARTIAL;
        if (data != nullptr && len >= 1 && data[0] == REFRESH_FULL) refreshMode = REFRESH_FULL;
        else if (data != nullptr && len >= 1 && data[0] == REFRESH_FAST) refreshMode = REFRESH_FAST;
        bool refreshSuccess = partial_write_to_panel(refreshMode);
        if (refreshSuccess) {
            displayed_etag = partialCtx.new_etag;
            uint8_t validatedResponse[] = {RESP_ACK, RESP_DIRECT_WRITE_REFRESH_SUCCESS};
            sendResponse(validatedResponse, sizeof(validatedResponse));
        } else {
            displayed_etag = 0;
            uint8_t timeoutResponse[] = {RESP_ACK, RESP_DIRECT_WRITE_REFRESH_TIMEOUT};
            sendResponse(timeoutResponse, sizeof(timeoutResponse));
        }
        cleanup_partial_write_state();
        return;
    }
    if (!directWriteActive) return;
    directWriteFinishAndRefresh(data, len, 0x72);
}

// Shared finalize+refresh tail for a full-frame direct-write session. Emits the
// END success ack {0x00,endOpcode} (0x72 legacy / 0x82 PIPE) or a NACK
// {0xFF,endOpcode} on compressed-flush/completeness failure, then refreshes the
// panel and emits {0x00,0x73}/{0x00,0x74}. Caller guarantees directWriteActive.
static void directWriteFinishAndRefresh(uint8_t* data, uint16_t len, uint8_t endOpcode) {
    directWriteStartTime = 0;
    if (directWriteCompressed && !zlib_stream_to_direct_write(nullptr, 0, true)) {
        cleanupDirectWriteState(true);
        uint8_t errorResponse[] = {0xFF, endOpcode};
        sendResponse(errorResponse, sizeof(errorResponse));
        return;
    }
    const bool gray4 = directWriteIsGray4();
    if (gray4 || directWriteBitplanes) {
        // Both planes must be present before refresh. Compressed and uncompressed
        // paths stream live as chunks, so confirm the full two-plane payload
        // arrived before refreshing stale RAM or committing an etag.
        if (directWriteBytesWritten != directWriteTotalBytes) {
            cleanupDirectWriteState(false);
            uint8_t errorResponse[] = {0xFF, endOpcode};
            sendResponse(errorResponse, sizeof(errorResponse));
            return;
        }
    }
    imageWriteLogFinish(directWriteBytesWritten, directWriteTotalBytes);
    int refreshMode = REFRESH_FULL;
    if (data != nullptr && len >= 1 && data[0] == 1) refreshMode = REFRESH_FAST;
    if (e1004_panel_used()) refreshMode = REFRESH_FULL;  // fast re-init would wipe RAM
    writeSerial("EPD refresh: ", false);
    writeSerial(refreshMode == REFRESH_FAST ? "FAST" : "FULL", false);
    writeSerial(" (mode=", false);
    writeSerial(String(refreshMode), false);
    writeSerial(", end payload ", false);
    if (data != nullptr && len > 0) {
        writeSerial("0x", false);
        writeSerial(String(data[0], HEX), false);
    } else {
        writeSerial("none (auto)", false);
    }
    writeSerial(")", true);
    uint8_t ackResponse[] = {0x00, endOpcode};
    sendResponse(ackResponse, sizeof(ackResponse));
    delay(20);
    epdRefreshInProgress = true;
    bool refreshSuccess = false;
    uint32_t newEtag = 0;
    bool hasNewEtag = data != nullptr && len >= 5;
    if (hasNewEtag) newEtag = parse_be_u32(data + 1);
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
    if (seeed_driver_used()) {
        seeed_gfx_direct_refresh(refreshMode);
        refreshSuccess = waitforrefresh(60);
        seeed_gfx_direct_sleep();
    } else
#endif
    {
#ifdef BBEP_T133A01
        if (e1004_panel_used()) e1004_end_plane();
#endif
        bbepRefresh(&bbep, refreshMode);
        refreshSuccess = waitforrefresh(60);
        // No bbepSleep here: cleanupDirectWriteState(false) releases the session,
        // keeping the controller awake + rail up when keep-alive holds it warm.
    }
    epdRefreshInProgress = false;
    cleanupDirectWriteState(false);
#ifdef TARGET_ESP32
    esp32_restart_ble_advertising();
#endif
    if (refreshSuccess) {
        // A successful refresh changed the panel image. Commit the new etag
        // when the client supplied a valid one; otherwise clear the stale etag
        // (etag-less full upload / auto-complete) so a later partial update
        // gets a clean ETAG mismatch and falls back to a full upload instead
        // of diffing against the wrong, now-outdated base image.
        if (hasNewEtag && newEtag != 0) displayed_etag = newEtag;
        else displayed_etag = 0;
        uint8_t refreshResponse[] = {RESP_ACK, RESP_DIRECT_WRITE_REFRESH_SUCCESS};
        sendResponse(refreshResponse, sizeof(refreshResponse));
    } else {
        if (hasNewEtag) displayed_etag = 0;
        uint8_t timeoutResponse[] = {RESP_ACK, RESP_DIRECT_WRITE_REFRESH_TIMEOUT};
        sendResponse(timeoutResponse, sizeof(timeoutResponse));
    }
}

// ===========================================================================
// PIPE_WRITE (0x0080-0x0082): sliding-window image transfer with QUIC-style SACK.
// Reuses the direct-write session machinery (directWriteComputeGeometry /
// directWriteActivatePanel / pipeConsumePayload -> bbepWriteData / zlib) so the
// legacy 0x70/0x71/0x72 path is untouched. Out-of-order frames are held in a
// 33-slot reorder queue while the controller stream pauses at a hole.
//
// PARTIAL-REGION mode (PIPE_FLAG_PARTIAL, flags bit1): a single-rectangle partial
// update rides the same sliding window. The 0x0080 START gains a 12-byte LE
// extension appended after total_size (payload len 22 instead of 10):
//   [old_etag:4 LE][x:2][y:2][w:2][h:2]   (LE, unlike 0x76's big-endian layout).
// total_size is the decompressed logical stream size = plane_size*2 where
// plane_size = calc_controller_plane_bytes(w,h) (old plane then new plane, the
// same stream 0x76 uses). Geometry/etag are validated exactly like the 0x76
// handler; the ACK sets response flags bit1 to confirm partial acceptance. DATA
// (0x0081) is routed to partialCtx (two 1bpp controller planes) instead of the
// full-frame writer, and partial transfers NEVER auto-complete — the explicit
// 0x0082 END carries the refresh selector (0->FULL,1->FAST,2/absent->PARTIAL) and
// new_etag [refresh:1][new_etag:4 BE], driving partial_write_to_panel().
// START NACK codes: 0x01 len/ver, 0x02 unknown flag, 0x03 size mismatch,
// 0x05 ETAG_MISMATCH (partial), 0x06 PARTIAL_UNSUPPORTED (partial, bpp!=1/seeed),
// 0x07 RECT_INVALID (partial, zero/OOB/misaligned rect). Any partial START NACK
// at the geometry/etag stages clears displayed_etag (parity with 0x76).
// ===========================================================================

static inline uint8_t pipeSlot(uint8_t seq) { return (uint8_t)(seq % PIPE_REORDER_SLOTS); }

void resetPipeWriteState(void) {
    pipeState = PipeWriteState{};
    for (int i = 0; i < PIPE_REORDER_SLOTS; ++i) pipeReorder[i].occupied = false;
}

bool pipeWriteActive(void) { return pipeState.active; }

// A chunk c is "received" for ACK purposes if it was accepted in-order (lies just
// below expected_seq within the mask window) or is currently held in the reorder
// queue. The accepted-prefix depth is bounded by received_count (chunks actually
// streamed this transfer, i.e. expected_seq advances): a plain mod-256 distance
// test would wrap during the first 32 chunks and assert phantom "received" bits
// for seqs 224-255 that predate the transfer (e.g. expected_seq=8 claiming seq
// 250). Never marks an in-range unreceived chunk; highest_seen=0 with only chunk
// 0 accepted yields mask=0.
static bool pipeChunkReceived(uint8_t c) {
    uint8_t below = (uint8_t)(pipeState.expected_seq - 1 - c);   // distance below expected
    uint32_t acceptedDepth = (pipeState.received_count < PIPE_ACK_MASK_BITS)
                           ? pipeState.received_count : PIPE_ACK_MASK_BITS;
    if (below < acceptedDepth) return true;                      // accepted (in-order prefix)
    return pipeReorder[pipeSlot(c)].occupied && pipeReorder[pipeSlot(c)].seq == c;
}

// Fills out[0]=highest_seen, out[1..4]=32-bit ack_mask LE. Mask bit i (LSB first)
// = chunk (highest_seen - 1 - i) received. highest_seen implicitly acked.
static void pipeBuildAckPayload(uint8_t* out) {
    uint8_t hs = pipeState.has_received ? pipeState.highest_seen
                                        : (uint8_t)(pipeState.expected_seq - 1);
    uint32_t mask = 0;
    for (uint8_t i = 0; i < PIPE_ACK_MASK_BITS; ++i) {
        if (pipeChunkReceived((uint8_t)(hs - 1 - i))) mask |= (1u << i);
    }
    out[0] = hs;
    out[1] = (uint8_t)(mask & 0xFF);
    out[2] = (uint8_t)((mask >> 8) & 0xFF);
    out[3] = (uint8_t)((mask >> 16) & 0xFF);
    out[4] = (uint8_t)((mask >> 24) & 0xFF);
}

// TODO(protocol): the canonical opendisplay_protocol.h defines no RESP_* mirror
// for the pipe-write opcodes, so the response opcode-echo byte in the helpers
// below (0x80 / 0x81 / 0x82) stays a raw literal. It also defines no data-phase
// pipe error namespace (only OD_ERR_PIPE_START_* for the 0x80 START), so the
// sendPipeNack() error codes (0x03 over-size/overflow, 0x04 out-of-window) are
// raw too. Add RESP_PIPE_WRITE_{START,DATA,END} + an OD_ERR_PIPE_DATA_* set
// upstream in opendisplay-protocol, then replace those literals here.

// {0x00,0x81, highest_seen, ack_mask LE(4)} via sendResponse (auto-encrypts when
// authenticated). Resets both cadence counters.
static void sendPipeAck(void) {
    uint8_t r[7] = {RESP_ACK, 0x81, 0, 0, 0, 0, 0};
    pipeBuildAckPayload(r + 2);
    sendResponse(r, sizeof(r));
    pipeState.frames_since_ack = 0;
    pipeState.ooo_acks_since_gap = 0;
}

// {0xFF,0x81, err, highest_seen, ack_mask LE(4)}. All 0x81 NACKs are FATAL: the
// payload is built from pipeState + reorder queue BEFORE any teardown, the error
// flag makes subsequent 0x0081 frames silently discard until the next 0x0080 /
// disconnect (pipeState and the reorder queue are deliberately NOT reset so the
// reported ACK position stays consistent), and the panel hardware is released the
// same way the legacy mid-stream {0xFF,0x71} failure does (cleanupDirectWriteState
// with refreshDisplay=true: sleep a powered controller cleanly, cut power, resume
// touch) instead of leaving the panel powered until the next transfer.
static void sendPipeNack(uint8_t err) {
    uint8_t r[8] = {RESP_NACK, 0x81, err, 0, 0, 0, 0, 0};
    pipeBuildAckPayload(r + 3);
    sendResponse(r, sizeof(r));
    pipeState.error = true;
    // Partial transfers own partialCtx, not the full-frame direct-write session:
    // clear the negotiated etag (any partial NACK invalidates it, parity with
    // send_direct_write_nack) and power the panel down via the partial cleanup.
    if (pipeState.partial) {
        displayed_etag = 0;
        cleanup_partial_write_state();
    } else {
        cleanupDirectWriteState(true);
    }
}

// {0xFF,0x80, err, 0x00}. Caller owns any session teardown.
static void sendPipeStartNack(uint8_t err) {
    uint8_t r[4] = {RESP_NACK, 0x80, err, 0x00};
    sendResponse(r, sizeof(r));
}

// Advance highest_seen only for a genuinely newer seq (forward distance small and
// nonzero) so duplicates/late frames never move it backward.
static void pipeUpdateHighestSeen(uint8_t seq) {
    if (!pipeState.has_received) {
        pipeState.has_received = true;
        pipeState.highest_seen = seq;
        return;
    }
    uint8_t fwd = (uint8_t)(seq - pipeState.highest_seen);
    if (fwd != 0 && fwd <= PIPE_ACK_MASK_BITS) pipeState.highest_seen = seq;
}

// Feed one DATA payload to the panel controller through the SAME machinery the
// legacy 0x71 path uses. Returns false on any write/decompress/overflow failure.
static bool pipeConsumePayload(uint8_t* data, uint16_t len) {
    if (len == 0) return true;
    imageWriteLogChunk(data, len);
    // Partial transfers stream into the two 1bpp controller planes via partialCtx;
    // partial_consume_bytes handles zlib-vs-raw and plane-at-a-time sub-window
    // addressing itself, so the full-frame directWrite* writers below are skipped.
    if (pipeState.partial) {
        if (!partial_consume_bytes(data, (uint32_t)len)) return false;
        imageWriteLogProgress(partialCtx.bytes_received, partialCtx.expected_stream_size);
        return true;
    }
    if (directWriteCompressed) {
        if (!handleDirectWriteCompressedData(data, len)) return false;
        directWriteCompressedReceived += len;
        imageWriteLogProgress(directWriteBytesWritten, directWriteTotalBytes);
        return true;
    }
    uint32_t remaining = (directWriteBytesWritten < directWriteTotalBytes)
                       ? (directWriteTotalBytes - directWriteBytesWritten) : 0;
    uint16_t toWrite = (len > remaining) ? (uint16_t)remaining : len;
    if (toWrite > 0) {
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
        if (seeed_driver_used()) {
            seeed_gfx_direct_write_chunk(data, toWrite);
            directWriteBytesWritten += toWrite;
        } else
#endif
        if (directWriteIsGray4() || directWriteBitplanes) {
            streamGray4Bytes(data, toWrite);  // advances directWriteBytesWritten, splits planes
        } else {
            directWriteSinkBytes(data, toWrite);
        }
    }
    imageWriteLogProgress(directWriteBytesWritten, directWriteTotalBytes);
    return true;
}

void handlePipeWriteStart(uint8_t* data, uint16_t len) {
    // A new START aborts any in-flight transfer of any family and resets pipe state
    // (mirrors legacy START). Reset happens up-front so even a malformed START is safe.
    if (partialCtx.active) cleanup_partial_write_state();
    if (directWriteActive) cleanupDirectWriteState(false);
    resetPipeWriteState();

    // Fixed 10-byte payload (opcode already stripped by the dispatcher):
    // ver(1)+flags(1)+req_w(1)+req_n(1)+client_max_frame(2)+total_size(4).
    // Tolerate trailing bytes (future fields).
    if (len < sizeof(struct PipeStartRequest)) { sendPipeStartNack(OD_ERR_PIPE_START_BAD_HEADER); return; }
    struct PipeStartRequest req;
    memcpy(&req, data, sizeof req);   // canonical 10-byte LE header (byte-identical to the old shifts)
    uint8_t  ver              = req.version;
    uint8_t  flags            = req.flags;
    uint8_t  req_w            = req.req_window;
    uint8_t  req_n            = req.req_ack_every;
    uint16_t client_max_frame = req.client_max_frame;
    uint32_t total_size       = req.total_size;

    if (ver != PIPE_VERSION) { sendPipeStartNack(OD_ERR_PIPE_START_BAD_HEADER); return; }
    // Defined flags: bit0 zlib compression, bit1 partial-region refresh. Any other bit unsupported.
    if ((flags & ~(PIPE_FLAG_COMPRESSED | PIPE_FLAG_PARTIAL)) != 0) { sendPipeStartNack(OD_ERR_PIPE_START_UNKNOWN_FLAG); return; }

    bool compressed = (flags & PIPE_FLAG_COMPRESSED) != 0;
    bool partial    = (flags & PIPE_FLAG_PARTIAL) != 0;

    // Partial START appends a 12-byte LE extension after total_size (payload len 22 vs 10):
    // [old_etag:4][x:2][y:2][w:2][h:2]. LE, unlike 0x76's big-endian layout.
    if (partial && len < sizeof(struct PipeStartRequest) + sizeof(struct PipePartialExt)) {
        sendPipeStartNack(OD_ERR_PIPE_START_BAD_HEADER); return;
    }
    uint32_t old_etag = 0;
    uint16_t rectX = 0, rectY = 0, rectW = 0, rectH = 0;
    uint32_t planeBytes = 0;
    if (partial) {
        struct PipePartialExt ext;
        memcpy(&ext, data + sizeof(struct PipeStartRequest), sizeof ext);  // 12-byte LE extension
        old_etag = ext.old_etag;
        rectX = ext.x;
        rectY = ext.y;
        rectW = ext.w;
        rectH = ext.h;

        // Partial validations (plan 1.2, order 5-7). All precede any hardware touch; any
        // failure clears displayed_etag for parity with send_direct_write_nack. These are
        // the same checks the 0x76 handler runs (bpp, etag, bounds, alignment).
        uint16_t dispW = globalConfig.displays[0].pixel_width;
        uint16_t dispH = globalConfig.displays[0].pixel_height;
        // 5: two 1bpp controller planes are the partial mechanism; seeed/IT8951 has no equivalent.
        if (getBitsPerPixel() != 1 || seeed_driver_used() || e1004_panel_used()) {
            displayed_etag = 0; sendPipeStartNack(OD_ERR_PIPE_START_PARTIAL_UNSUPPORTED); return;
        }
        // 6: etag gate — nonzero and must match what is currently on the panel.
        if (old_etag == 0 || old_etag != displayed_etag) {
            displayed_etag = 0; sendPipeStartNack(OD_ERR_PIPE_START_ETAG_MISMATCH); return;
        }
        // 7: rectangle must be non-empty, in-bounds, and x/width byte-aligned (1bpp packing).
        if (rectW == 0 || rectH == 0 ||
            (uint32_t)rectX + rectW > dispW || (uint32_t)rectY + rectH > dispH ||
            (rectX & 7u) != 0 || (rectW & 7u) != 0) {
            displayed_etag = 0; sendPipeStartNack(OD_ERR_PIPE_START_RECT_INVALID); return;
        }
    }

    // total_size validation (plan 1.2, order 8). Pure config/geometry math (no panel I/O),
    // so a NACK here needs no teardown. Partial: plane_size*2 (flat old+new planes, like
    // 0x76). Full: directWriteComputeGeometry's decompressed panel byte total.
    if (partial) {
        planeBytes = calc_controller_plane_bytes(rectW, rectH);
        if (planeBytes == 0 || total_size != planeBytes * 2u) {
            // Plan 1.2: every partial-request NACK at steps 5-8 clears the etag
            // (parity with send_direct_write_nack).
            displayed_etag = 0; sendPipeStartNack(OD_ERR_PIPE_START_SIZE_MISMATCH); return;
        }
    } else {
        directWriteComputeGeometry(compressed);
        if (total_size != directWriteTotalBytes) { sendPipeStartNack(OD_ERR_PIPE_START_SIZE_MISMATCH); return; }
    }

    // Effective values (min-rule, plan 1.1). Floors at 1; N <= W; frame <= 244.
    uint8_t w_eff = req_w > PIPE_MAX_W ? PIPE_MAX_W : req_w;
    if (w_eff == 0) w_eff = 1;
    if (isEncryptionEnabled() && isAuthenticated() && w_eff > 32) w_eff = 32;  // defensive (mask width)
    uint8_t n_eff = req_n > PIPE_MAX_N ? PIPE_MAX_N : req_n;
    if (n_eff == 0) n_eff = 1;
    if (n_eff > w_eff) n_eff = w_eff;
    uint16_t frame_eff = client_max_frame < PIPE_MAX_FRAME ? client_max_frame : PIPE_MAX_FRAME;

    pipeState.active = true;
    pipeState.error = false;
    pipeState.compressed = compressed;
    pipeState.gap_open = false;
    pipeState.window = w_eff;
    pipeState.ack_every = n_eff;
    pipeState.max_frame = frame_eff;
    pipeState.expected_seq = 0;
    pipeState.has_received = false;
    pipeState.highest_seen = 0;
    pipeState.received_count = 0;
    pipeState.frames_since_ack = 0;
    pipeState.ooo_acks_since_gap = 0;
    pipeState.total_size = total_size;
    pipeState.queued_count = 0;
    pipeState.queue_high_water = 0;
    pipeState.partial = partial;

    // Partial transfers own partialCtx (two 1bpp planes); init it exactly as the 0x76
    // START does, but new_etag stays 0 — it rides the 0x0082 END. Bookkeeping only; the
    // panel RAM prep (partial_prepare_panel_ram) waits until after the ACK, below.
    if (partial) {
        memset(&partialCtx, 0, sizeof(partialCtx));
        partialCtx.active = true;
        partialCtx.compressed = compressed;
        partialCtx.flags = flags;
        partialCtx.new_etag = 0;
        partialCtx.x = rectX;
        partialCtx.y = rectY;
        partialCtx.width = rectW;
        partialCtx.height = rectH;
        partialCtx.expected_stream_size = total_size;
        partialCtx.plane_size = planeBytes;
        partialCtx.current_plane = 0xFF;
        partialCtx.start_time = millis();
    }

    // Respond BEFORE panel bring-up: slow panels (Spectra/ACeP-class init can take
    // seconds) must not starve the client's 0x0080 START wait. Clients gate pipe
    // attempts on the config pipe bit and wait a normal command timeout (30 s,
    // sized for the ESP32 response-queue flush landing after bring-up), but
    // responding first keeps the wait short on nRF and any direct-notify path.
    // Ordering is safe on both targets: on ESP32 this handler runs in the main-loop
    // queue drain, so 0x0081 frames arriving during bring-up park in the 33-slot
    // ingest ring until we return; on nRF the Bluefruit write callback dispatches
    // commands sequentially from its callback task, so activation below completes
    // before any queued 0x0081 write is handed to the dispatcher.
    // Device maxima; flags bit0 SET (selective repeat), bit1 = partial accepted (plan 1.2).
    // TODO(protocol): 0x80 has no RESP_* mirror (see sendPipeAck), and the START
    // response-flags bit0 "selective repeat" has no canonical constant (distinct from
    // the request-side PIPE_FLAG_*). Add RESP_PIPE_WRITE_START + a response-flag
    // constant upstream, then replace the raw 0x80 / 0x01 here.
    uint8_t resp[8] = {RESP_ACK, 0x80, PIPE_VERSION, PIPE_MAX_W, PIPE_MAX_N,
                       (uint8_t)(PIPE_MAX_FRAME & 0xFF), (uint8_t)((PIPE_MAX_FRAME >> 8) & 0xFF),
                       (uint8_t)(0x01 | (partial ? PIPE_FLAG_PARTIAL : 0))};
    sendResponse(resp, sizeof(resp));

    if (partial) {
        // Partial bring-up (0x76 parity): white-fill both controller planes and reset the
        // zlib stream if compressed. Deliberately NOT the full-frame path — do not suspend
        // touch, set directWriteActive, or call directWriteActivatePanel (partial_write_to_panel
        // powers the panel down on the 0x0082 END; partial_prepare_panel_ram brought it up).
        imageWriteLogReset();
        imageWriteLogStart(total_size);
        partial_prepare_panel_ram();
        if (compressed) od_zlib_stream_reset(total_size);
        return;
    }

    // Bring up the full-frame session exactly like legacy START (touch suspend, panel prep).
    imageWriteLogReset();
    touchSuspendForEpdRefresh();
    directWriteTouchSuspended = true;
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
    if (seeed_driver_used()) {
        seeed_gfx_prepare_hardware();
    }
#endif
    directWriteDecompressedTotal = total_size;   // compressed zlib reset + overflow guard
    directWriteActivatePanel();
}

void handlePipeWriteData(uint8_t* data, uint16_t len) {
    if (!pipeState.active || pipeState.error) return;   // silent discard
    if (len < 1) return;
    uint8_t  seq     = data[0];
    uint8_t* payload = data + 1;
    uint16_t plen    = (uint16_t)(len - 1);
    if (plen > PIPE_REORDER_SLOT_SIZE) { sendPipeNack(0x03); return; }  // over-size frame (impossible <=244)

    const uint8_t W   = pipeState.window;
    uint8_t fwd  = (uint8_t)(seq - pipeState.expected_seq);   // 0 in-order; 1..W-1 ahead
    uint8_t back = (uint8_t)(pipeState.expected_seq - seq);   // >=1 below expected

    if (fwd == 0) {
        // In-order accept -> stream to controller, then drain contiguous successors.
        // Count every accepted frame (trigger + drained) toward the ACK cadence so a
        // post-gap drain refunds tokens promptly instead of waiting for fresh frames.
        if (!pipeConsumePayload(payload, plen)) { sendPipeNack(pipeState.compressed ? 0x02 : 0x03); return; }
        pipeState.expected_seq++;
        pipeState.received_count++;
        pipeState.frames_since_ack++;
        pipeUpdateHighestSeen(seq);
        while (pipeReorder[pipeSlot(pipeState.expected_seq)].occupied &&
               pipeReorder[pipeSlot(pipeState.expected_seq)].seq == pipeState.expected_seq) {
            PipeReorderSlot& s = pipeReorder[pipeSlot(pipeState.expected_seq)];
            if (!pipeConsumePayload(s.data, s.len)) { sendPipeNack(pipeState.compressed ? 0x02 : 0x03); return; }
            s.occupied = false;
            if (pipeState.queued_count > 0) pipeState.queued_count--;
            pipeState.expected_seq++;
            pipeState.received_count++;
            if (pipeState.frames_since_ack < 0xFF) pipeState.frames_since_ack++;
        }
        if (pipeState.queued_count == 0) pipeState.gap_open = false;
        // Auto-complete (uncompressed FULL-FRAME only, mirrors legacy handleDirectWriteData
        // auto-finish). The shared helper emits the single unsolicited {0x00,0x82} END ack then
        // refreshes with a FULL waveform. MUST be gated on !partial: a partial transfer never
        // touches directWrite* (both are 0), so 0>=0 would false-fire a FULL refresh on the very
        // first frame — partial transfers complete only on the explicit 0x0082 END (plan 1.5).
        if (!pipeState.partial && !pipeState.compressed && directWriteBytesWritten >= directWriteTotalBytes) {
            sendPipeAck();                                   // final tail flush ({0x00,0x81})
            directWriteFinishAndRefresh(nullptr, 0, 0x82);   // {0x00,0x82} + FULL refresh, no etag
            resetPipeWriteState();
            return;
        }
        if (pipeState.frames_since_ack >= pipeState.ack_every) sendPipeAck();
        return;
    }

    if (fwd < W) {
        // Ahead within the window -> PAUSE POINT: hold in the reorder queue, nothing
        // reaches the controller past the hole.
        PipeReorderSlot& s = pipeReorder[pipeSlot(seq)];
        bool duplicate = (s.occupied && s.seq == seq);
        s.occupied = true;
        s.seq = seq;
        s.len = plen;
        memcpy(s.data, payload, plen);
        if (!duplicate) {
            pipeState.queued_count++;
            if (pipeState.queued_count > pipeState.queue_high_water)
                pipeState.queue_high_water = pipeState.queued_count;
        }
        if (pipeState.queued_count >= PIPE_REORDER_SLOTS) { sendPipeNack(0x03); return; }  // overflow guard
        pipeUpdateHighestSeen(seq);
        // Gap ACK: immediately when the gap first opens (fast-retransmit), then
        // rate-limited to one per ack_every subsequent out-of-order arrivals.
        if (!pipeState.gap_open) {
            pipeState.gap_open = true;
            sendPipeAck();                                   // resets ooo_acks_since_gap to 0
        } else if (++pipeState.ooo_acks_since_gap >= pipeState.ack_every) {
            sendPipeAck();
        }
        return;
    }

    // fwd >= W: either a duplicate of an already-accepted chunk (seq just below
    // expected) or a genuinely out-of-window frame.
    if (back <= W) {
        // Duplicate/below-expected -> discard, ACK so the sender learns our position
        // (rate-limited the same way as out-of-order arrivals).
        if (!pipeState.gap_open) {
            sendPipeAck();
        } else if (++pipeState.ooo_acks_since_gap >= pipeState.ack_every) {
            sendPipeAck();
        }
        return;
    }

    // Out of window on both sides -> protocol violation.
    sendPipeNack(0x04);
}

void handlePipeWriteEnd(uint8_t* data, uint16_t len) {
    if (!pipeState.active) {
        uint8_t n[2] = {RESP_NACK, 0x82};   // no active pipe transfer
        sendResponse(n, sizeof(n));
        return;
    }
    if (pipeState.error) {
        uint8_t n[2] = {RESP_NACK, 0x82};   // a fatal error already NACKed this transfer
        sendResponse(n, sizeof(n));
        // sendPipeNack already released the panel hardware at NACK time; this
        // re-run is a defensive no-op (partialCtx / directWriteActive already down).
        if (pipeState.partial) cleanup_partial_write_state();
        else cleanupDirectWriteState(false);
        resetPipeWriteState();
        return;
    }
    // Tail-flush ACK precedes the END result (plan 1.3c / 1.5).
    sendPipeAck();

    // Partial transfers never auto-complete (plan 1.5): the 0x0082 END alone carries the
    // refresh mode + new_etag. Completeness mirrors the 0x76 partial branch.
    if (pipeState.partial) {
        bool incomplete = (pipeState.queued_count > 0);
        if (partialCtx.compressed) {
            if (partialCtx.bytes_received == 0 || !zlib_stream_to_partial_write(nullptr, 0, true)) incomplete = true;
        } else if (partialCtx.bytes_written != partialCtx.expected_stream_size) {
            incomplete = true;
        }
        if (incomplete) {
            uint8_t n[2] = {RESP_NACK, 0x82};
            sendResponse(n, sizeof(n));
            displayed_etag = 0;
            cleanup_partial_write_state();
            resetPipeWriteState();
            return;
        }
        imageWriteLogFinish(partialCtx.bytes_received, partialCtx.expected_stream_size);
        uint8_t ackResponse[] = {RESP_ACK, 0x82};
        sendResponse(ackResponse, sizeof(ackResponse));
        // Refresh selector rides the END tail (plan 1.4): 0->FULL, 1->FAST, 2/absent->PARTIAL.
        int refreshMode = REFRESH_PARTIAL;
        if (data != nullptr && len >= 1 && data[0] == REFRESH_FULL) refreshMode = REFRESH_FULL;
        else if (data != nullptr && len >= 1 && data[0] == REFRESH_FAST) refreshMode = REFRESH_FAST;
        // new_etag rides the END tail [refresh:1][new_etag:4 BE]; absent => 0.
        uint32_t newEtag = (len >= 5) ? parse_be_u32(data + 1) : 0;
        bool refreshSuccess = partial_write_to_panel(refreshMode);
        if (refreshSuccess) {
            displayed_etag = newEtag;
            uint8_t validatedResponse[] = {RESP_ACK, RESP_DIRECT_WRITE_REFRESH_SUCCESS};
            sendResponse(validatedResponse, sizeof(validatedResponse));
        } else {
            displayed_etag = 0;
            uint8_t timeoutResponse[] = {RESP_ACK, RESP_DIRECT_WRITE_REFRESH_TIMEOUT};
            sendResponse(timeoutResponse, sizeof(timeoutResponse));
        }
        cleanup_partial_write_state();
        resetPipeWriteState();
        return;
    }

    // The client must not send END before every chunk is acked; a hole or short
    // byte count here is a protocol violation.
    bool incomplete = (pipeState.queued_count > 0);
    if (!pipeState.compressed && directWriteBytesWritten < directWriteTotalBytes) incomplete = true;
    if (incomplete) {
        uint8_t n[2] = {RESP_NACK, 0x82};
        sendResponse(n, sizeof(n));
        // Mid-stream abort with the panel powered: use the legacy mid-stream
        // variant (refreshDisplay=true → sleep the controller cleanly, cut power,
        // resume touch), matching the legacy {0xFF,0x71}/zlib-flush failure paths.
        cleanupDirectWriteState(true);
        resetPipeWriteState();
        return;
    }
    // Shared END/refresh flow emits {0x00,0x82} then {0x00,0x73}/{0x00,0x74}.
    // Compressed incompleteness surfaces as a zlib-flush NACK {0xFF,0x82} inside.
    directWriteFinishAndRefresh(data, len, 0x82);
    resetPipeWriteState();
}

static void cleanup_partial_write_state(void) {
    // Tear the panel down only when a transfer/refresh is actually in flight
    // (PWR_ACTIVE) — i.e. on error / NACK / disconnect-mid-stream / watchdog. After
    // a successful refresh, epdSessionRelease already moved to PWR_WARM, so
    // post-success cleanups become bookkeeping-only and leave keep-alive running.
    bool teardown = partialCtx.active && pwrmgmState == PWR_ACTIVE;
    memset(&partialCtx, 0, sizeof(partialCtx));
    if (teardown) epdSessionForceOff();
}

static bool panel_skips_bbep_set_addr_window(void) {
    return bbep.type == EP397_800x480 || bbep.type == EP397_800x480_4GRAY ||
           bbep.type == EP426_800x480 || bbep.type == EP426_800x480_4GRAY;
}

static bool panel_uses_pixel_ram_x(BBEPDISP *pBBEP) {
    return pBBEP->type == EP397_800x480 || pBBEP->type == EP397_800x480_4GRAY ||
           pBBEP->type == EP426_800x480 || pBBEP->type == EP426_800x480_4GRAY;
}

static bool panel_uses_ep397_y_decrement(BBEPDISP *pBBEP) {
    return pBBEP->type == EP397_800x480 || pBBEP->type == EP397_800x480_4GRAY;
}

static bool panel_uses_ep426_x_decrement(BBEPDISP *pBBEP) {
    return pBBEP->type == EP426_800x480 || pBBEP->type == EP426_800x480_4GRAY;
}

static bool panel_skips_reinit_on_partial_refresh(BBEPDISP *pBBEP) {
    return panel_uses_ep397_y_decrement(pBBEP) || panel_uses_ep426_x_decrement(pBBEP);
}

static void partial_set_ep397_ram_y(BBEPDISP *pBBEP, int ty, int cy) {
    uint8_t uc[4];
    int yLast = ty + cy - 1;
    int ramYStart = (pBBEP->native_height - 1) - ty;
    int ramYEnd = (pBBEP->native_height - 1) - yLast;

    bbepWriteCmd(pBBEP, SSD1608_SET_RAMYPOS);
    uc[0] = (uint8_t)(ramYStart & 0xff);
    uc[1] = (uint8_t)(ramYStart >> 8);
    uc[2] = (uint8_t)(ramYEnd & 0xff);
    uc[3] = (uint8_t)(ramYEnd >> 8);
    bbepWriteData(pBBEP, uc, 4);

    bbepWriteCmd(pBBEP, SSD1608_SET_RAMYCOUNT);
    uc[0] = (uint8_t)(ramYStart & 0xff);
    uc[1] = (uint8_t)(ramYStart >> 8);
    bbepWriteData(pBBEP, uc, 2);
}

static void partial_set_ep426_ram_y(BBEPDISP *pBBEP, int ty, int cy) {
    uint8_t uc[4];
    int yLast = ty + cy - 1;

    // Match epd426_init_* 0x45 wire order: Y start in bytes 0-1, Y end in bytes 2-3.
    bbepWriteCmd(pBBEP, SSD1608_SET_RAMYPOS);
    uc[0] = (uint8_t)ty;
    uc[1] = (uint8_t)(ty >> 8);
    uc[2] = (uint8_t)yLast;
    uc[3] = (uint8_t)(yLast >> 8);
    bbepWriteData(pBBEP, uc, 4);

    bbepWriteCmd(pBBEP, SSD1608_SET_RAMYCOUNT);
    uc[0] = (uint8_t)ty;
    uc[1] = (uint8_t)(ty >> 8);
    bbepWriteData(pBBEP, uc, 2);
}

static void partial_set_pixel_ram_x(BBEPDISP *pBBEP, int x, int cx) {
    uint8_t uc[4];
    int px0 = x;
    int px1 = x + cx - 1;
    if (panel_uses_ep426_x_decrement(pBBEP)) {
        px0 = (pBBEP->native_width - 1) - x;
        px1 = (pBBEP->native_width - 1) - (x + cx - 1);
    }

    bbepWriteCmd(pBBEP, SSD1608_SET_RAMXPOS);
    uc[0] = (uint8_t)(px0 & 0xff);
    uc[1] = (uint8_t)((px0 >> 8) & 0xff);
    uc[2] = (uint8_t)(px1 & 0xff);
    uc[3] = (uint8_t)(px1 >> 8);
    bbepWriteData(pBBEP, uc, 4);

    bbepWriteCmd(pBBEP, SSD1608_SET_RAMXCOUNT);
    uc[0] = (uint8_t)(px0 & 0xff);
    uc[1] = (uint8_t)(px0 >> 8);
    bbepWriteData(pBBEP, uc, 2);
}

static void partial_set_addr_window(BBEPDISP *pBBEP, int x, int y, int cx, int cy) {
    if (!panel_skips_bbep_set_addr_window()) {
        bbepSetAddrWindow(pBBEP, x, y, cx, cy);
        return;
    }
    if (!pBBEP) return;

    uint8_t uc[4];
    int ty = y;
    cx = (cx + 7) & 0xfff8;

    if (panel_uses_pixel_ram_x(pBBEP)) {
        partial_set_pixel_ram_x(pBBEP, x, cx);
    } else {
        int tx = x / 8;
        bbepWriteCmd(pBBEP, SSD1608_SET_RAMXPOS);
        uc[0] = (uint8_t)tx;
        uc[1] = (uint8_t)(tx + ((cx - 1) >> 3));
        bbepWriteData(pBBEP, uc, 2);
        bbepCMD2(pBBEP, SSD1608_SET_RAMXCOUNT, (uint8_t)tx);
    }

    if (panel_uses_ep426_x_decrement(pBBEP)) {
        partial_set_ep426_ram_y(pBBEP, ty, cy);
    } else if (panel_uses_ep397_y_decrement(pBBEP)) {
        partial_set_ep397_ram_y(pBBEP, ty, cy);
    } else {
        bbepWriteCmd(pBBEP, SSD1608_SET_RAMYPOS);
        uc[0] = (uint8_t)ty;
        uc[1] = (uint8_t)(ty >> 8);
        uc[2] = (uint8_t)(ty + cy - 1);
        uc[3] = (uint8_t)((ty + cy - 1) >> 8);
        bbepWriteData(pBBEP, uc, 4);
        uc[0] = (uint8_t)ty;
        uc[1] = (uint8_t)(ty >> 8);
        bbepWriteCmd(pBBEP, SSD1608_SET_RAMYCOUNT);
        bbepWriteData(pBBEP, uc, 2);
    }
    bbepWaitBusy(pBBEP);
}

static bool partial_consume_bytes(uint8_t* data, uint32_t len) {
    if (partialCtx.compressed) {
        if (len > UINT32_MAX - partialCtx.bytes_received) return false;
    } else {
        if (partialCtx.bytes_received > partialCtx.expected_stream_size ||
            len > partialCtx.expected_stream_size - partialCtx.bytes_received) {
            return false;
        }
    }
    partialCtx.bytes_received += len;
    if (partialCtx.compressed) return zlib_stream_to_partial_write(data, len, false);
    return partial_write_stream_bytes(data, len);
}

static bool zlib_stream_to_direct_write(const uint8_t* data, uint32_t len, bool final) {
    od_zlib_status_t status = od_zlib_stream_push(data, len, final);
    if (status == OD_ZLIB_STATUS_ERROR) {
        writeSerial(String("zlib stream error: ") + od_zlib_stream_error(), true);
        return false;
    }

    for (;;) {
        size_t bytesOut = 0;
        status = od_zlib_stream_poll(decompressionChunk, OPENDISPLAY_DECOMPRESSION_CHUNK_SIZE, &bytesOut);
        if (bytesOut > 0) {
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_SEEED_GFX)
            if (seeed_driver_used()) {
                seeed_gfx_direct_write_chunk(decompressionChunk, (uint32_t)bytesOut);
                directWriteBytesWritten += (uint32_t)bytesOut;
            } else
#endif
            if (directWriteIsGray4() || directWriteBitplanes) {
                uint32_t before = directWriteBytesWritten;
                streamGray4Bytes(decompressionChunk, (uint32_t)bytesOut);
                if (directWriteBytesWritten - before != (uint32_t)bytesOut) {
                    return false;
                }
            } else
            {
                directWriteSinkBytes(decompressionChunk, (uint32_t)bytesOut);
            }
            if (directWriteBytesWritten > directWriteDecompressedTotal) {
                return false;
            }
        }
        if (status == OD_ZLIB_STATUS_OUTPUT_READY) continue;
        if (status == OD_ZLIB_STATUS_NEEDS_INPUT) return !final;
        if (status == OD_ZLIB_STATUS_DONE) {
            if (directWriteBytesWritten != directWriteDecompressedTotal) {
                return false;
            }
            return true;
        }
        writeSerial(String("zlib stream error: ") + od_zlib_stream_error(), true);
        return false;
    }
}

static bool zlib_stream_to_partial_write(const uint8_t* data, uint32_t len, bool final) {
    od_zlib_status_t status = od_zlib_stream_push(data, len, final);
    if (status == OD_ZLIB_STATUS_ERROR) {
        writeSerial(String("partial zlib stream error: ") + od_zlib_stream_error(), true);
        return false;
    }

    for (;;) {
        size_t bytesOut = 0;
        status = od_zlib_stream_poll(decompressionChunk, OPENDISPLAY_DECOMPRESSION_CHUNK_SIZE, &bytesOut);
        if (bytesOut > 0 && !partial_write_stream_bytes(decompressionChunk, (uint32_t)bytesOut)) return false;
        if (status == OD_ZLIB_STATUS_OUTPUT_READY) continue;
        if (status == OD_ZLIB_STATUS_NEEDS_INPUT) return !final;
        if (status == OD_ZLIB_STATUS_DONE) return partialCtx.bytes_written == partialCtx.expected_stream_size;
        writeSerial(String("partial zlib stream error: ") + od_zlib_stream_error(), true);
        return false;
    }
}

static bool partial_write_stream_bytes(uint8_t* data, uint32_t len) {
    uint32_t offset = 0;
    while (offset < len) {
        if (partialCtx.bytes_written >= partialCtx.expected_stream_size) return false;

        uint8_t targetPlane = partialCtx.bytes_written < partialCtx.plane_size ? PLANE_1 : PLANE_0;
        if (partialCtx.current_plane != targetPlane) {
            if (targetPlane == PLANE_0 && partialCtx.bytes_written != partialCtx.plane_size) return false;
            partial_set_addr_window(&bbep, partialCtx.x, partialCtx.y, partialCtx.width, partialCtx.height);
            bbepStartWrite(&bbep, targetPlane);
            partialCtx.current_plane = targetPlane;
        }

        uint32_t planeEnd = targetPlane == PLANE_1 ? partialCtx.plane_size : partialCtx.expected_stream_size;
        uint32_t chunk = planeEnd - partialCtx.bytes_written;
        if (chunk > len - offset) chunk = len - offset;
        bbepWriteData(&bbep, data + offset, (int)chunk);
        partialCtx.bytes_written += chunk;
        offset += chunk;
    }
    return true;
}

static bool partial_trigger_refresh(int refreshMode) {
    if (refreshMode < 0 || refreshMode > 3) refreshMode = REFRESH_PARTIAL;
    if (panel_skips_reinit_on_partial_refresh(&bbep)) {
        if (panel_uses_ep397_y_decrement(&bbep)) {
            static const uint8_t u8CMDz3[4] = {0xf7, 0xd7, 0xff, 0};
            bbepCMD2(&bbep, SSD1608_DISP_CTRL2, u8CMDz3[refreshMode]);
        } else {
            static const uint8_t u8CMD[4] = {0xf7, 0xc7, 0xff, 0xc0};
            bbepCMD2(&bbep, SSD1608_DISP_CTRL2, u8CMD[refreshMode]);
        }
        bbepWriteCmd(&bbep, SSD1608_MASTER_ACTIVATE);
        return waitforrefresh(60);
    }
    bbepRefresh(&bbep, refreshMode);
    return waitforrefresh(60);
}

static void partial_prepare_panel_ram(void) {
    // Delta in ms since function entry, to profile where prep wall-clock goes.
    uint32_t t0 = millis();
    writeSerial("[+" + String(millis() - t0) + "ms] EPD partial start: acquire panel session", true);
    // Acquire subsumes pwrmgm(true) + bbepInitIO + bbepWakeUp + init-seq resend.
    // Warm re-acquire skips the ~900 ms rail bring-up + bbepInitIO (Phase 1).
    bool cold = epdSessionAcquire(true);
    writeSerial("[+" + String(millis() - t0) + "ms] after epdSessionAcquire (" +
                String(cold ? "cold" : "warm") + ")", true);
    // The two white fills guarantee PLANE_0 == PLANE_1 OUTSIDE the rect so uninit
    // controller RAM can't flash noise during MASTER_ACTIVATE. A full-frame rect's
    // enforced plane_size*2 stream overwrites 100% of both planes, so there is no
    // "outside the rect" to protect — provably safe to skip even on a cold panel
    // (Phase 1 skip condition 1). Sub-rects still fill.
    bool fullFrame = partialCtx.x == 0 && partialCtx.y == 0 &&
                     partialCtx.width  == globalConfig.displays[0].pixel_width &&
                     partialCtx.height == globalConfig.displays[0].pixel_height;
    if (!fullFrame) {
        bbepFill(&bbep, BBEP_WHITE, PLANE_1);
        bbepFill(&bbep, BBEP_WHITE, PLANE_0);
        writeSerial("[+" + String(millis() - t0) + "ms] after fills (ran: sub-rect)", true);
    } else {
        writeSerial("[+" + String(millis() - t0) + "ms] fills skipped (full-frame rect)", true);
    }
}

static bool partial_write_to_panel(int refreshMode) {
    writeSerial("EPD refresh: PARTIAL (raw rect ", false);
    writeSerial(String(partialCtx.x), false);
    writeSerial(",", false);
    writeSerial(String(partialCtx.y), false);
    writeSerial(" ", false);
    writeSerial(String(partialCtx.width), false);
    writeSerial("x", false);
    writeSerial(String(partialCtx.height), false);
    writeSerial(")", true);

    if (partialCtx.bytes_written != partialCtx.expected_stream_size) return false;
//    delay(20);  THIS DELAY IS UNNEEDED
    epdRefreshInProgress = true;
    bool refreshSuccess = partial_trigger_refresh(refreshMode);
    epdRefreshInProgress = false;
    // A successful partial refresh leaves both controller planes consistent.
    if (refreshSuccess) epdPlanesPrepared = true;
    // Release keeps the panel warm (rail/SPI up, controller awake) on success;
    // powers it fully down on failure or on AXP2101 (window 0) boards.
    epdSessionRelease(refreshSuccess);
    return refreshSuccess;
}

static uint32_t calc_controller_plane_bytes(uint16_t width, uint16_t height) {
    return ((uint32_t)(width + 7u) / 8u) * height;
}

static uint32_t parse_be_u32(const uint8_t* data) {
    return ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
           ((uint32_t)data[2] << 8)  |  (uint32_t)data[3];
}

static void send_direct_write_nack(uint8_t opcode, uint8_t error, bool cleanupState) {
    displayed_etag = 0;
    if (cleanupState) {
        if (partialCtx.active) cleanup_partial_write_state();
        else cleanupDirectWriteState(false);
    }
    uint8_t errResponse[] = {RESP_NACK, opcode, error, 0x00};
    sendResponse(errResponse, sizeof(errResponse));
}
