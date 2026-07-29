#include "main.h"
#include "boot_screen.h"
#include "buzzer_control.h"
#include "communication.h"
#include "device_control.h"
#include "display_service.h"
#include "power_latch.h"
#include "wake_button.h"
#include "touch_input.h"
#include "encryption.h"
#include "ble_transport.h"
#include "od_log.h"

#if defined(TARGET_ESP32) && defined(OPENDISPLAY_LOG_UART)
#include <HardwareSerial.h>
#ifndef OPENDISPLAY_LOG_UART_RX
#define OPENDISPLAY_LOG_UART_RX 44
#endif
#ifndef OPENDISPLAY_LOG_UART_TX
#define OPENDISPLAY_LOG_UART_TX 43
#endif
static HardwareSerial LogSerialPort(1);
#endif

#ifdef TARGET_ESP32
// Distinguishes a hidden mid-cycle reset (PANIC/WDT/BROWNOUT/SW) from a real
// power-on or deep-sleep wake; any reset here clears the wake cause, so the
// next boot takes the NORMAL BOOT branch and redraws the boot screen.
static const char* resetReasonName(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_POWERON:   return "POWERON";
        case ESP_RST_EXT:       return "EXT";
        case ESP_RST_SW:        return "SW";
        case ESP_RST_PANIC:     return "PANIC";
        case ESP_RST_INT_WDT:   return "INT_WDT";
        case ESP_RST_TASK_WDT:  return "TASK_WDT";
        case ESP_RST_WDT:       return "WDT";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "UNKNOWN";
    }
}

// Defined with the sleep helpers below loop()'s activity poller; setup() logs
// the window length when arming the button-wake hold.
static uint32_t minWakeTimeMs();
#endif

void setup() {
    #if defined(TARGET_ESP32) && defined(OPENDISPLAY_LOG_UART)
    LogSerialPort.begin(115200, SERIAL_8N1, OPENDISPLAY_LOG_UART_RX, OPENDISPLAY_LOG_UART_TX);
    delay(100);
    #elif !defined(DISABLE_USB_SERIAL)
    Serial.begin(115200);
    #if defined(TARGET_NRF) && defined(OPENDISPLAY_BOOT_DIAG)
    // Full-firmware boot diagnostic. Reaching this LED proves that reset,
    // application handoff, C/C++ runtime initialization, global constructors,
    // FreeRTOS startup, and entry into setup() all completed.
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    digitalWrite(LED_GREEN, LED_STATE_ON);
    digitalWrite(LED_BLUE, !LED_STATE_ON);

    // Do not let later initialization hide the CDC port by faulting first.
    // The full application remains linked; execution continues only after a
    // host actually opens native USB serial.
    bool blueOn = false;
    uint32_t lastBlueToggleMs = millis();
    while (!Serial) {
        if (millis() - lastBlueToggleMs >= 250u) {
            lastBlueToggleMs = millis();
            blueOn = !blueOn;
            digitalWrite(LED_BLUE, blueOn ? LED_STATE_ON : !LED_STATE_ON);
        }
        delay(10);
    }
    digitalWrite(LED_BLUE, !LED_STATE_ON);
    Serial.println();
    Serial.println("[BOOTDIAG] ENTERED setup(); USB CDC connected");
    Serial.println("[BOOTDIAG] continuing normal boot in 5 seconds");
    Serial.flush();
    delay(5000);
    #else
    delay(100);
    #endif
    #endif
    #if defined(TARGET_ESP32) && defined(OPENDISPLAY_LOG_UART)
    od_log_init(&LogSerialPort);
    #elif !defined(DISABLE_USB_SERIAL)
    od_log_init(&Serial);
    #endif
    od_log_info("=== FIRMWARE INFO ===");
    uint8_t fwMajor = getFirmwareMajor();
    uint8_t fwMinor = getFirmwareMinor();
    uint8_t fwPatch = getFirmwarePatch();
    od_log_info("Firmware Version: %u.%u.%u", fwMajor, fwMinor, fwPatch);
    const char* shaCStr = SHA_STRING;
    String shaStr = String(shaCStr);
    if (shaStr.length() >= 2 && shaStr.charAt(0) == '"' && shaStr.charAt(shaStr.length() - 1) == '"') {
        shaStr = shaStr.substring(1, shaStr.length() - 1);
    }
    if (shaStr.length() > 0 && shaStr != "\"\"" && shaStr != "") {
        od_log_info("Git SHA: %s", shaStr.c_str());
    } else {
        od_log_info("Git SHA: (not set)");
    }
    // Set only by the ESP32 wake-cause check below; NRF has no deep-sleep wake path.
    bool is_deep_sleep_wake = false;
    bool woke_by_button = false;
    #ifdef TARGET_ESP32
    esp_reset_reason_t reset_reason = esp_reset_reason();
    const char* resetReasonStr = resetReasonName(reset_reason);
    od_log_info("Reset reason: %s (%d)", resetReasonStr, (int)reset_reason);
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    is_deep_sleep_wake = (wakeup_reason != ESP_SLEEP_WAKEUP_UNDEFINED);
    if (is_deep_sleep_wake) {
        woke_from_deep_sleep = true;
        deep_sleep_count++;
        od_log_info("=== WOKE FROM DEEP SLEEP ===");
        woke_by_button = detectButtonWake(wakeup_reason);  // logs the named cause + pin(s)
        od_log_info("Deep sleep count: %u", deep_sleep_count);
    } else {
        woke_from_deep_sleep = false;
        od_log_info("=== NORMAL BOOT ===");
        // The bootloader reloads RTC memory segments from the app image on every
        // reset except a deep-sleep wake, so RTC_DATA_ATTR does NOT survive
        // panic/WDT/SW/brownout resets: a hidden mid-cycle reset lands here with
        // count 0, indistinguishable from a true first boot (captured on hardware
        // in docs/FINDINGS_DEEP_SLEEP_WAKE_BOOT_SCREEN_2026-07-07.md).
        od_log_info("Deep sleep count (RTC): %u", deep_sleep_count);
    }
    #endif
    od_log_info("Starting setup...");
    if (is_deep_sleep_wake) { od_log_info("[wake] >> full_config_init"); od_log_flush(); }
#if defined(TARGET_NRF) && defined(OPENDISPLAY_BOOT_DIAG)
    Serial.println("[BOOTDIAG] before full_config_init()");
    Serial.flush();
#endif
    full_config_init();
#if defined(TARGET_NRF) && defined(OPENDISPLAY_BOOT_DIAG)
    Serial.println("[BOOTDIAG] after full_config_init()");
    Serial.flush();
#endif
#ifdef OPENDISPLAY_HAS_WIFI
    // Reserve mbedTLS's two ~16.7 KB record buffers HERE and nowhere else: config is
    // loaded (so we know whether TLS is even used) but ble.begin() and initWiFi() have not
    // yet taken their ~100 KB, so internal DRAM is still contiguous. mbedtls_ssl_setup()
    // needs both buffers contiguous and cannot be satisfied later on a churned heap --
    // observed failing with -0x7f00 at 51 KB free / 31.7 KB largest block. No-op when
    // encryption is disabled.
    od_tls_reserve_records();
#endif
    if (is_deep_sleep_wake) { od_log_info("[wake] << full_config_init >> initio"); od_log_flush(); }
#if defined(TARGET_NRF) && defined(OPENDISPLAY_BOOT_DIAG)
    Serial.println("[BOOTDIAG] before initio()");
    Serial.flush();
#endif
    initio();
#if defined(TARGET_NRF) && defined(OPENDISPLAY_BOOT_DIAG)
    Serial.println("[BOOTDIAG] after initio()");
    Serial.flush();
#endif
#ifdef TARGET_NRF
    // SoftDevice must start before display/SPI; advertising starts after boot screen.
    {
        // Named local, not a temporary: the name outlives the call regardless of
        // whether the stack copies it.
        String bleDeviceName = "OD" + getChipIdHex();
#ifdef OPENDISPLAY_BOOT_DIAG
        Serial.println("[BOOTDIAG] before ble.begin() / SoftDevice enable");
        Serial.flush();
#endif
        ble.begin(bleDeviceName.c_str());
#ifdef OPENDISPLAY_BOOT_DIAG
        Serial.println("[BOOTDIAG] after ble.begin() / SoftDevice enable");
        Serial.flush();
#endif
    }
#endif
    if (!is_deep_sleep_wake) {
        // Arm here rather than at declaration: this branch is the boot screen
        // redraw, and every real reset (power-on, panic, WDT, SW) clears the
        // wake cause and lands here. A deep-sleep wake skips it and keeps the
        // pre-sleep flag, so a wake never advertises as a reboot.
        rebootFlag = 1;
        // Wake keeps the panel image; skipping initDisplay() (EPD rail power +
        // full refresh) is the wake path's main energy saving.
        initDisplay();
        od_log_info("Display initialized");
    }
#ifdef TARGET_ESP32
    // Full BLE after display: ESP32 queues commands for loop() until setup returns.
    if (is_deep_sleep_wake) { od_log_info("[wake] >> ble_begin"); od_log_flush(); }
    {
        String bleDeviceName = "OD" + getChipIdHex();
        if (ble.begin(bleDeviceName.c_str())) {
            // Historical order: build the manufacturer data into the advertisement
            // BEFORE the first start(), since setAdvertisementData() must be the
            // last data call before start() (see ble_transport_esp32.cpp).
            updatemsdata();
            ble.startAdvertising();
            od_log_info("Device ready: %s", bleDeviceName.c_str());
            od_log_info("Waiting for BLE connections...");
        }
    }
    if (is_deep_sleep_wake) { od_log_info("[wake] << ble_begin"); od_log_flush(); }
#elif defined(TARGET_NRF)
    ble.startAdvertising();
#endif
    #ifdef OPENDISPLAY_HAS_WIFI
    if (!is_deep_sleep_wake) {
        initWiFi(false);  // wake: WiFi stays deferred to fullSetupAfterConnection()
    }
    #endif
    updatemsdata();
    if (is_deep_sleep_wake) { od_log_info("[wake] >> initButtons"); od_log_flush(); }
    initButtons();
    if (is_deep_sleep_wake) { od_log_info("[wake] >> initTouchInput"); od_log_flush(); }
    initTouchInput();
    #ifdef TARGET_ESP32
    if (is_deep_sleep_wake) {
        // Arm the awake window LAST so buttons/GT911 bring-up doesn't shrink the
        // host's connection window. Without this, loop() falls into the idle
        // branch and re-enters deep sleep almost immediately.
        od_log_info("Advertising for %u ms (sleep_timeout_ms), waiting for connection...", globalConfig.power_option.sleep_timeout_ms);
        advertising_timeout_active = true;
        advertising_start_time = millis();
        if (woke_by_button) {
            // A button press means a user is present: hold awake for at least
            // the minimum window so they (or a host) get time to interact.
            minWakeWindowActive = true;
            minWakeWindowStartMs = millis();
            uint32_t minWakeMs = minWakeTimeMs();
            od_log_info("Button wake: holding awake >= %u ms", (unsigned)minWakeMs);
        }
    } else if (deep_sleep_count == 0) {
        // First boot — or a hidden mid-cycle reset, which reloads the RTC count
        // to 0 (see the NORMAL BOOT comment above). Inert on wired devices:
        // every consumer of the hold is power_mode/deep-sleep gated.
        minWakeWindowActive = true;
        minWakeWindowStartMs = millis();
    }
    // Both sleep paths measure quiet time from here, not from power-on.
    lastActivityMs = millis();
    #endif
    od_log_info("=== Setup completed successfully ===");
#ifdef TARGET_ESP32
    od_log_info("Heap: free=%u min=%u", (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap());
#endif
}

uint32_t getDeepSleepCount() {
#ifdef TARGET_ESP32
    return deep_sleep_count;
#else
    return 0;
#endif
}

// Deferred work, serviced by loop(). File-static on purpose: these encode
// application policy, so nothing outside this file reads them, and the two that
// other translation units need to RAISE do so through the request functions
// below (declared in communication.h) rather than by touching the state.
//
// Declared here, above pollActivity(), because that reads the advertising flag
// as its only trace of a connect+drop landing entirely inside one loop pass.
//
// Not volatile: every writer runs on the loop task. That became true in Phase 3,
// when nRF's stack callbacks stopped running application code -- before that the
// disconnect path executed on the SoftDevice callback task.
static bool s_disconnectCleanupPending = false;
static bool s_advertisingRestartPending = false;
static bool s_msdUpdatePending = false;

void requestTransferSessionCleanup(void) {
    s_disconnectCleanupPending = true;
}

void requestAdvertisingRestart(void) {
    s_advertisingRestartPending = true;
}

#ifdef TARGET_ESP32
// Minimum awake window (first boot / button wake). A floor layered UNDER the
// quiet-window logic, not a replacement: sleep requires both the existing
// idle/advertising quiet condition AND this hold expired, so interaction keeps
// extending the quiet window inside and beyond the floor. Timer wakes never
// arm the hold — their behavior is unchanged.
static uint32_t minWakeTimeMs() {
    uint16_t s = globalConfig.power_option.min_wake_time_seconds;
    return (uint32_t)(s ? s : DEFAULT_MIN_WAKE_TIME_SECONDS) * 1000UL;
}

static bool minWakeHoldActive() {
    if (!minWakeWindowActive) return false;
    if (millis() - minWakeWindowStartMs >= minWakeTimeMs()) {
        minWakeWindowActive = false;
        od_log_info("Minimum wake window elapsed, deep sleep permitted");
        return false;
    }
    return true;
}

// Single point of activity detection. Rather than have every producer (BLE host
// task, buttons, touch, LAN) stamp a timestamp, sample the state they already
// mutate and treat any change since the previous pass as activity.
//
// Runs at the top of loop() so an event raised during the previous pass always
// lands before this pass decides to sleep.
static void pollActivity() {
    static bool activityPrimed = false;
    static uint8_t prevCommandHead = 0;
    static uint8_t prevResponseHead = 0;
    static uint8_t prevConnCount = 0;
    static bool prevLanSession = false;
    static uint8_t prevDynamic[sizeof(dynamicreturndata)] = {0};

    // Queue heads are producer-side, so a command that arrived and drained within
    // a single pass still registers. The heads wrap (RX mod COMMAND_QUEUE_SIZE,
    // TX mod RESPONSE_QUEUE_SIZE), but aliasing needs a whole queue of traffic
    // inside one pass, and queues only fill while a client is connected — which
    // stamps below regardless.
    const uint8_t commandHead = bleRxQueueHead();
    const uint8_t responseHead = bleTxQueueHead();
    // Covers connect and disconnect. The disconnect edge is what re-arms the
    // window so a dropped client gets a full reconnect opportunity.
    const uint8_t connCount = ble.connectedCount();
#ifdef OPENDISPLAY_HAS_WIFI
    const bool lanSession = wifiInitialized && wifiServerConnected && wifiClient.connected();
#else
    const bool lanSession = false;
#endif

    if (!activityPrimed) {
        activityPrimed = true;
    } else if (commandHead != prevCommandHead ||
               responseHead != prevResponseHead ||
               connCount != prevConnCount ||
               lanSession != prevLanSession ||
               // Button presses and touch events land here before advertising.
               memcmp(prevDynamic, dynamicreturndata, sizeof(prevDynamic)) != 0 ||
               // Set by onDisconnect and cleared further down this loop, so it is
               // the only trace of a connect+drop that lands entirely between two
               // passes — connCount reads 0 on both sides of such a blip.
               s_advertisingRestartPending ||
               // A live link or unfinished work is activity in itself, not just its edges.
               connCount > 0 || lanSession ||
               bleRxQueuePending() || bleTxQueuePending()) {
        lastActivityMs = millis();
    }

    prevCommandHead = commandHead;
    prevResponseHead = responseHead;
    prevConnCount = connCount;
    prevLanSession = lanSession;
    memcpy(prevDynamic, dynamicreturndata, sizeof(prevDynamic));
}
#endif  // TARGET_ESP32 -- deep-sleep activity tracking is ESP32-only

// The BLE session helpers below are portable as of Phase 3: both targets now
// dispatch commands and service connect/disconnect from loop().

// Services the deferred BLE-disconnect session teardown flagged by
// the BLE transport's disconnect event. Runs on the loop() task so the heavyweight
// EPD force-off (bbepSleep/delay/SPI.end/rail cut) and partial/pipe cleanup never
// race SPI streaming or pipe-frame processing on the stack callback task.
// Deferred while a refresh is mid-flight; the flag stays set until a later pass
// clears it. Also raised by the LAN transport, so it is not a BLE-only path.
static void serviceBleDisconnectCleanup() {
    if (!s_disconnectCleanupPending || epdRefreshInProgress) return;
    s_disconnectCleanupPending = false;
    // BLE and LAN both raise this flag, so tear down only when the transport that
    // OWNS the in-flight transfer is the one that went away. Otherwise a BLE
    // disconnect kills a live LAN push (and a LAN disconnect kills a BLE push)
    // purely because the other link dropped. Owner is recorded at START.
    //
    // The guard is NOT inside OPENDISPLAY_HAS_WIFI, though it used to be. Only the
    // LAN half is WiFi-specific; ble.isConnected() is meaningful on every target, and
    // gating the whole test left nRF with no guard at all. That mattered: this flag
    // can be serviced tens of seconds late (loop() blocked in an EPD refresh), by
    // which time a NEW client may be connected and mid-transfer -- and the
    // resetPipeWriteState() below would destroy its session, not the departed one's.
#ifdef OPENDISPLAY_HAS_WIFI
    const bool lanOwnsSession = (transferSessionOrigin() != 0);   // != ORIGIN_BLE
    const bool ownerStillUp = lanOwnsSession ? wifiLanClientConnected() : ble.isConnected();
#else
    const bool lanOwnsSession = false;
    const bool ownerStillUp = ble.isConnected();
#endif
    if (ownerStillUp) {
        od_log_info("Disconnect cleanup skipped: transfer still owned by a live %s session",
                    lanOwnsSession ? "LAN" : "BLE");
        return;
    }
    // ACTIVE-only-teardown invariant: a WARM (post-successful-refresh) panel
    // SURVIVES disconnect and keeps its keep-alive window, so the cleanups below
    // no-op on power when WARM and only tear down a mid-transfer (PWR_ACTIVE)
    // session. No logic change needed for keep-alive.
    if (directWriteActive) cleanupDirectWriteState(true);
    // Partial sessions (0x76 or pipe-partial) power the panel without setting
    // directWriteActive; release it here instead of waiting on the 15-min watchdog.
    cleanupPartialWriteOnDisconnect();
    resetPipeWriteState();   // clear any pipe transfer + reorder queue on disconnect
}

// Deferral policy for re-arming the radio, formerly buried inside
// esp32_restart_ble_advertising(). BleTransport::restartAdvertising() is
// unconditional by contract; deciding *when* to call it is an application
// concern (mid-refresh, already reconnected, stack not up), so it lives here
// alongside the other loop()-serviced BLE helpers. The flag stays raised on the
// "not yet" paths so a later pass retries.
static void serviceBleAdvertisingRestart() {
    if (!s_advertisingRestartPending) return;
    // Capability gate, and the reason this helper is safe for ANY caller to
    // raise the flag: where the stack re-arms advertising itself (nRF's
    // restartOnDisconnect(true)), driving our own stop()/start() would fight it.
    // Refusing here rather than at each raise site means a new raiser -- the
    // post-refresh hook in display_service.cpp, or a future portable
    // requestAdvertisingRestart() -- cannot reintroduce that conflict by
    // forgetting a target guard.
    if (ble.restartsAdvertisingOnDisconnect()) {
        s_advertisingRestartPending = false;
        return;
    }
    if (!ble.isReady()) return;                              // stack down; retry later
    if (ble.isConnected()) {                                 // a client beat us to it
        s_advertisingRestartPending = false;
        return;
    }
    if (epdRefreshInProgress) return;                        // never mid-refresh
    s_advertisingRestartPending = false;
    ble.restartAdvertising();
    updatemsdata();
}

// Translate the transport's consume-once connect/disconnect events into the
// application's deferred-work flags, and do the connect-side work that used to
// run inline on the stack callback task. Must run before anything that reads
// those flags -- including pollActivity(), which uses
// s_advertisingRestartPending as its only trace of a connect+drop landing
// entirely between two passes.
static void serviceBleEvents() {
    if (ble.takeConnectedEvent()) {
        rebootFlag = 0;
        s_msdUpdatePending = true;
        // SoftDevice PHY/DLE calls on nRF, no-op on ESP32. Deliberately here and
        // not in the connect callback: the callback contract is copy-and-flag only.
        ble.requestFastLink();
    }
    uint8_t disconnectReason = 0;
    uint8_t rxBoundary = 0;
    if (ble.takeDisconnectedEvent(&disconnectReason, &rxBoundary)) {
        od_log_info("Disconnect reason: %u", disconnectReason);
        // Drop anything the departed client left in the RX ring. Without this,
        // serviceBleRx() runs BEFORE serviceBleDisconnectCleanup() in the pass, so
        // up to a full window of frames from a dead session would dispatch --
        // touching pipe/partial state that resetPipeWriteState() is about to
        // discard anyway, and emitting responses that queueBleNotifyCopy() then
        // drops for want of a connection.
        //
        // Bounded by rxBoundary, the ring head captured when that link went down.
        // "Discard everything present now" was wrong: loop() can sit inside a ~16 s
        // EPD refresh, and a disconnect, a reconnect, and the NEW client's first
        // command all land before this event is serviced -- so the flush ate a frame
        // from a session that had never disconnected.
        //
        // Deliberately here and NOT in serviceBleDisconnectCleanup(): that flag is
        // raised by the LAN transport too, and a LAN drop must not discard queued
        // BLE frames. Only a real BLE disconnect event invalidates this ring.
        const uint8_t droppedRx = bleRxQueueDiscardTo(rxBoundary);
        if (droppedRx > 0) {
            od_log_warn("Dropped %u queued command(s) from the disconnected client", droppedRx);
        }
        // Raise the flag; do NOT tear the session down here. The teardown belongs
        // in serviceBleDisconnectCleanup(), which holds it off while an EPD
        // refresh is mid-flight and checks whether LAN still owns the transfer.
        // Doing it inline would reintroduce the mid-refresh SPI teardown that
        // moving nRF off the callback task was meant to eliminate.
        s_disconnectCleanupPending = true;
        // Raised unconditionally: serviceBleAdvertisingRestart() owns the
        // capability decision, so this site does not need to know whether the
        // stack re-arms the radio by itself. On such a target the flag is simply
        // cleared unserviced, later in this same pass.
        s_advertisingRestartPending = true;
    }
}

// Bounded drain: service up to COMMAND_QUEUE_SIZE commands per pass so a
// sustained W-deep PIPE_WRITE window burst isn't starved at one-per-loop, while
// the rest of loop() still runs each pass. Responses are flushed BETWEEN
// commands so pipe ACKs generated by this drain never overflow the 10-slot
// response ring (see serviceBleTx).
//
// This is the only place commands are dispatched, on either target. Nothing may
// call it from inside a command handler: doing so would make handlers reentrant
// and corrupt multi-frame transfer state mid-stream.
static void serviceBleRx() {
    uint8_t drained = 0;
    while (drained < COMMAND_QUEUE_SIZE) {
        CommandQueueItem* item = bleRxQueuePeek();
        if (item == nullptr) break;
        // imageDataWritten (misleading name) actually services any BLE command.
        // The dispatch banner (commandName() in communication.cpp) already logs
        // which command runs, so no drain-start/-end framing line is needed here.
        imageDataWritten(0, nullptr, item->data, item->len);
        bleRxQueueConsume();
        drained++;
        serviceBleTx();
    }
}

// Platform policy hook 1: work this target does before the shared body, with the
// option to claim the whole pass. Only ESP32 has any -- the deep-sleep wake
// window is a real capability difference, so the plan says hook it rather than
// merge it. Returns true when the pass is finished and loop() must return.
static bool platformLoopPrologue() {
#ifdef TARGET_ESP32
    pollActivity();
    // THIS IS THE MAIN (FIRST) LOOP FOR A DEEP SLEEP ENABLED ESP32
    if (woke_from_deep_sleep && advertising_timeout_active) {
        if (ble.isConnected()) {
            od_log_info("BLE connection established - switching to full mode");
            advertising_timeout_active = false;
            fullSetupAfterConnection();
            woke_from_deep_sleep = false;
            return true;
        }
        // A connect+drop entirely inside one poll gap leaves the radio dark for the
        // rest of the window; the flags are otherwise only serviced past this return.
        serviceBleDisconnectCleanup();   // tear down before re-advertising
        serviceBleAdvertisingRestart();
        uint32_t advertising_timeout_ms = globalConfig.power_option.sleep_timeout_ms;
        if (advertising_timeout_ms == 0) {
            advertising_timeout_ms = DEFAULT_IDLE_HOLD_MS;
        }
        // Measured from the last activity, not from window start: a client that
        // connects and drops re-arms the full window instead of inheriting it.
        uint32_t idle_duration = millis() - lastActivityMs;
        // On a button wake the min-wake hold keeps this window open past the
        // quiet timeout; idleDelay(50) below services buttons/touch throughout.
        if (idle_duration >= advertising_timeout_ms && !minWakeHoldActive()) {
            uint32_t advertisingElapsedMs = millis() - advertising_start_time;
            od_log_info("BLE advertising timeout (idle %u ms of %u ms window) - no connection, returning to deep sleep",
                        (unsigned)idle_duration, (unsigned)advertisingElapsedMs);
            advertising_timeout_active = false;
            enterDeepSleep();
            return true;
        }
        // idleDelay() services buttons + touch (and LED flash) while it waits, so a
        // wake-time touch is polled during this window even though the branch returns
        // on every pass until a client connects. It lands in dynamicreturndata, reaches
        // a mid-window client, and pollActivity picks it up next pass to hold the window
        // open — none of which happens if we just delay() here without servicing input.
        idleDelay(50); // idleDelay() polls touch and buttons while waiting
        return true;
    }
#endif
    return false;
}

// Platform policy hook 2: what this target does when nothing is in flight.
// ESP32 owns the deep-sleep decision; nRF just idles at its configured cadence.
// Never reached while work is outstanding -- loop() handles that case itself.
static void platformIdle() {
#ifdef TARGET_ESP32
    if (globalConfig.power_option.deep_sleep_time_seconds > 0 && globalConfig.power_option.power_mode == 1) {
        uint32_t idleHoldMs = globalConfig.power_option.sleep_timeout_ms;
        if (idleHoldMs == 0) {
            idleHoldMs = DEFAULT_IDLE_HOLD_MS;
        }
        uint32_t idleMs = millis() - lastActivityMs;
        // The min-wake hold covers first boot and connect-then-drop during a
        // button-wake window (woke_from_deep_sleep cleared on connect above).
        if (idleMs < idleHoldMs || minWakeHoldActive()) {
            idleDelay(5);
        } else {
            od_log_info("Idle %u ms (hold %u ms) - entering deep sleep", (unsigned)idleMs, (unsigned)idleHoldMs);
            enterDeepSleep();
        }
    } else {
        // Non-battery (USB) idle: keep the loop responsive. A 2000 ms idle here
        // stalls BLE command/response servicing for up to 2 s when a client
        // connects mid-delay (the queued write waits out the delay before the
        // loop re-evaluates), which reads as a sluggish/unreliable first
        // exchange. Use the same short cadence as the battery idle-hold path.
        idleDelay(5);
    }
    static uint32_t lastMsdUpdate = 0;
    if (millis() - lastMsdUpdate >= 60000) {
        lastMsdUpdate = millis();
        updatemsdata();
    }
#else
    if (globalConfig.power_option.sleep_timeout_ms > 0) {
        idleDelay(globalConfig.power_option.sleep_timeout_ms);
        updatemsdata();
    } else {
        idleDelay(500);
    }
#endif
}

// One loop body for both targets. The per-target policy that genuinely differs
// lives in the two hooks above; everything here is shared.
void loop() {
    serviceBleEvents();
    processLedFlash();
    epdSessionTick();   // millis()-poll: power the panel down screen_timeout_seconds after last release
    buzzerService();

    if (platformLoopPrologue()) return;

    // Drain commands, then service the deferred work the stack callbacks flagged.
    // Cleanup runs before the advertising restart so a disconnected session is
    // fully torn down before the radio re-arms.
    serviceBleRx();
    serviceBleTx();
    serviceBleDisconnectCleanup();
    if (s_msdUpdatePending) {
        s_msdUpdatePending = false;
        updatemsdata();
    }
    serviceBleAdvertisingRestart();   // no-op where the stack re-arms itself

    // Session watchdogs. Shared as of Phase 4: these are transport-agnostic and
    // were ESP32-only for no reason other than living in the ESP32 loop arm, so
    // nRF gains them. A hung transfer there used to sit until disconnect.
    // Both now live in display_service.cpp, beside the transfer state they tear
    // down. Splitting them across files is how the direct-write one came to
    // release the panel while leaving its enclosing PIPE session running.
    checkTransferTimeouts();

    #ifdef OPENDISPLAY_HAS_WIFI
    // WiFi handling runs after BLE queue processing to avoid blocking
    // BLE command responses (moved from top of loop in v1.6 fix).
    handleWiFiServer();
    static uint32_t lastWiFiCheck = 0;
    if (wifiInitialized && (millis() - lastWiFiCheck > 10000)) {
        lastWiFiCheck = millis();
        wl_status_t wifiStatus = WiFi.status();
        if (wifiStatus != WL_CONNECTED && wifiConnected) {
            od_log_warn("WiFi connection lost (status: %d)", wifiStatus);
            wifiConnected = false;
            if (wifiServerConnected) {
                disconnectWiFiServer();
            }
        } else if (wifiStatus == WL_CONNECTED && !wifiConnected) {
            String wifiIp = WiFi.localIP().toString();
            od_log_info("WiFi reconnected (IP: %s)", wifiIp.c_str());
            wifiConnected = true;
            restartWiFiLanAfterReconnect();
        }
    }
    const bool wifiLanSession = wifiInitialized && wifiServerConnected && wifiClient.connected();
    #else
    const bool wifiLanSession = false;
    #endif

    // Work in flight *this iteration* only. Every term is transient and most are
    // cleared earlier in this same pass, so this must never be the sole gate on
    // deep sleep — lastActivityMs supplies the quiet window. The terms that only
    // one target can ever raise (s_advertisingRestartPending, wifiLanSession)
    // are simply false on the other, so one expression serves both.
    // eventPending() closes the callback-timing hole: an event raised after
    // serviceBleEvents() ran in this pass is otherwise invisible until the next
    // pass -- and this pass is about to park. It is transient like the rest;
    // take*Event() clears the peeked flag at the next loop top.
    //
    // No transfer-state term belongs here, and its absence is deliberate. A live
    // transfer already has its connected BLE or LAN owner holding the gate, and
    // one whose transport is gone cannot progress, so refusing to sleep on it
    // would burn power for work that will never happen. That state is healed in
    // checkTransferTimeouts() instead. See
    // docs/PLAN_WORK_GATE_TRANSFER_TERMS_2026-07-29.md.
    const bool workInFlight = bleRxQueuePending() || bleTxQueuePending() ||
                              ble.isConnected() ||
                              ble.eventPending() ||
                              s_advertisingRestartPending ||
                              epdRefreshInProgress ||
                              wifiLanSession;
    if (workInFlight) {
        delay(1);
    } else {
        platformIdle();
    }
    ble.tick();          // no-op on ESP32
    processButtonEvents();
    processTouchInput();
    buzzerService();
}

// Button/LED runtime moved to device_control.cpp

// Cooperative delay: services the things that must keep ticking while loop()
// waits. Called ONLY from loop() -- never from a command handler, which is what
// makes the RX rule below safe to state as a hard invariant.
void idleDelay(uint32_t delayMs) {
    const uint32_t CHECK_INTERVAL_MS = 100;
    uint32_t remainingDelay = delayMs;
    while (remainingDelay > 0) {
        ble.tick();   // no-op on ESP32
        processButtonEvents();
        processTouchInput();
        processLedFlash();
        epdSessionTick();   // expire the keep-alive window while a long idleDelay blocks
        buzzerService();
        // Keep responses moving: loop() is the ring's only drainer, so a long
        // idleDelay would otherwise hold queued ACKs for its full duration.
        // Draining TX is safe here because it only notifies; it dispatches nothing.
        serviceBleTx();
        // RX and transport events are deliberately NOT serviced here -- return to
        // loop() and let serviceBleEvents()/serviceBleRx() handle them at top
        // level. Dispatching RX inside idleDelay would make command handlers
        // reentrant the moment anything calls idleDelay from a handler, corrupting
        // multi-frame transfer state; consuming events here would move the single
        // consumer out of loop() and break the ordering serviceBleEvents() relies
        // on. Returning early also caps latency at one CHECK_INTERVAL_MS rather
        // than the caller's full delay, which is what makes nRF's move to
        // loop()-side dispatch viable: its idle waits are 500 ms and up.
        //
        // This break set answers "did work appear while we were parked?", which is
        // NOT the question workInFlight answers ("is there work?"). A term belongs
        // here only if (i) it can go false->true asynchronously, on a task other
        // than loop(), and (ii) idleDelay cannot service that work itself. RX and
        // transport events are the only two that qualify. bleTxQueuePending() is
        // excluded because serviceBleTx() already runs every chunk above;
        // epdRefreshInProgress, the transfer-state flags, s_advertisingRestartPending
        // and wifiLanSession are excluded because only loop() itself raises them,
        // so none can change while loop() is sitting inside this function.
        if (bleRxQueuePending() || ble.eventPending()) return;
        uint32_t chunkDelay = (remainingDelay > CHECK_INTERVAL_MS) ? CHECK_INTERVAL_MS : remainingDelay;
        delay(chunkDelay);
        remainingDelay -= chunkDelay;
    }
}


#ifdef TARGET_ESP32
void fullSetupAfterConnection() {
    od_log_info("=== Full Setup After Connection ===");
#ifdef OPENDISPLAY_HAS_WIFI
    initWiFi(false);
#endif
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_FASTEPD)
    if (globalConfig.display_count > 0 && fastepd_driver_used()) {
        od_log_info("Panel: FastEPD (bb_epaper not used)");
        od_log_info("=== Full setup completed ===");
        return;
    }
#endif
    if (globalConfig.display_count > 0) {
        memset(&bbep, 0, sizeof(BBEPDISP));
        int panelType = mapEpd(globalConfig.displays[0].panel_ic_type);
        od_log_info("Panel type: %d", panelType);
        bbepSetPanelType(&bbep, panelType);
        bbepSetRotation(&bbep, globalConfig.displays[0].rotation * 90);
    }
    od_log_info("=== Full setup completed ===");
}

void enterDeepSleep(bool force, uint16_t overrideSleepSeconds) {
    if (globalConfig.power_option.power_mode != 1) {
        od_log_debug("Skipping deep sleep - not battery powered (power_mode: %u)", globalConfig.power_option.power_mode);
        return;
    }
    if (globalConfig.power_option.deep_sleep_time_seconds == 0) {
        od_log_debug("Skipping deep sleep - deep_sleep_time_seconds is 0");
        return;
    }
    // Callers sample their idle state before getting here; a central can connect in
    // that gap. Re-check so we never tear down the stack on a live link.
    if (!force && ble.isConnected()) {
        od_log_debug("Skipping deep sleep - BLE client connected");
        lastActivityMs = millis();
        return;
    }
    // Defense in depth for the min-wake hold (first boot / button wake). MUST
    // stay ahead of the advertising stop below: everything past that point
    // commits to esp_deep_sleep_start(), so a late abort would leave the device
    // awake with the radio dark. force (host 0x0053) bypasses the hold.
    if (!force && minWakeHoldActive()) {
        od_log_debug("Skipping deep sleep - minimum wake window active");
        return;
    }
    // Panel power-down MUST sit below every early-return above (including the
    // min-wake hold): on mains (power_mode != 1) enterDeepSleep bails before here,
    // so a WARM panel stays warm and the keep-alive tick in idleDelay(2000) expires
    // it after the configured window. On battery this is the routine
    // WARM-at-idle-hold-expiry path (idle-hold default 10 s often < the keep-alive
    // window) and also closes the pre-existing "deep sleep never powers the panel
    // down" hazard. Net effect on battery ESP32: effective keep-alive =
    // min(configured window, idle-hold).
    epdSessionForceOff();
    woke_from_deep_sleep = true; // Will be true on next boot
    ble.stopAdvertising();
    delay(200);
    ble.end();
    delay(100);
    od_log_info("BLE deinitialized");
    // Host override (0x0053 payload) applies to this one cycle only: it is a
    // parameter, never stored, so an aborted or later sleep reverts to config.
    uint16_t sleepSeconds = overrideSleepSeconds ? overrideSleepSeconds
                                                 : globalConfig.power_option.deep_sleep_time_seconds;
    uint64_t sleep_timeout_us = (uint64_t)sleepSeconds * 1000000ULL;
    esp_sleep_enable_timer_wakeup(sleep_timeout_us);
    // After the timer arm, before powerLatchHoldForSleep(): the latch-hold
    // manipulation then cannot disturb freshly configured RTC pulls, and its
    // gpio_hold_en() touches only the latch pin, never the wake pads.
    armButtonWakeSources();
    od_log_info("Entering deep sleep for %u seconds%s", sleepSeconds,
                overrideSleepSeconds ? " (host override, one cycle)" : " (config)");
    od_log_info("Heap: free=%u min=%u", (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap());
    od_log_flush(); // drain UART/Serial prior to deep sleep
    delay(100); // Brief delay to ensure serial output is sent
    powerLatchHoldForSleep();
    esp_deep_sleep_start();
}
#endif

// Panel rail is cut after this — drive control lines LOW; BUSY stays an input.
static void configureDisplayPinsLowPower() {
    const DisplayConfig& d = globalConfig.displays[0];
    const uint8_t pins[] = {
        d.cs_pin, d.clk_pin, d.data_pin, d.dc_pin, d.reset_pin,
    };
    for (uint8_t pin : pins) {
        if (pin == 0xFF) continue;
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
    if (d.busy_pin != 0xFF) {
        pinMode(d.busy_pin, INPUT);
    }

    if (!(globalConfig.system_config.device_flags &
          (DEVICE_FLAG_BATTERY_LATCH | DEVICE_FLAG_PWR_LATCH_DFF))) {
        const uint8_t auxPins[] = {
            globalConfig.system_config.pwr_pin_2,
            globalConfig.system_config.pwr_pin_3,
        };
        for (uint8_t pin : auxPins) {
            if (pin == 0xFF || pin == 0) continue;
            pinMode(pin, OUTPUT);
            digitalWrite(pin, LOW);
        }
    }
}

void pwrmgm(bool onoff){
    if(globalConfig.display_count == 0){
        od_log_warn("No display configured");
        return;
    }
    // Idempotency guard keyed on the panel power state machine (the single source
    // of truth). Makes every legacy caller safe: a same-state call becomes a no-op,
    // while a real transition (true-after-false, or the boot true/false/true rail
    // cycle) always proceeds because each call flips the state. pwrmgm owns the
    // OFF<->(ACTIVE) transitions; epdSessionAcquire/Release own ACTIVE<->WARM.
    if (onoff  && pwrmgmState != PWR_OFF) return;   // already powered (WARM or ACTIVE)
    if (!onoff && pwrmgmState == PWR_OFF) return;   // already off
    displayPowerState = onoff;
    pwrmgmState = onoff ? PWR_ACTIVE : PWR_OFF;
    if (!onoff) pwrmgmOffDeadlineMs = 0;
    uint8_t axp2101_bus_id = 0xFF;
    bool axp2101_found = false;
    for(uint8_t i = 0; i < globalConfig.sensor_count; i++){
        if(globalConfig.sensors[i].sensor_type == OD_SENSOR_TYPE_AXP2101){
            axp2101_bus_id = globalConfig.sensors[i].bus_id;
            axp2101_found = true;
            break;
        }
    }
    if(axp2101_found){
        if(onoff){
        od_log_info("Powering up AXP2101 PMIC...");
            initAXP2101(axp2101_bus_id);
        }
        else{
            od_log_info("Powering down AXP2101 PMIC...");
            powerDownAXP2101();
            Wire.end();
            invalidateOpenDisplayWire();
            pinMode(47, OUTPUT);
            digitalWrite(47, HIGH);
            pinMode(48, OUTPUT);
            digitalWrite(48, HIGH);
        }
    }
#if defined(TARGET_ESP32) && defined(OPENDISPLAY_FASTEPD)
    const bool fastepd_driver_spi = fastepd_driver_used();
#else
    const bool fastepd_driver_spi = false;
#endif
    const DisplayConfig& disp = globalConfig.displays[0];
    if (onoff) {
        if (globalConfig.system_config.pwr_pin != 0xFF) {
            digitalWrite(globalConfig.system_config.pwr_pin, HIGH);
            delay(800);
        } else {
            od_log_warn("Power pin not set");
        }
        if (!fastepd_driver_spi) {
            if (disp.reset_pin != 0xFF) {
                pinMode(disp.reset_pin, OUTPUT);
                digitalWrite(disp.reset_pin, HIGH);
            }
            if (disp.cs_pin != 0xFF) {
                pinMode(disp.cs_pin, OUTPUT);
                digitalWrite(disp.cs_pin, HIGH);
            }
            if (disp.dc_pin != 0xFF) {
                pinMode(disp.dc_pin, OUTPUT);
                digitalWrite(disp.dc_pin, LOW);
            }
            if (disp.clk_pin != 0xFF) {
                pinMode(disp.clk_pin, OUTPUT);
                digitalWrite(disp.clk_pin, LOW);
            }
            if (disp.data_pin != 0xFF) {
                pinMode(disp.data_pin, OUTPUT);
                digitalWrite(disp.data_pin, LOW);
            }
            if (disp.busy_pin != 0xFF) {
                pinMode(disp.busy_pin, INPUT);
            }
            delay(100);
        } else {
            if (disp.reset_pin != 0xFF) {
                pinMode(disp.reset_pin, OUTPUT);
                digitalWrite(disp.reset_pin, HIGH);
            }
            delay(200);
        }
        initOrRestoreWireForOpenDisplay();
    } else {
        if (!fastepd_driver_spi) {
            SPI.end();
        }
        // Keep I2C alive when sensors/touch use data_bus[0] (e.g. reTerminal MISC_I2C on GPIO0/1).
        if (!openDisplayI2cBusConfigured()) {
            Wire.end();
            invalidateOpenDisplayWire();
        }
        if (globalConfig.system_config.pwr_pin != 0xFF) {
            configureDisplayPinsLowPower();
            digitalWrite(globalConfig.system_config.pwr_pin, LOW);
        }
    }
}

void xiaoinit(){
    powerDownExternalFlash(20,24,21,25,22,23);
    //pinMode(31, INPUT);
    //pinMode(14, INPUT);
    pinMode(13, OUTPUT);  //that actually does something
    digitalWrite(13, LOW);
    //pinMode(17, INPUT);
}

void ws_pp_init(){
    od_log_info("===  Photo Printer Initialization ===");
    pinMode(21, OUTPUT);
    digitalWrite(21, HIGH);
    pinMode(1, INPUT);
    pinMode(2, INPUT);
    pinMode(3, INPUT);
    pinMode(4, INPUT);
    pinMode(5, OUTPUT);
    digitalWrite(5, HIGH);
    pinMode(6, INPUT);
    pinMode(7, LOW);
    digitalWrite(7, LOW);
    pinMode(14, INPUT);
    pinMode(15, INPUT);
    pinMode(16, INPUT);
    pinMode(17, INPUT);
    pinMode(18, INPUT);
    pinMode(38, OUTPUT);
    digitalWrite(38, HIGH);
    pinMode(39, OUTPUT);
    digitalWrite(39, HIGH);
    pinMode(40, OUTPUT);
    digitalWrite(40, HIGH);
    pinMode(41, OUTPUT);
    digitalWrite(41, HIGH);
    pinMode(42, OUTPUT);
    digitalWrite(42, HIGH);
    pinMode(45, OUTPUT);
    digitalWrite(45, HIGH);
    od_log_info("Photo Printer initialized");
}

#ifdef TARGET_NRF
void powerDownExternalFlashFromConfig(void) {
    if (!globalConfig.loaded || globalConfig.flash_config_count == 0) {
        return;
    }
    const FlashConfig* flashCfg = nullptr;
    for (uint8_t i = 0; i < globalConfig.flash_config_count; i++) {
        if ((globalConfig.flash_configs[i].flags & OD_FLASH_FLAG_ENABLED) != 0) {
            flashCfg = &globalConfig.flash_configs[i];
            break;
        }
    }
    if (flashCfg == nullptr) {
        return;
    }
    const uint8_t mosiPin = flashCfg->mosi_pin;
    const uint8_t sckPin = flashCfg->sck_pin;
    const uint8_t csPin = flashCfg->cs_pin;
    if (mosiPin == 0xFF || sckPin == 0xFF || csPin == 0xFF) {
        od_log_warn("Flash config: invalid MOSI/SCK/CS pins");
        return;
    }
    od_log_debug("Flash config: deep sleep MOSI=%u SCK=%u CS=%u", mosiPin, sckPin, csPin);

    pinMode(mosiPin, OUTPUT);
    pinMode(sckPin, OUTPUT);
    pinMode(csPin, OUTPUT);
    digitalWrite(sckPin, LOW);
    digitalWrite(csPin, LOW);

    uint8_t cmd = 0xB9;
    for (uint8_t bit = 0; bit < 8; bit++) {
        digitalWrite(mosiPin, (cmd & 0x80) ? HIGH : LOW);
        cmd <<= 1;
        delayMicroseconds(1);
        digitalWrite(sckPin, HIGH);
        delayMicroseconds(1);
        digitalWrite(sckPin, LOW);
    }
    digitalWrite(csPin, HIGH);
    delayMicroseconds(30);

    // Park like powerDownExternalFlash: CLK/MOSI LOW, CS HIGH (deselected, deep sleep).
    pinMode(mosiPin, OUTPUT);
    digitalWrite(mosiPin, LOW);
    pinMode(sckPin, OUTPUT);
    digitalWrite(sckPin, LOW);
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
}
#else
void powerDownExternalFlashFromConfig(void) {}
#endif

bool powerDownExternalFlash(uint8_t mosiPin, uint8_t misoPin, uint8_t sckPin, uint8_t csPin, uint8_t wpPin, uint8_t holdPin) {
    #ifdef TARGET_NRF
    auto spiTransfer = [&](uint8_t data) -> uint8_t {
        uint8_t result = 0;
        for (int i = 7; i >= 0; i--) {
            digitalWrite(mosiPin, (data >> i) & 1);
            digitalWrite(sckPin, LOW);
            delayMicroseconds(1);
            result |= (digitalRead(misoPin) << i);
            digitalWrite(sckPin, HIGH);
            delayMicroseconds(1);
        }
        return result;
    };
    od_log_info("=== External Flash Power-Down ===");
    od_log_debug("Pin configuration: MOSI=%u MISO=%u SCK=%u CS=%u WP=%u HOLD=%u",
                 mosiPin, misoPin, sckPin, csPin, wpPin, holdPin);
    od_log_debug("Configuring SPI pins...");
    pinMode(mosiPin, OUTPUT);
    pinMode(misoPin, INPUT);
    pinMode(sckPin, OUTPUT);
    pinMode(csPin, OUTPUT);
    pinMode(wpPin, OUTPUT);
    pinMode(holdPin, OUTPUT);
    od_log_debug("SPI pins configured");
    digitalWrite(sckPin, HIGH);  // Clock idle high (SPI mode 0)
    digitalWrite(csPin, HIGH);   // CS inactive
    digitalWrite(wpPin, HIGH);   // WP disabled (active-low)
    digitalWrite(holdPin, HIGH); // HOLD disabled (active-low)
    od_log_debug("Control pins set: CS=HIGH, WP=HIGH (disabled), HOLD=HIGH (disabled), SCK=HIGH (idle)");
    delay(1);
    od_log_debug("Attempting to wake flash from deep power-down (command 0xAB)...");
    digitalWrite(csPin, LOW);
    spiTransfer(0xAB);
    digitalWrite(csPin, HIGH);
    delay(10); // Wait for flash to wake up (typically 3-35us, using 10ms for safety)
    od_log_debug("Wake-up command sent, waiting 10ms...");
    od_log_debug("Reading JEDEC ID before power-down...");
    digitalWrite(csPin, LOW);
    spiTransfer(0x9F); // JEDEC ID command
    uint8_t jedecId[3];
    for (int i = 0; i < 3; i++) {
        jedecId[i] = spiTransfer(0x00);
    }
    digitalWrite(csPin, HIGH);
    od_log_debug("JEDEC ID before: 0x%02X%02X%02X (Manufacturer=0x%02X, MemoryType=0x%02X, Capacity=0x%02X)",
                 jedecId[0], jedecId[1], jedecId[2], jedecId[0], jedecId[1], jedecId[2]);
    delay(1);
    od_log_debug("Sending deep power-down command (0xB9)...");
    digitalWrite(csPin, LOW);
    spiTransfer(0xB9);
    digitalWrite(csPin, HIGH);
    if(false){
    od_log_debug("Deep power-down command sent, waiting 10ms...");
    delay(10); // Wait for command to complete
    od_log_debug("Reading JEDEC ID after power-down command...");
    digitalWrite(csPin, LOW);
    spiTransfer(0x9F);
    uint8_t jedecIdAfter[3];
    for (int i = 0; i < 3; i++) {
        jedecIdAfter[i] = spiTransfer(0x00);
    }
    digitalWrite(csPin, HIGH);
    od_log_debug("JEDEC ID after: 0x%02X%02X%02X (byte[0]=0x%02X, byte[1]=0x%02X, byte[2]=0x%02X)",
                 jedecIdAfter[0], jedecIdAfter[1], jedecIdAfter[2], jedecIdAfter[0], jedecIdAfter[1], jedecIdAfter[2]);
    }
    // CS/WP/HOLD are active-low: keep HIGH so the chip stays deselected and in deep sleep.
    // CLK/MOSI/MISO LOW — defined idle levels, no floating buffers on the MCU side.
    const uint8_t qspiLow[] = { mosiPin, misoPin, sckPin };
    for (uint8_t pin : qspiLow) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
    const uint8_t qspiHigh[] = { csPin, wpPin, holdPin };
    for (uint8_t pin : qspiHigh) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, HIGH);
    }
    #else
    od_log_warn("External flash power-down not implemented for ESP32");
    return false;
    #endif
    return false;
}
