// nRF52840 (Adafruit Bluefruit / SoftDevice S140) implementation of BleTransport.
//
// The whole file is gated on TARGET_NRF, so an ESP32 build compiles it to an
// empty translation unit -- no build_src_filter changes needed across the CI
// environments. Every Bluefruit object lives here as file-static state; nothing
// outside this file names a Bluefruit type.
#ifdef TARGET_NRF

#include "ble_transport.h"
#include "ble_transport_nrf.h"
#include "command_queue.h"
#include "link_owner.h"
#include "structs.h"
#include "encryption.h"
#include "od_log.h"

extern "C" {
#include "nrf_soc.h"
}

extern struct GlobalConfig globalConfig;
String getChipIdHex();

BleTransport ble;

// --- stack objects (were globals in main.h) ---------------------------------
static BLEDfu s_dfu;
static BLEService s_imageService("2446");
// max_len is a GATT-declared attribute length the SoftDevice reserves for real
// (vloc = BLE_GATTS_VLOC_STACK), so 512 cost 512 B of attribute table while the
// link caps at ATT MTU 247 (payload 244) -- half of it was unreachable.
static BLECharacteristic s_imageCharacteristic(
    "2446", BLEWrite | BLEWriteWithoutResponse | BLENotify, OD_BLE_MAX_FRAME);

static bool     s_begun = false;
static uint16_t s_connHandle = BLE_CONN_HANDLE_INVALID;
static volatile bool s_connectedEvent = false;
static volatile bool s_disconnectedEvent = false;
// uint16_t for signature parity with ESP32, where NimBLE's reason genuinely needs
// the width. The SoftDevice hands us a raw HCI byte with no wrapping, so nothing
// is lost or aliased here -- but the two targets now log the same way.
static volatile uint16_t s_disconnectReason = 0;
static volatile uint32_t s_connectedWord = 0;
static volatile uint32_t s_disconnectedWord = 0;

// --- the instance table (CONNECTION_POLICY R3 requirement 5) -----------------
// Degenerate at one entry: Bluefruit.begin(1, 0) configures the SoftDevice for a
// single peripheral link, so cross-central injection is unreachable at the link
// layer. It exists anyway because the mechanisms above it are portable -- the R3a
// wait polls per-handle liveness, and frames carry identity tags on both targets.
// A single-link target still queues frames that can outlive their session across a
// disconnect/reconnect pair inside one blocked-loop window, which is precisely
// what the tag protects against.
static volatile uint32_t s_instanceWord = 0;
static volatile bool     s_instanceSubscribed = false;
// Claim disposition: 0 while the claim is in flight, otherwise the identity word
// the claim was decided for. See the matching field in ble_transport_esp32.cpp for
// why it carries the identity rather than a bare flag.
static volatile uint32_t s_instanceDecidedWord = 0;

// --- advertising interval policy --------------------------------------------
static uint32_t s_advBoostUntil = 0;

static constexpr uint16_t NRF_ADV_INTERVAL_MIN = 256;   // 160 ms
static constexpr uint16_t NRF_ADV_INTERVAL_MAX = 1600;  // 1000 ms
static constexpr uint16_t NRF_ADV_BOOST_MIN = 32;       // 20 ms
static constexpr uint16_t NRF_ADV_BOOST_MAX = 48;       // 30 ms
static constexpr uint32_t NRF_ADV_BOOST_MS = 3000;

static void applyAdvInterval() {
    if (s_advBoostUntil != 0 && millis() < s_advBoostUntil) {
        Bluefruit.Advertising.setInterval(NRF_ADV_BOOST_MIN, NRF_ADV_BOOST_MAX);
    } else {
        s_advBoostUntil = 0;
        Bluefruit.Advertising.setInterval(NRF_ADV_INTERVAL_MIN, NRF_ADV_INTERVAL_MAX);
    }
}

// --- link-layer diagnostics (implementation-private) ------------------------
// DLE (Data Length Extension) sets the max Link-Layer PDU payload: 27 octets by
// default, up to 251 once negotiated. The nRF peripheral only auto-accepts the
// central's request, which arrives AFTER the connect callback, so we log twice:
// once at connect (baseline) and once ~2.5 s later (negotiated).
// `atInfo` separates the two callers: the pre-negotiation baseline is diagnostic
// noise and stays at DEBUG, while the negotiated result -- the answer to "did the
// 2M PHY and 251-octet DLE actually get granted" -- logs at INFO so it survives a
// default-level bench capture. Mirrors logNegotiatedLink() on ESP32.
static void logLinkParams(uint16_t conn_handle, const char* phase, bool atInfo) {
    BLEConnection* conn = Bluefruit.Connection(conn_handle);
    if (conn == nullptr) {
        od_log_debug("[LINK %s] no connection (handle %u)", phase, conn_handle);
        return;
    }
    uint8_t  phy = conn->getPHY();
    uint16_t mtu = conn->getMtu();                // ATT MTU (23 default; 247 cap here)
    uint16_t dle = conn->getDataLength();         // LL PDU payload octets (27 default; 251 max)
    uint16_t ci  = conn->getConnectionInterval(); // units of 1.25 ms
    const char* phyStr = (phy == BLE_GAP_PHY_2MBPS) ? "2M" :
                         (phy == BLE_GAP_PHY_1MBPS) ? "1M" : "?";
    char line[128];
    snprintf(line, sizeof(line),
             "[LINK %s] PHY=%s  ATT_MTU=%u  DLE=%u octets  connInterval=%.2f ms",
             phase, phyStr, mtu, dle, ci * 1.25f);
    if (atInfo) {
        od_log_info("%s", line);
    } else {
        od_log_debug("%s", line);
    }
}

// One-shot timer (armed only on connect -- no per-loop polling). Fires once on
// the FreeRTOS timer task after the central finishes negotiation. This is a
// third execution context; it only logs, which is why it is safe here.
static SoftwareTimer s_linkDiagTimer;
static uint16_t      s_linkDiagConn = BLE_CONN_HANDLE_INVALID;

static void linkDiagCallback(TimerHandle_t /*xTimer*/) {
    if (Bluefruit.connected()) {
        logLinkParams(s_linkDiagConn, "negotiated", true);   // INFO: the granted result
    }
}

static void armLinkDiag(uint16_t conn_handle) {
    s_linkDiagConn = conn_handle;
    static bool created = false;
    if (!created) {
        // Create the one-shot (repeating=false) on the first connection only.
        s_linkDiagTimer.begin(500, linkDiagCallback, NULL, false);
        created = true;
    }
    s_linkDiagTimer.reset();   // start/restart the one-shot; fires ~2.5 s later
}

// --- stack callbacks (SoftDevice callback task) ------------------------------
// The threading contract, matching ESP32: a stack callback may copy bytes into the
// RX ring, publish its own instance metadata, attempt the single ownership claim
// CAS, and set an event flag. Everything else -- command dispatch, zlib inflate,
// EPD SPI streaming, notify(), the connect/disconnect application work, even the
// PHY/DLE request -- runs on the loop() task. Anything added below beyond that set
// reintroduces the cross-task races this design exists to remove.
static void onConnectCb(uint16_t conn_handle) {
    s_connHandle = conn_handle;
    // Same normative order as ESP32 (R2): epoch, then table entry, then claim.
    const uint16_t epoch = linkNextEpoch();
    const uint32_t word = linkPackWord(OWNER_BLE, conn_handle, epoch);
    __atomic_store_n(&s_instanceDecidedWord, (uint32_t)0, __ATOMIC_RELAXED);
    __atomic_store_n(&s_instanceWord, word, __ATOMIC_RELEASE);
    s_instanceSubscribed = false;
    const LinkId id = { OWNER_BLE, conn_handle, epoch };
    const bool admitted = linkClaim(id);
    // Publish the identity the claim resolved FOR, so the loop-side refusal scan
    // can bind the disposition to this exact instance.
    __atomic_store_n(&s_instanceDecidedWord, word, __ATOMIC_RELEASE);
    od_log_info("=== BLE CLIENT CONNECTED (nRF) h=%u e=%u %s ===",
                (unsigned)conn_handle, (unsigned)epoch,
                admitted ? "[owner]" : "[contender - refused service]");
    // Payload before flag, flag RELEASE-stored: see the ESP32 twin.
    __atomic_store_n(&s_connectedWord, word, __ATOMIC_RELAXED);
    __atomic_store_n(&s_connectedEvent, true, __ATOMIC_RELEASE);
}

static void onDisconnectCb(uint16_t conn_handle, uint8_t reason) {
    (void)conn_handle;
    od_log_info("=== BLE CLIENT DISCONNECTED (nRF) reason=0x%03X ===", (unsigned)reason);
    s_connHandle = BLE_CONN_HANDLE_INVALID;
    __atomic_store_n(&s_disconnectReason, (uint16_t)reason, __ATOMIC_RELAXED);
    const uint32_t word = __atomic_load_n(&s_instanceWord, __ATOMIC_ACQUIRE);
    __atomic_store_n(&s_disconnectedWord, word, __ATOMIC_RELAXED);
    // Retire the entry: this is what makes link death observable per handle, for
    // the R3a wait and for the loop's owner comparison. No RX-boundary capture --
    // frames carry their writer's identity instead (CommandQueueItem::tag).
    s_instanceSubscribed = false;
    __atomic_store_n(&s_instanceDecidedWord, (uint32_t)0, __ATOMIC_RELAXED);
    __atomic_store_n(&s_instanceWord, (uint32_t)0, __ATOMIC_RELEASE);
    // The token is deliberately NOT released here. An intermediate version did
    // release it on this callback, to let a fast reconnect win a fresh claim -- but
    // that admits a new owner while the departed session's transfer, crypto and TX
    // ring are still live, and RX/TX are serviced before the deferred cleanup, so
    // the new client's commands would run against the old session's state and its
    // queued responses could be delivered to the newcomer. Release stays where R3a
    // puts it: the last step of the abort, after teardown.
    //
    // What makes that safe for a reconnecting client is contender refusal
    // (serviceContenderRefusal in main.cpp): a client that reconnects into a
    // still-held slot is disconnected once the loop runs, and its NEXT connect
    // claims cleanly. Without refusal the client would sit on nRF's only
    // peripheral link forever with every write filtered, since admission is decided
    // once per instance and never revisited.
    __atomic_store_n(&s_disconnectedEvent, true, __ATOMIC_RELEASE);
}

// Adapter: Bluefruit's write_callback_t is BLECharacteristic*-shaped, whereas the
// shared dispatcher takes an opaque pointer (it ignores both leading arguments on
// every target). Adapting here is what keeps Bluefruit types out of
// communication.cpp.
static void onWriteCb(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
    (void)chr;
    // Same owner filter as ESP32, so the two targets read identically. Latent
    // rather than live here -- the SoftDevice permits only one peripheral link, so
    // there is no second central to filter out -- but the TAG it produces is not
    // latent at all: it is what stops a frame queued by a departed session from
    // dispatching into the next one.
    //
    // This callback used to (void) the handle it was given.
    const uint32_t word = __atomic_load_n(&s_instanceWord, __ATOMIC_ACQUIRE);
    if (word == 0 || linkUnpackWord(word).handle != conn_hdl || word != linkOwnerWord()) {
        od_log_debug("Dropped write from non-owner h=%u", (unsigned)conn_hdl);
        return;
    }
    // bleRxQueuePush() owns the arrival log and every drop reason (empty, too large,
    // ring full) so this callback and ESP32's onWrite() cannot report the same frame
    // differently. This site used to print "queue full" for all three, sending you
    // after ring depth when the real cause was a malformed frame. Add no logging here.
    (void)bleRxQueuePush(data, len, word);
}

// --- BleTransport ------------------------------------------------------------
bool BleTransport::begin(const char* deviceName) {
    Bluefruit.configCentralBandwidth(BANDWIDTH_MAX);
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
    Bluefruit.autoConnLed(false);
    Bluefruit.setTxPower(globalConfig.power_option.tx_power);
    Bluefruit.begin(1, 0);
    od_log_info("BLE initialized successfully");
    od_log_info("Setting up BLE service 0x2446...");
    s_imageService.begin();
    od_log_info("BLE service started");
    s_imageCharacteristic.setWriteCallback(onWriteCb);
    od_log_info("BLE write callback set");
    s_imageCharacteristic.begin();
    od_log_info("BLE characteristic started");
    // Register the DFU service LAST so its presence/absence (it is only added when
    // encryption is disabled) never shifts the handles of s_imageCharacteristic and
    // its CCCD. GATT handles are assigned in begin() order; keeping the app
    // characteristic ahead of the conditional DFU service keeps its handles stable
    // across encryption on/off, so a client's cached CCCD handle stays valid and
    // notify setup won't fail with ATT "Invalid handle". Must stay after
    // Bluefruit.begin() (SoftDevice up first).
    if (!isEncryptionEnabled()) {
        s_dfu.begin();
        od_log_info("BLE DFU initialized successfully (encryption disabled)");
    } else {
        od_log_info("BLE DFU service NOT initialized (encryption enabled - use CMD_ENTER_DFU)");
    }
    Bluefruit.Periph.setConnectCallback(onConnectCb);
    Bluefruit.Periph.setDisconnectCallback(onDisconnectCb);
    od_log_info("BLE callbacks registered");
    Bluefruit.setName(deviceName);
    od_log_info("Device name set to: %s", deviceName);
    od_log_info("Configuring power management...");
    sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
    sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
    od_log_info("Power management configured");
    s_begun = true;
    return true;
}

void BleTransport::startAdvertising() {
    od_log_info("Configuring BLE advertising...");
    Bluefruit.Advertising.clearData();
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addName();
    // Deliberately kept inside this sequence rather than hoisted to the caller:
    // updatemsdata() lands in setManufacturerData() below, which itself sets
    // setFastTimeout(1). Calling it after the setFastTimeout(10) line would leave
    // the fast-advertising window at 1 s instead of 10 s -- a real behaviour
    // change. Phase 1 keeps the historical order byte-for-byte.
    updatemsdata();
    Bluefruit.Advertising.restartOnDisconnect(true);
    applyAdvInterval();
    Bluefruit.Advertising.setFastTimeout(10);
    od_log_info("Starting BLE advertising...");
    Bluefruit.Advertising.start(0);
}

void BleTransport::restartAdvertising() {
    Bluefruit.Advertising.stop();
    Bluefruit.Advertising.start(0);
}

void BleTransport::stopAdvertising() {
    Bluefruit.Advertising.stop();
}

void BleTransport::end() {
    // No-op: the SoftDevice stays up for the life of the nRF firmware. Only the
    // ESP32 tears the controller down (deep sleep / pre-restart).
}

bool BleTransport::isReady() const {
    return s_begun;
}

uint8_t BleTransport::connectedCount() const {
    // Bluefruit.connected() returns the number of active connections; the
    // peripheral is configured for a single link (Bluefruit.begin(1, 0)).
    return (uint8_t)Bluefruit.connected();
}

bool BleTransport::notifyReady() const {
    // Gated on ownership as well as CCCD, matching ESP32: with the slot unowned
    // there is nobody a response may legitimately go to.
    if (!Bluefruit.connected() || !s_imageCharacteristic.notifyEnabled()) return false;
    const uint32_t owner = linkOwnerWord();
    return owner != 0 && owner == __atomic_load_n(&s_instanceWord, __ATOMIC_ACQUIRE);
}

bool BleTransport::notify(const uint8_t* data, uint16_t len) {
    // Single-link by construction (Bluefruit.begin(1, 0)), so there is no
    // all-subscribers overload to avoid as there is on NimBLE -- but the ownership
    // gate is kept so both targets refuse to notify an unowned or foreign link.
    const uint32_t owner = linkOwnerWord();
    if (owner == 0 || owner != __atomic_load_n(&s_instanceWord, __ATOMIC_ACQUIRE)) return false;
    return s_imageCharacteristic.notify(data, len);
}

bool BleTransport::instanceLive(uint16_t handle, uint16_t epoch) const {
    return __atomic_load_n(&s_instanceWord, __ATOMIC_ACQUIRE) ==
           linkPackWord(OWNER_BLE, handle, epoch);
}

uint8_t BleTransport::liveInstanceCount() const {
    return __atomic_load_n(&s_instanceWord, __ATOMIC_ACQUIRE) != 0 ? 1 : 0;
}

uint32_t BleTransport::instanceWordAt(uint8_t index) const {
    if (index != 0) return 0;
    return __atomic_load_n(&s_instanceWord, __ATOMIC_ACQUIRE);
}

uint32_t BleTransport::instanceClaimDecidedWordAt(uint8_t index) const {
    if (index != 0) return 0;
    return __atomic_load_n(&s_instanceDecidedWord, __ATOMIC_ACQUIRE);
}

uint8_t BleTransport::instanceCapacity() { return 1; }

bool BleTransport::disconnect(uint16_t handle, uint16_t epoch) {
    // Re-validate as late as possible; see the ESP32 twin and the header note.
    if (!instanceLive(handle, epoch)) return true;   // already gone, or reassigned
    // Bluefruit takes ONLY a handle: BLEConnection::disconnect() calls
    // sd_ble_gap_disconnect(_conn_hdl, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION),
    // so 0x13 is sent and there is no reason argument to pass -- which is why the
    // seam exposes none.
    //
    // Unlike the DFU-entry disconnect in device_control.cpp, restartOnDisconnect
    // is deliberately left ON: this drop frees the slot for the next client rather
    // than ending the device's session life.
    if (!Bluefruit.connected()) return true;   // already gone
    const bool ok = Bluefruit.disconnect(handle);
    if (!ok) od_log_warn("WARNING: BLE disconnect request failed for handle %u", (unsigned)handle);
    return ok;
}

uint16_t BleTransport::connIntervalMs(uint16_t handle) const {
    BLEConnection* conn = Bluefruit.Connection(handle);
    if (conn == nullptr) return 0;
    const uint16_t units = conn->getConnectionInterval();   // 1.25 ms units
    if (units == 0) return 0;
    return (uint16_t)((units * 5 + 3) / 4);
}

void BleTransport::setManufacturerData(const uint8_t* msd, uint8_t len) {
    Bluefruit.Advertising.clearData();
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addName();
    Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, msd, len);
    applyAdvInterval();
    Bluefruit.Advertising.setFastTimeout(1);
    Bluefruit.Advertising.stop();
    Bluefruit.Advertising.start(0);
}

// Proactively upgrade the link for throughput: the nRF peripheral only
// auto-accepts the central's PHY/DLE requests, so if the phone never asks we
// stay at 1M / 27 octets. Both requests are no-ops if the peer already
// negotiated the same or better.
//
// Called from loop() when the connect event is consumed, NOT from the connect
// callback: these are SoftDevice calls, which the Phase 3 callback contract
// ("copy bytes, set a flag") excludes. The few milliseconds of delay cost
// nothing -- the central's own request arrives later than this either way, which
// is why the diagnostics below log twice.
void BleTransport::requestFastLink() {
    BLEConnection* conn = Bluefruit.Connection(s_connHandle);
    if (conn == nullptr) return;

    logLinkParams(s_connHandle, "at connect", false);   // DEBUG: baseline, pre-negotiation

    // 2 Mbps PHY (tx + rx). Peer may decline and stay at 1M.
    conn->requestPHY(BLE_GAP_PHY_2MBPS);

    // 251-octet Link-Layer PDUs (max DLE). AUTO time lets the controller derive
    // the PHY-appropriate on-air duration.
    ble_gap_data_length_params_t dl;
    dl.max_tx_octets  = 251;
    dl.max_rx_octets  = 251;
    dl.max_tx_time_us = BLE_GAP_DATA_LENGTH_AUTO;
    dl.max_rx_time_us = BLE_GAP_DATA_LENGTH_AUTO;
    ble_gap_data_length_limitation_t limit = { 0, 0, 0 };
    if (!conn->requestDataLengthUpdate(&dl, &limit)) {
        od_log_warn("DLE 251 request rejected (tx_lim=%u rx_lim=%u time_lim_us=%u)",
                    limit.tx_payload_limited_octets, limit.rx_payload_limited_octets,
                    limit.tx_rx_time_limited_us);
    }
    od_log_debug("Requested fast link: 2M PHY + 251-octet DLE");
    armLinkDiag(s_connHandle);                   // re-log once negotiation settles
}

void BleTransport::boostAdvertising() {
    s_advBoostUntil = millis() + NRF_ADV_BOOST_MS;
}

void BleTransport::tick() {
    static bool was_boosted = false;
    const bool boosting = (s_advBoostUntil != 0 && millis() < s_advBoostUntil);
    if (boosting) {
        was_boosted = true;
        return;
    }
    if (!was_boosted || !Bluefruit.Advertising.isRunning()) {
        was_boosted = false;
        s_advBoostUntil = 0;
        return;
    }
    was_boosted = false;
    s_advBoostUntil = 0;
    Bluefruit.Advertising.setInterval(NRF_ADV_INTERVAL_MIN, NRF_ADV_INTERVAL_MAX);
    Bluefruit.Advertising.stop();
    Bluefruit.Advertising.start(0);
}

bool BleTransport::eventPending() const {
    // RELAXED: a non-destructive peek used to decide whether to return to loop(),
    // never to establish ordering. Reading one pass stale is harmless -- the next
    // pass sees it.
    return __atomic_load_n(&s_connectedEvent, __ATOMIC_RELAXED) ||
           __atomic_load_n(&s_disconnectedEvent, __ATOMIC_RELAXED);
}

bool BleTransport::takeConnectedEvent(uint32_t* instanceWord) {
    // Atomic exchange, not check-then-clear. The old form could lose an event that
    // arrived inside the gap, and -- now that the flag carries an identity payload
    // -- could also lose an event that arrived inside the gap. ACQUIRE pairs with
    // the callback's RELEASE store of the flag, which it makes after writing the
    // payload, so the payload we read is at least fully written.
    //
    // It does NOT bind the payload to the flag we just consumed: acquire/release
    // orders writes that PRECEDE the release, and nothing freezes the payload
    // afterwards, so a second connect landing between the exchange and the load
    // below hands us ITS word instead. That is tolerable only because no decision
    // depends on it -- teardown and connect-side work are both derived from the
    // owner token and the instance table, which are authoritative. The payload is
    // diagnostic. Do not build a decision on it without binding it properly.
    if (!__atomic_exchange_n(&s_connectedEvent, false, __ATOMIC_ACQUIRE)) return false;
    if (instanceWord != nullptr) {
        *instanceWord = __atomic_load_n(&s_connectedWord, __ATOMIC_RELAXED);
    }
    return true;
}

bool BleTransport::takeDisconnectedEvent(uint16_t* reason, uint32_t* instanceWord) {
    // See takeConnectedEvent(), including the caveat: the exchange stops events
    // being lost in the old check-then-clear gap, but does not bind this payload to
    // the flag just consumed. The reason code below is therefore diagnostic -- a
    // burst of disconnects can report the latest reason twice. Teardown decides on
    // table state, not on this.
    if (!__atomic_exchange_n(&s_disconnectedEvent, false, __ATOMIC_ACQUIRE)) return false;
    if (reason != nullptr) *reason = __atomic_load_n(&s_disconnectReason, __ATOMIC_RELAXED);
    if (instanceWord != nullptr) {
        *instanceWord = __atomic_load_n(&s_disconnectedWord, __ATOMIC_RELAXED);
    }
    return true;
}

bool BleTransport::restartsAdvertisingOnDisconnect() const {
    // Bluefruit.Advertising.restartOnDisconnect(true) in startAdvertising(): the
    // SoftDevice re-arms the radio itself, so the application must not also
    // schedule a restart or the two fight over the advertising state.
    return true;
}

const char* BleTransport::addressString() {
    // Lowercase colon-separated, matching the ESP32 implementation's contract.
    //
    // Byte order matters: Bluefruit::getAddr(mac) memcpy's ble_gap_addr_t.addr
    // straight out of sd_ble_gap_addr_get(), and the SoftDevice stores that
    // LSB-first (bluefruit.cpp:508-515). The advertised AdvA and every
    // human-readable form are MSB-first, so emit it reversed -- otherwise this
    // returns a byte-swapped address that looks plausible and matches nothing.
    static char s_addr[18];
    uint8_t mac[6] = {0};
    (void)Bluefruit.getAddr(mac);   // return value is the address TYPE, not a status
    snprintf(s_addr, sizeof(s_addr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
    return s_addr;
}

#endif  // TARGET_NRF
