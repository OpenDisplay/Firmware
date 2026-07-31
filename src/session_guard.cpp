// The shared session-teardown routine and the R3a link-down wait.
// See session_guard.h for the caller set and CONNECTION_POLICY R3a/R6 for the rules.

#include <Arduino.h>

#include "session_guard.h"
#include "ble_transport.h"
#include "command_queue.h"
#include "config_parser.h"
#include "display_service.h"
#include "encryption.h"
#include "link_owner.h"
#include "od_log.h"
#include "structs.h"
#include "touch_input.h"
#ifdef OPENDISPLAY_HAS_WIFI
#include "wifi_service.h"
#endif

extern bool directWriteActive;
extern chunked_write_state_t chunkedWriteState;

// How long to wait for a requested drop to actually take the link down.
//
// Sized against CLIENT BEHAVIOUR, not against a supervision timeout: an alive peer
// terminates within a few connection intervals -- tens of ms, since this firmware
// requests no interval and the central's negotiated value applies. A peer already
// gone is reaped by the link layer at ~4-6 s, far outside this bound, and that is
// deliberate: expiry here is not a failure needing recovery, just an early exit
// into an abort that runs regardless (R3a). This is the least load-bearing
// threshold in the freeze-hardening plan.
#ifndef OD_BLE_LINK_DOWN_WAIT_MS
#define OD_BLE_LINK_DOWN_WAIT_MS 150
#endif

// Tick granularity for the wait below. Deliberately a plain delay() rather than
// idleDelay(): see bleDropAndWait().
#ifndef OD_BLE_LINK_DOWN_POLL_MS
#define OD_BLE_LINK_DOWN_POLL_MS 2
#endif

bool bleDropAndWait(uint16_t handle, uint16_t epoch) {
    if (!ble.instanceLive(handle, epoch)) return true;   // already down
    ble.disconnect(handle, epoch);
    const uint32_t deadline = millis() + OD_BLE_LINK_DOWN_WAIT_MS;
    while ((int32_t)(millis() - deadline) < 0) {
        // Per-handle, so a refused contender still attached cannot mask the owner's
        // departure (the aggregate count would read 1 and never reach 0).
        if (!ble.instanceLive(handle, epoch)) return true;
        // A plain delay, NOT idleDelay(). idleDelay() early-outs on
        // ble.eventPending(), and mid-teardown there is always an event pending --
        // the owner's own disconnect, deliberately left unconsumed so
        // serviceBleEvents() still sees it -- so every call would return instantly
        // and this loop would degrade into a busy spin for its full bound. A plain
        // tick also services neither RX nor transport events, which is precisely the
        // safety property wanted here.
        delay(OD_BLE_LINK_DOWN_POLL_MS);
    }
    const bool down = !ble.instanceLive(handle, epoch);
    if (!down) {
        od_log_warn("Link-down wait expired for h=%u (%u ms); proceeding, stale link is inert",
                    (unsigned)handle, (unsigned)OD_BLE_LINK_DOWN_WAIT_MS);
    }
    return down;
}

void resetChunkedWriteState(void) {
    // A real primitive replacing the open-coded inline clears in communication.cpp.
    // The buffer itself is deliberately not zeroed: `active = false` makes it
    // unreachable, and MAX_CONFIG_SIZE is large enough that clearing it on every
    // teardown would be pointless work.
    chunkedWriteState.active = false;
    chunkedWriteState.totalSize = 0;
    chunkedWriteState.receivedSize = 0;
    chunkedWriteState.expectedChunks = 0;
    chunkedWriteState.receivedChunks = 0;
}

void abortToKnownState(const char* reason, bool dropLink, LinkId ownerId) {
    // 1. Log first, one line, so a teardown is always attributable even if a later
    //    step wedges.
    od_log_info("[abort] %s (dropLink=%d owner=%u/h%u/e%u)", reason ? reason : "?",
                dropLink ? 1 : 0, (unsigned)ownerId.who, (unsigned)ownerId.handle,
                (unsigned)ownerId.epoch);

    // 2. Optional client NACK: skipped when dropLink, since the link is about to go.
    //    Note the asymmetry with Phase 4's auth-abuse drop, which must DELIVER its
    //    final FE -- it runs its own bounded TX barrier BEFORE calling this, rather
    //    than asking the abort to hold the link open. Two contradictory jobs in one
    //    routine is what that split avoids.

    // 3-5. Transfer state. cleanupDirectWriteState forces power off only for a
    //      mid-transfer (PWR_ACTIVE) session and no-ops on WARM, which preserves the
    //      ACTIVE-only-teardown invariant: a WARM keep-alive panel SURVIVES an abort,
    //      including an idle or watchdog drop while the panel is warm from a prior
    //      push. epdSessionForceOff() is deliberately NOT called here -- it powers off
    //      every state except PWR_OFF, WARM included, so it would kill exactly the
    //      panel that must survive. Deep sleep calls it separately, from the sleep
    //      path, because no panel may stay powered through sleep.
    if (directWriteActive) cleanupDirectWriteState(true);
    cleanupPartialWriteOnDisconnect();
    resetPipeWriteState();

    // 6. Config chunked upload -- previously had no reset function at all, only
    //    open-coded inline assignments, so no teardown or watchdog touched it.
    resetChunkedWriteState();

    // 7. Touch: assert the suspend counter reaches 0 even when teardown bypassed
    //    cleanupDirectWriteState (which is the only place that used to clear
    //    directWriteTouchSuspended), so a partial-path teardown cannot leave touch
    //    suspended forever.
    touchForceResume();

    // 8. Crypto. NEW on the disconnect path: today crypto state survives a BLE link
    //    drop entirely -- clearEncryptionSession() runs on session-timeout-at-command,
    //    a new auth, config reload and LAN teardown, but no BLE disconnect path.
    clearEncryptionSession();

    // 9. Both rings. R6 requires RX and TX drained of the departed session's traffic;
    //    an earlier draft flushed TX only, which left the teardown window open.
    //    Draining RX is sound because callback filtering means every frame in it
    //    passed the owner check when written. A frame the owner writes AFTER this,
    //    during step 10's wait, is deliberately not re-flushed: it carries the
    //    departing instance's tag and fails the dispatch check once step 11 releases.
    const uint8_t droppedRx = bleRxQueueReset();
    if (droppedRx > 0) {
        od_log_warn("[abort] dropped %u queued command(s) from the departed session",
                    (unsigned)droppedRx);
    }
    bleTxQueueReset();

    // 10. The drop, dispatched on the OWNER'S TRANSPORT. This routine is not
    //     BLE-only: the transfer watchdog that calls it is origin-agnostic, so
    //     dropping a BLE handle for a timed-out LAN owner would leave the owning
    //     socket alive while its token was released -- an R1 violation.
    if (dropLink) {
        if (ownerId.who == OWNER_BLE) {
            (void)bleDropAndWait(ownerId.handle, ownerId.epoch);
        } else if (ownerId.who == OWNER_LAN) {
#ifdef OPENDISPLAY_HAS_WIFI
            // A TCP close is synchronous, so no wait bound applies on LAN -- the
            // asynchrony R3a exists for is BLE-only.
            wifiLanDropOwnedSocket();
#endif
        }
    }

    // 11. Release, STRICTLY after step 10. Releasing at request time (an earlier
    //     draft) would let a new connection be admitted while the old link was still
    //     physically up. If the wait expired the release still happens -- the stale
    //     link is inert by construction.
    //
    //     For the terminal caller this is naturally inert: the word was exchanged to
    //     OWNER_TERMINAL before the abort, so this full-identity CAS does not match
    //     and the admission gate stays shut.
    linkRelease(ownerId);
}

void abortToKnownState(const char* reason, bool dropLink) {
    abortToKnownState(reason, dropLink, linkOwnerId());
}
