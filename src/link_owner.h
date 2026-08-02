#ifndef LINK_OWNER_H
#define LINK_OWNER_H

#include <stdint.h>

// Connection ownership arbiter -- CONNECTION_POLICY R1/R2, and the activity clock
// of R4. Deliberately free of BLE/WiFi headers: the BLE transports, the LAN
// transport and the session guard all include it, and it must not drag a stack
// header into any of them.
//
// THE TOKEN IS ONE 32-BIT WORD, and that is the whole design:
//
//     [31:30] transport   [29:16] handle   [15:0] epoch
//
// A claim is one compare-and-swap against it; a read is one atomic load. That is
// what lets a stack callback -- which runs on the host task, long before any loop
// pass -- decide "is this write from the owner?" without a lock. An earlier draft
// made the token loop-task-only plain state while still requiring callback-side
// write filtering; those are incompatible, because at the moment a contender's
// onWrite fires there is nothing for the filter to compare against.
//
// Why 16 bits of epoch: the word must stay lock-free, and neither Cortex-M4 nor
// the ESP32 ISAs have a lock-free 64-bit CAS. HCI connection handles are
// spec-bounded at 0x0EFF (12 bits), so 14 bits holds them with headroom. The
// invariant the width rests on is that no outstanding event may survive a full
// counter cycle: epochs churn at link-layer connection rate (tens of ms each), so
// a 2^16 cycle needs ~half an hour of continuous connect churn inside a single
// blocking window that later COMPLETES. A refresh that never completes never
// resumes loop(), so nothing is consumed there and a collision has no consumer to
// mislead.
//
// All-zero means unowned, so epoch 0 is never allocated (linkNextEpoch re-draws).

enum LinkOwnerKind {
    OWNER_NONE     = 0,
    OWNER_BLE      = 1,
    OWNER_LAN      = 2,
    // One-way admission gate for a terminal transition (deep sleep). Claims fail
    // against it, so no connection can be admitted between the abort's release and
    // the stack teardown that follows. Nothing transitions out of it: the next wake
    // reloads RAM. See CONNECTION_POLICY 7e row 3.
    OWNER_TERMINAL = 3,
};

struct LinkId {
    uint8_t  who;      // LinkOwnerKind
    uint16_t handle;   // BLE conn handle; LAN is single-socket and uses 0
    uint16_t epoch;
};

#define OD_LINK_WORD_TERMINAL 0xC0000000UL

static inline uint32_t linkPackWord(uint8_t who, uint16_t handle, uint16_t epoch) {
    return ((uint32_t)(who & 0x3) << 30) | ((uint32_t)(handle & 0x3FFF) << 16) | (uint32_t)epoch;
}

static inline LinkId linkUnpackWord(uint32_t w) {
    LinkId id;
    id.who    = (uint8_t)((w >> 30) & 0x3);
    id.handle = (uint16_t)((w >> 16) & 0x3FFF);
    id.epoch  = (uint16_t)(w & 0xFFFF);
    return id;
}

static inline uint32_t linkIdWord(LinkId id) { return linkPackWord(id.who, id.handle, id.epoch); }

// Allocated in the connect callback for EVERY connection instance, admitted or
// not -- never on successful claim. A refused contender that carried no epoch
// could not be told apart from an incumbent that reused its handle (7a row 4),
// and the identity is what the admission decision is MADE on. Atomic because BLE
// allocates on the stack callback task and LAN on the loop task.
uint16_t linkNextEpoch(void);

// One CAS. Succeeds only against the all-zero (unowned) word, so it is safe to
// call from a stack callback and it is where admission actually happens. Failure
// means "someone else owns the slot" -- the caller is a contender.
bool linkClaim(LinkId id);

// CAS holder -> unowned. Loop task only, and only after R3a's link-down wait.
// Matches the FULL identity, so a stale-epoch release is inert, and it can never
// zero the terminal word (which is not a valid `id` to pass here).
void linkRelease(LinkId id);

// Unconditional exchange to OWNER_TERMINAL, returning the identity it displaced.
// The caller hands that identity to abortToKnownState(): after this returns,
// linkOwnerId() reads terminal, NOT the departing owner, so the abort must not
// re-derive it. Deep-sleep path only.
LinkId linkMarkTerminal(void);

// One atomic load; callable from ANY task, which is the point.
LinkId linkOwnerId(void);
uint32_t linkOwnerWord(void);

// Full-triple comparison. A handle alone is never sufficient: NimBLE reuses
// handles from 0 upward, so a reconnect can be handed the same one.
bool linkIsOwner(LinkId id);
static inline bool linkIsOwnerWord(uint32_t w) { return w != 0 && w == linkOwnerWord(); }

// --- activity clock (CONNECTION_POLICY R4) -----------------------------------
// "Idle" is: no inbound command from the owner on the owning transport, AND no
// refresh in progress.
//
// NOT loop-task-only, despite what the policy's phrasing suggests: the command and
// refresh stamps are, but ADMISSION stamps the baseline too, and that runs in the
// BLE connect callback on the stack host task. So the state is atomic, and the
// baseline carries the owner word it belongs to -- otherwise a reader can pair a
// newly published owner with the previous owner's baseline and judge a client that
// just connected to have been silent for minutes.
//
// The baseline is the LATER of admission, last recognised owner command, and last
// refresh end. Admission is included so a freshly admitted, still-silent client
// gets a full window before its first command instead of reading as infinitely
// idle. Refresh end is included because loop() does not run for a refresh's
// duration while wall-clock time passes, so a naive millis()-lastStamp would
// accrue the whole refresh and drop an actively engaged client the instant loop()
// resumes.
uint32_t linkMsSinceOwnerCommand(void);   // 0 when unowned
// Restart the idle window for the current owner. Named for its main caller -- the
// dispatcher, on a recognised command from the owner -- but it is also what starts
// the window at a point the policy defines as a fresh baseline rather than as
// activity: LAN calls it at TLS handshake COMPLETION, because handshake traffic is
// not a command and the window must not run from TCP accept.
void     linkStampOwnerCommand(void);
void     linkStampRefreshEnd(void);       // endRefresh(), both bracket sites

#endif  // LINK_OWNER_H
