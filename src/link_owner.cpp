// Connection ownership arbiter. See link_owner.h for the word layout and why the
// token has to be a single atomic word rather than loop-task-only state.

#include <Arduino.h>

#include "link_owner.h"
#include "od_log.h"

// The token. Written by CAS from either the stack callback task (BLE connect) or
// the loop task (LAN accept, release), read from both.
static volatile uint32_t s_ownerWord = 0;

// Epoch source. Fetch-add rather than ++ because BLE allocates on the callback
// task and LAN on the loop task.
static volatile uint16_t s_epochCounter = 0;

// Activity clock.
//
// ONE baseline, not three timestamps compared with a max(). The policy defines the
// baseline as "the later of admission, last recognised command, and last refresh
// end" -- but each of those events stamps the clock AT the moment it happens, so
// storing millis() into a single variable at each of the three sites already IS
// their maximum, and it stays correct across the ~49.7-day millis() wrap.
//
// A three-timestamp max() is not: comparing stamps pairwise needs signed
// difference arithmetic, which is only meaningful while the values are within 2^31
// of each other. An admission stamp left far behind a recent command stamp
// straddles that and the comparison inverts, pinning the baseline to the OLDEST
// stamp and reporting a ~49-day silence. Caught by tools/test_link_owner.cpp's
// wrap case; the single baseline removes the class of bug rather than patching the
// comparison.
//
// Atomic, not plain. The command and refresh stamps are loop-task-only as the
// policy says, but ADMISSION is not: linkClaim() runs in the BLE connect callback,
// on the stack host task. So the variable genuinely has two writing contexts and a
// plain global would be a data race regardless of how benign the values look.
static volatile uint32_t s_baselineMs = 0;

// Which owner the baseline above belongs to. The baseline cannot be published
// atomically WITH the owner word (two words, no lock-free 64-bit CAS here), and the
// gap is not theoretical: linkClaim's CAS publishes the new owner first, so between
// it and the baseline store a reader sees the NEW owner beside the PREVIOUS owner's
// baseline and computes an arbitrarily large silence -- which Phase 3 would act on
// by dropping a client that just connected.
//
// Re-reading the owner word around the load does not fix that, because both reads
// see the same (new) owner. Tagging does: the claimer stores the baseline, THEN
// release-stores this tag, so a reader whose tag does not match the live owner
// knows the baseline is not yet its own and reports 0 -- "just admitted", which is
// both true and the safe direction.
static volatile uint32_t s_baselineOwner = 0;

uint16_t linkNextEpoch(void) {
    uint16_t e;
    do {
        e = __atomic_add_fetch(&s_epochCounter, 1, __ATOMIC_RELAXED);
    } while (e == 0);   // 0 is reserved: the all-zero word means unowned
    return e;
}

bool linkClaim(LinkId id) {
    if (id.who != OWNER_BLE && id.who != OWNER_LAN) return false;
    uint32_t expected = 0;                      // succeeds ONLY against unowned
    const uint32_t desired = linkIdWord(id);
    const bool won = __atomic_compare_exchange_n(&s_ownerWord, &expected, desired,
                                                 false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
    if (won) {
        // Admission starts the idle window (R4): a freshly admitted client gets a
        // full window before its first command, rather than reading as infinitely
        // idle the instant it connects.
        //
        // Baseline first, then the tag that claims it for this owner. A reader that
        // catches the gap sees a tag that does not match the live owner and reports
        // 0 rather than the previous owner's stale baseline.
        __atomic_store_n(&s_baselineMs, millis(), __ATOMIC_RELAXED);
        __atomic_store_n(&s_baselineOwner, desired, __ATOMIC_RELEASE);
    }
    return won;
}

void linkRelease(LinkId id) {
    if (id.who == OWNER_NONE || id.who == OWNER_TERMINAL) return;
    uint32_t expected = linkIdWord(id);
    // Full-identity CAS: a release carrying a stale epoch, a foreign transport, or
    // the identity the terminal gate displaced simply fails and is inert.
    (void)__atomic_compare_exchange_n(&s_ownerWord, &expected, (uint32_t)0,
                                      false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
}

LinkId linkMarkTerminal(void) {
    const uint32_t prev = __atomic_exchange_n(&s_ownerWord, (uint32_t)OD_LINK_WORD_TERMINAL,
                                              __ATOMIC_ACQ_REL);
    return linkUnpackWord(prev);
}

uint32_t linkOwnerWord(void) {
    return __atomic_load_n(&s_ownerWord, __ATOMIC_ACQUIRE);
}

LinkId linkOwnerId(void) {
    return linkUnpackWord(linkOwnerWord());
}

bool linkIsOwner(LinkId id) {
    if (id.who == OWNER_NONE) return false;
    return linkIdWord(id) == linkOwnerWord();
}

// --- activity clock ----------------------------------------------------------
uint32_t linkMsSinceOwnerCommand(void) {
    const uint32_t owner = linkOwnerWord();
    const LinkId id = linkUnpackWord(owner);
    if (id.who != OWNER_BLE && id.who != OWNER_LAN) return 0;
    // Acquire the tag BEFORE the baseline it guards: if the tag already names this
    // owner, the release-store that published it also published the baseline.
    if (__atomic_load_n(&s_baselineOwner, __ATOMIC_ACQUIRE) != owner) {
        return 0;   // claim still in flight: this owner has no baseline yet
    }
    const uint32_t base = __atomic_load_n(&s_baselineMs, __ATOMIC_RELAXED);
    // Re-check that the owner did not change under us; if it did, `base` may belong
    // to a different session and 0 is the right answer for the one that just took
    // the slot.
    if (linkOwnerWord() != owner) return 0;
    // One unsigned subtraction, which is wrap-correct by construction: modular
    // arithmetic gives the true elapsed interval for any gap under 2^32 ms.
    return millis() - base;
}

void linkStampOwnerCommand(void) {
    // Loop-task-only, and only ever for the live owner, so the tag is already this
    // owner's -- but stamp it the same way regardless, so the two writers publish
    // through one discipline.
    __atomic_store_n(&s_baselineMs, millis(), __ATOMIC_RELAXED);
    __atomic_store_n(&s_baselineOwner, linkOwnerWord(), __ATOMIC_RELEASE);
}

void linkStampRefreshEnd(void) {
    // Unconditional by design: because each stamp writes the CURRENT time, and time
    // only moves forward, re-stamping can only ever DELAY a drop and never cause a
    // spurious one -- which is what makes it safe to apply at the transition
    // without first testing who owns the slot.
    __atomic_store_n(&s_baselineMs, millis(), __ATOMIC_RELAXED);
    __atomic_store_n(&s_baselineOwner, linkOwnerWord(), __ATOMIC_RELEASE);
}
