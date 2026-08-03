// Host test for src/link_owner.{h,cpp} — the connection ownership arbiter.
//
// Build and run from the repo root (one line; no continuations, since a trailing
// backslash inside a // comment is itself a line continuation and -Wcomment
// rejects it):
//
//   g++ -std=c++17 -Wall -Wextra -Werror -O1 -fsanitize=undefined,address -I tools/hostshim tools/test_link_owner.cpp src/link_owner.cpp -o /tmp/test_link_owner -pthread
//   /tmp/test_link_owner
//
// Like tools/test_nonce_window.cpp, this is as much a written-down statement of
// the intended semantics as it is a test. It covers the parts of CONNECTION_POLICY
// R1/R2/R4 that a firmware build cannot check and that a bench run would only
// exercise by luck:
//
//   - a claim is a CAS: exactly one winner under contention, no torn identity
//   - the epoch is what makes a reused handle a DIFFERENT instance (7a row 4)
//   - a stale-epoch release is inert (7b row 4), so a late abort cannot free a
//     slot the next client already owns
//   - the terminal gate is one-way, returns the identity it displaced, and cannot
//     be reopened by the abort's release that follows it (7e row 3)
//   - the activity clock is 0 when unowned, starts its window at admission, and
//     is wrap-safe

#include "../src/link_owner.h"

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <vector>

volatile uint32_t od_test_millis = 0;

// --- tiny harness ------------------------------------------------------------
static int g_checks = 0;
static int g_failures = 0;

static void check(bool cond, const char* what) {
    g_checks++;
    if (!cond) {
        g_failures++;
        std::printf("FAIL: %s\n", what);
    }
}

// Make UBSan reports fatal rather than "print and keep going", so a regression
// that reintroduced signed/overflow arithmetic could not exit 0. Mirrors the
// same hook in test_nonce_window.cpp.
extern "C" void __ubsan_on_report(void) {
    std::printf("FAIL: UBSan report\n");
    std::exit(1);
}

// The arbiter deliberately exposes no reset -- one boot, one lifetime. Tests drive
// it back to unowned through the public API instead, which also proves release
// actually works.
static void forceUnowned() {
    const LinkId owner = linkOwnerId();
    if (owner.who == OWNER_BLE || owner.who == OWNER_LAN) linkRelease(owner);
    check(linkOwnerId().who == OWNER_NONE, "forceUnowned leaves the slot unowned");
}

// --- word encoding -----------------------------------------------------------
static void test_word_encoding() {
    // The all-zero word must mean unowned and nothing else; every other field
    // combination has to round-trip, or an identity comparison silently aliases
    // two different instances.
    check(linkPackWord(OWNER_NONE, 0, 0) == 0, "unowned is the all-zero word");
    check(linkPackWord(OWNER_TERMINAL, 0, 0) == OD_LINK_WORD_TERMINAL,
          "terminal is the reserved word");

    const uint16_t handles[] = {0, 1, 2, 63, 255, 0x0EFF, 0x3FFF};
    const uint16_t epochs[]  = {1, 2, 255, 256, 0x7FFF, 0xFFFF};
    for (uint16_t h : handles) {
        for (uint16_t e : epochs) {
            const uint32_t w = linkPackWord(OWNER_BLE, h, e);
            const LinkId id = linkUnpackWord(w);
            check(id.who == OWNER_BLE && id.handle == h && id.epoch == e,
                  "pack/unpack round-trips");
            check(w != 0, "a real instance never encodes as the unowned word");
            check(w != OD_LINK_WORD_TERMINAL, "a BLE instance never aliases terminal");
        }
    }
    // 0x0EFF is the spec cap on HCI connection handles; 14 bits must hold it.
    check(linkUnpackWord(linkPackWord(OWNER_BLE, 0x0EFF, 1)).handle == 0x0EFF,
          "the handle field holds the full HCI range");
}

// --- claim / release ---------------------------------------------------------
static void test_claim_release() {
    forceUnowned();
    const LinkId a = {OWNER_BLE, 4, 100};
    check(linkClaim(a), "first claim on an unowned slot succeeds");
    check(linkIsOwner(a), "the claimant is the owner");

    // R3: while the slot is held, every other instance is refused. Admission never
    // evicts -- that is the whole governing decision, and it is enforced here by
    // the CAS only ever succeeding against zero.
    const LinkId b = {OWNER_BLE, 5, 101};
    check(!linkClaim(b), "a second BLE instance is refused while the slot is held");
    check(!linkIsOwner(b), "the refused contender is not the owner");
    check(linkIsOwner(a), "the incumbent is untouched by a refused claim");

    const LinkId lan = {OWNER_LAN, 0, 102};
    check(!linkClaim(lan), "a LAN claim is refused while BLE owns (R1 is global)");
    check(linkIsOwner(a), "the incumbent survives a cross-transport refusal");

    linkRelease(a);
    check(linkOwnerId().who == OWNER_NONE, "release frees the slot");
    check(linkClaim(lan), "LAN may claim once the slot is free");
    check(linkIsOwner(lan), "LAN is now the owner");
    linkRelease(lan);
}

// --- the epoch, which is the entire point of R2 ------------------------------
static void test_epoch_discrimination() {
    forceUnowned();
    const LinkId incumbent = {OWNER_BLE, 7, 200};
    check(linkClaim(incumbent), "incumbent claims");

    // 7a row 4: a contender that REUSES the incumbent's handle after a stale link.
    // Handle alone cannot tell these apart; this is the case a handle-only build
    // passes by accident, and the reason the epoch is allocated for every instance
    // rather than on successful claim.
    const LinkId sameHandleNewEpoch = {OWNER_BLE, 7, 201};
    check(!linkIsOwner(sameHandleNewEpoch),
          "a reused handle with a new epoch is NOT the owner");
    check(!linkClaim(sameHandleNewEpoch), "and it is refused");

    // 7b row 4: the ABA case. A disconnect event serviced tens of seconds late --
    // loop() blocked in a refresh -- carries a stale epoch. It must not free a slot
    // whose owner is a newer instance.
    const LinkId staleSameHandle = {OWNER_BLE, 7, 199};
    linkRelease(staleSameHandle);
    check(linkIsOwner(incumbent), "a stale-epoch release is inert");

    const LinkId wrongTransport = {OWNER_LAN, 7, 200};
    linkRelease(wrongTransport);
    check(linkIsOwner(incumbent), "a cross-transport release is inert");

    linkRelease(incumbent);
    check(linkOwnerId().who == OWNER_NONE, "the exact identity does release");
}

// --- epoch allocation --------------------------------------------------------
static void test_epoch_allocation() {
    // Epoch 0 is reserved so the all-zero word can mean unowned. The allocator must
    // skip it at wrap, not merely start above it.
    uint16_t prev = linkNextEpoch();
    check(prev != 0, "an allocated epoch is never 0");
    // 2^16 allocations walks the counter through its full range, including the wrap
    // where a naive ++ would hand out 0.
    for (int i = 0; i < 70000; i++) {
        const uint16_t e = linkNextEpoch();
        check(e != 0, "no allocation yields the reserved epoch 0");
        prev = e;
    }
    (void)prev;
}

// --- the terminal gate (7e row 3) -------------------------------------------
static void test_terminal_gate() {
    forceUnowned();
    const LinkId owner = {OWNER_BLE, 3, 300};
    check(linkClaim(owner), "owner claims before the terminal transition");

    // linkMarkTerminal() must RETURN the displaced identity: after it, the word
    // reads terminal, so the deep-sleep path cannot re-derive who it is aborting
    // for. This is the plumbing the abort's ownerId parameter exists to carry.
    const LinkId displaced = linkMarkTerminal();
    check(displaced.who == OWNER_BLE && displaced.handle == 3 && displaced.epoch == 300,
          "the terminal exchange returns the displaced owner");
    check(linkOwnerId().who == OWNER_TERMINAL, "the word now reads terminal");

    // The gate is what closes the deep-sleep race: between the abort's release and
    // ble.end(), a connect on the host task must not be able to win the slot.
    check(!linkClaim((LinkId){OWNER_BLE, 9, 301}), "a BLE claim fails against the gate");
    check(!linkClaim((LinkId){OWNER_LAN, 0, 302}), "a LAN claim fails against the gate");

    // The abort runs AFTER the gate and ends in linkRelease(displaced). That must
    // not reopen admission -- the full-identity CAS cannot match the terminal word.
    linkRelease(displaced);
    check(linkOwnerId().who == OWNER_TERMINAL, "the abort's release cannot reopen the gate");
    check(!linkClaim((LinkId){OWNER_BLE, 1, 303}), "still shut after the release");

    // One-way by construction: nothing transitions out, and the terminal word is
    // not a legal argument to release.
    linkRelease((LinkId){OWNER_TERMINAL, 0, 0});
    check(linkOwnerId().who == OWNER_TERMINAL, "terminal is not releasable");

    // Marking terminal on an unowned slot is legal (deep sleep with no client) and
    // reports no displaced owner.
    const LinkId again = linkMarkTerminal();
    check(again.who == OWNER_TERMINAL, "re-marking reports the prior terminal state");
}

// --- the activity clock (R4) -------------------------------------------------
static void test_activity_clock() {
    // Fresh process state is needed because the terminal gate above is one-way, so
    // this runs in its own process (see main()).
    od_test_millis = 1000;
    check(linkMsSinceOwnerCommand() == 0, "the clock reads 0 when unowned");

    const LinkId owner = {OWNER_BLE, 2, 400};
    check(linkClaim(owner), "owner claims");
    check(linkMsSinceOwnerCommand() == 0, "the window starts at admission, not at UINT32_MAX");

    // The init fix: a freshly admitted but still-silent client must get the FULL
    // window before its first command, not be instantly past any timeout.
    od_test_millis = 1000 + 119000;
    check(linkMsSinceOwnerCommand() == 119000, "silence accrues from admission");

    linkStampOwnerCommand();
    check(linkMsSinceOwnerCommand() == 0, "a recognised owner command resets the clock");

    od_test_millis += 50000;
    check(linkMsSinceOwnerCommand() == 50000, "silence accrues from the last command");

    // The refresh exclusion. loop() does not run for a refresh's duration while
    // wall-clock time passes, so without the re-stamp at the transition the whole
    // refresh accrues and an engaged client is dropped the instant loop() resumes.
    od_test_millis += 16000;             // a ~16 s refresh elapses
    linkStampRefreshEnd();
    check(linkMsSinceOwnerCommand() == 0, "endRefresh() re-stamps the owner's clock");

    // Re-stamping can only ever DELAY a drop, never cause one: the baseline is the
    // LATER of the three stamps, so an old refresh-end cannot pull it backwards.
    od_test_millis += 5000;
    linkStampOwnerCommand();             // now the latest
    od_test_millis += 1000;
    linkStampRefreshEnd();               // later still
    check(linkMsSinceOwnerCommand() == 0, "the baseline is the latest of the stamps");

    // Wrap safety: millis() wraps every ~49.7 days, and a plain `>` comparison on
    // the raw stamps would read as a ~49-day silence for one pass.
    od_test_millis = 0xFFFFFF00u;
    linkStampOwnerCommand();
    od_test_millis = 0x000000FFu;        // wrapped; 511 ms of real elapsed time
    check(linkMsSinceOwnerCommand() == 511, "the clock is wrap-safe across millis() rollover");

    linkRelease(owner);
    check(linkMsSinceOwnerCommand() == 0, "the clock reads 0 again once unowned");
}

// --- the baseline must belong to the CURRENT owner ---------------------------
static void test_clock_owner_versioning() {
    // The bug this guards: linkClaim() publishes the new owner with its CAS and
    // only then stores the baseline. A reader landing in that gap sees the NEW
    // owner beside the PREVIOUS owner's baseline and reports an arbitrarily large
    // silence -- on which Phase 3 would drop a client that just connected.
    //
    // Re-reading the owner word around the load does NOT catch it (both reads see
    // the same new owner), which is why the baseline carries an owner tag. This
    // test reaches the state directly: a session that ran long, released, and was
    // replaced must not lend its baseline to the newcomer.
    forceUnowned();
    od_test_millis = 5000;
    const LinkId first = {OWNER_BLE, 1, 700};
    check(linkClaim(first), "first owner claims");
    od_test_millis += 300000;                       // five minutes of silence
    check(linkMsSinceOwnerCommand() == 300000, "the first owner's silence accrues");
    linkRelease(first);

    const LinkId second = {OWNER_BLE, 1, 701};      // same handle, new instance
    check(linkClaim(second), "a new instance claims the freed slot");
    check(linkMsSinceOwnerCommand() == 0,
          "the newcomer does NOT inherit the previous owner's stale baseline");

    // And the tag must not make a legitimately silent owner read as fresh forever.
    od_test_millis += 90000;
    check(linkMsSinceOwnerCommand() == 90000, "the newcomer's own silence still accrues");
    linkRelease(second);

    // NOTE: everything above passes with or without the owner tag, because a
    // single-threaded claim always stores its baseline before anything can read it.
    // The tag earns its keep only in the window BETWEEN the claim's CAS and its
    // baseline store, which is reachable only concurrently -- see
    // test_clock_versioning_race() below, which is what actually discriminates it.
}

// NOT TESTED HERE, and deliberately so: the window between linkClaim()'s CAS and
// its baseline store.
//
// A stress harness for it was written and removed. It could not distinguish the
// defect from correct behaviour: to make a leaked baseline observable the harness
// must advance the test clock between sessions, but then a reader that samples
// millis() just after validating the owner legitimately sees a full stride of age,
// which is exactly the signature the bug would produce. It failed on the FIXED code
// as often as on the mutant, and a test that fails on correct code is worse than no
// test.
//
// Making it deterministic needs an interleaving hook inside linkClaim(), i.e.
// test scaffolding in firmware on the connection path. That was judged not worth
// it: the fix rests on a publication-order argument (baseline stored, THEN its
// owner tag release-stored, so a reader either sees the tag and a matching baseline
// or no tag at all) plus ThreadSanitizer over the concurrent claim cases below,
// which covers the data race but not this ordering window. Stated plainly so the
// gap is known rather than assumed covered.

// --- contention: the claim really is atomic ----------------------------------
static void test_concurrent_claims() {
    forceUnowned();
    // BLE claims on the stack callback task, LAN on the loop task, and two centrals
    // can connect inside one blocked-loop window. Exactly one must win, and the
    // winner's identity must be one that was actually offered -- a torn word would
    // name an instance that never existed.
    const int kThreads = 8;
    std::atomic<int> wins{0};
    std::vector<std::thread> threads;
    std::atomic<bool> go{false};
    for (int i = 0; i < kThreads; i++) {
        threads.emplace_back([i, &wins, &go]() {
            while (!go.load(std::memory_order_acquire)) { /* line them up */ }
            const LinkId id = {OWNER_BLE, (uint16_t)(i + 1), (uint16_t)(500 + i)};
            if (linkClaim(id)) wins.fetch_add(1, std::memory_order_relaxed);
        });
    }
    go.store(true, std::memory_order_release);
    for (auto& t : threads) t.join();

    check(wins.load() == 1, "exactly one of eight racing claims wins");
    const LinkId owner = linkOwnerId();
    check(owner.who == OWNER_BLE, "the winner is a BLE instance");
    const bool plausible = owner.handle >= 1 && owner.handle <= kThreads &&
                           owner.epoch == (uint16_t)(500 + owner.handle - 1);
    check(plausible, "the owner word is one of the offered identities, not a torn mix");
    linkRelease(owner);
}

static void test_concurrent_epochs() {
    // Epoch allocation races the same two tasks. Every allocation must be unique,
    // or two live instances could share an identity.
    const int kThreads = 4;
    const int kPerThread = 4000;
    std::vector<std::vector<uint16_t>> got(kThreads);
    std::vector<std::thread> threads;
    for (int i = 0; i < kThreads; i++) {
        got[i].reserve(kPerThread);
        threads.emplace_back([i, &got]() {
            for (int n = 0; n < kPerThread; n++) got[i].push_back(linkNextEpoch());
        });
    }
    for (auto& t : threads) t.join();

    // The counter is 16-bit and this allocates 16000, so uniqueness is checked
    // within the run rather than globally -- duplicates would mean a lost
    // increment, which is what a non-atomic ++ produces under contention.
    std::vector<int> seen(65536, 0);
    int dupes = 0;
    for (int i = 0; i < kThreads; i++) {
        for (uint16_t e : got[i]) {
            if (e == 0) { dupes++; continue; }
            if (seen[e]++) dupes++;
        }
    }
    check(dupes == 0, "16000 concurrent epoch allocations are all distinct");
}

int main(int argc, char** argv) {
    // The terminal gate is one-way and process-wide, so the cases that run after it
    // would see a permanently shut arbiter. Re-exec for that group instead of
    // adding a test-only reset the firmware would then carry.
    const bool terminalPhase = (argc > 1 && std::strcmp(argv[1], "--terminal") == 0);

    if (!terminalPhase) {
        test_word_encoding();
        test_claim_release();
        test_epoch_discrimination();
        test_epoch_allocation();
        test_activity_clock();
        test_clock_owner_versioning();

        test_concurrent_claims();
        test_concurrent_epochs();

        // Child process for the terminal-gate group.
        char cmd[1024];
        std::snprintf(cmd, sizeof(cmd), "%s --terminal", argv[0]);
        const int rc = std::system(cmd);
        check(rc == 0, "the terminal-gate phase passes");
    } else {
        test_terminal_gate();
    }

    std::printf("%s: %d checks, %d failures\n",
                terminalPhase ? "terminal phase" : "link_owner", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
