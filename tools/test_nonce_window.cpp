// Host test for src/nonce_window.h — standalone, no test framework.
//
// Build and run from the repo root:
//
//   g++ -std=c++17 -Wall -Wextra -Werror -O1 -fsanitize=undefined,address
//       tools/test_nonce_window.cpp -o /tmp/test_nonce_window
//   /tmp/test_nonce_window
//
// This file is as much a written-down statement of the intended semantics of the
// anti-replay window as it is a test.
//
// The representation under test (see the header):
//   bit i of the bitmap == "counter (last_seen - i) has been consumed"
//   bit 0 == last_seen itself
// so the backward window is OD_NONCE_BACKWARD_BITS wide (indices 0..255). There
// is no forward window: a counter ahead of last_seen is accepted at any distance.

#include "../src/nonce_window.h"

#include <cinttypes>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <set>

// ---------------------------------------------------------------------------
// Make UBSan reports fatal
//
// UBSan defaults to "print and keep going", so a regression that reintroduces
// signed counter arithmetic would emit a runtime-error line and still exit 0 —
// invisible to CI. __ubsan_on_report is the sanitizer's weak notification hook;
// overriding it turns any report into a nonzero exit without needing the caller
// to remember UBSAN_OPTIONS=halt_on_error=1.
//
// Defined unconditionally: libubsan declares it weak, so this overrides it when
// the sanitizer is linked in, and is simply unreferenced when it is not. (It
// cannot be guarded on a macro — GCC only started defining __SANITIZE_UNDEFINED__
// in GCC 14, and this must work on older CI runners.)
// ---------------------------------------------------------------------------

extern "C" void __ubsan_on_report(void);
extern "C" void __ubsan_on_report(void) {
    std::fputs("FAIL: undefined behaviour reported by UBSan (see the runtime error above)\n",
               stderr);
    std::fflush(stderr);
    std::_Exit(EXIT_FAILURE);
}

// ---------------------------------------------------------------------------
// Minimal check harness
// ---------------------------------------------------------------------------

static unsigned long g_checks = 0;
static unsigned long g_failures = 0;

// Free-form context describing the enclosing loop iteration, printed on failure
// so that a failing case inside a data-driven loop is identifiable.
static char g_context[256] = "";

static void set_context(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

static void set_context(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(g_context, sizeof(g_context), fmt, ap);
    va_end(ap);
}

static void clear_context(void) { g_context[0] = '\0'; }

static const char* res_name(NonceResult r) {
    switch (r) {
        case NONCE_OK: return "NONCE_OK";
        case NONCE_BAD_SESSION: return "NONCE_BAD_SESSION";
        case NONCE_OUT_OF_WINDOW: return "NONCE_OUT_OF_WINDOW";
        case NONCE_REPLAY: return "NONCE_REPLAY";
    }
    return "NONCE_<invalid>";
}

static void report_failure(const char* file, int line, const char* expr) {
    ++g_failures;
    if (g_context[0] != '\0') {
        std::printf("FAIL %s:%d [%s]: %s\n", file, line, g_context, expr);
    } else {
        std::printf("FAIL %s:%d: %s\n", file, line, expr);
    }
}

#define CHECK(expr)                                        \
    do {                                                   \
        ++g_checks;                                        \
        if (!(expr)) report_failure(__FILE__, __LINE__, #expr); \
    } while (0)

// Compares two NonceResult values and prints both symbolic names on mismatch.
#define CHECK_RES(actual, expected)                                            \
    do {                                                                       \
        ++g_checks;                                                            \
        const NonceResult check_a_ = (actual);                                 \
        const NonceResult check_e_ = (expected);                               \
        if (check_a_ != check_e_) {                                            \
            ++g_failures;                                                      \
            if (g_context[0] != '\0') {                                        \
                std::printf("FAIL %s:%d [%s]: %s -> got %s, want %s\n",        \
                            __FILE__, __LINE__, g_context, #actual,            \
                            res_name(check_a_), res_name(check_e_));           \
            } else {                                                           \
                std::printf("FAIL %s:%d: %s -> got %s, want %s\n", __FILE__,   \
                            __LINE__, #actual, res_name(check_a_),             \
                            res_name(check_e_));                               \
            }                                                                  \
        }                                                                      \
    } while (0)

// ---------------------------------------------------------------------------
// State container
// ---------------------------------------------------------------------------

struct NonceState {
    uint64_t bm[OD_NONCE_BITMAP_WORDS];
    uint64_t last_seen;
};

// All members are uint64_t, so the struct has no padding and can be compared
// byte-for-byte by the purity test below.
static_assert(sizeof(NonceState) == sizeof(uint64_t) * (OD_NONCE_BITMAP_WORDS + 1),
              "NonceState must be padding-free for the byte-identical purity check");

static void state_reset(NonceState* s, uint64_t last_seen) {
    std::memset(s->bm, 0, sizeof(s->bm));
    s->last_seen = last_seen;
}

static NonceResult check(const NonceState* s, uint64_t counter) {
    return od_nonce_check(s->bm, s->last_seen, counter);
}

static void commit(NonceState* s, uint64_t counter) {
    od_nonce_commit(s->bm, &s->last_seen, counter);
}

// Accept-and-record, the way firmware uses the pair: check, and only commit if
// the frame would be accepted (and, in firmware, only after the CCM tag verifies).
static NonceResult check_and_commit(NonceState* s, uint64_t counter) {
    const NonceResult r = check(s, counter);
    if (r == NONCE_OK) commit(s, counter);
    return r;
}

// ---------------------------------------------------------------------------
// Decision D coverage: fresh session (last_seen = 0, empty bitmap)
//
// "Not seen" is a clear bit, not a reserved sentinel, so counter 0 on a virgin
// state is accepted exactly once with no has_seen_counter flag anywhere.
// ---------------------------------------------------------------------------

static void test_fresh_session(void) {
    NonceState s;
    state_reset(&s, 0);

    CHECK_RES(check(&s, 0), NONCE_OK);
    commit(&s, 0);
    CHECK_RES(check(&s, 0), NONCE_REPLAY);

    // last_seen has not moved: committing at fwd == 0 only sets bit 0.
    CHECK(s.last_seen == 0);
    CHECK(od_nonce_bit_test(s.bm, 0));

    // Counter 1 is one ahead and cannot have been seen.
    CHECK_RES(check(&s, 1), NONCE_OK);

    // The no-wrap contract, stated deliberately rather than inherited.
    //
    // Ordering is numeric, so with last_seen == 0 there is nothing "behind": every
    // other counter is ahead and accepted at any distance. These three were
    // previously read as modular distances behind zero (back == 1, 255, 256) and
    // answered OK / OK / OUT_OF_WINDOW. They are all OK now, and for the opposite
    // reason.
    CHECK_RES(check(&s, UINT64_MAX), NONCE_OK);
    CHECK_RES(check(&s, 0u - (uint64_t)OD_NONCE_BACKWARD_BITS), NONCE_OK);
    CHECK_RES(check(&s, 0u - (uint64_t)(OD_NONCE_BACKWARD_BITS - 1)), NONCE_OK);

    // The state effect is what actually differs, and it is why wrap is not
    // supported: accepting UINT64_MAX drives last_seen to the top of the range,
    // after which nothing below it can ever be accepted again and the session must
    // re-authenticate. Only a key holder can reach this (commit is post-CCM), so
    // it is a contract, not a vulnerability.
    {
        NonceState t;
        state_reset(&t, 0);
        commit(&t, UINT64_MAX);
        CHECK(t.last_seen == UINT64_MAX);
        CHECK_RES(check(&t, UINT64_MAX), NONCE_REPLAY);
        CHECK_RES(check(&t, 0), NONCE_OUT_OF_WINDOW);
    }
}

// ---------------------------------------------------------------------------
// Decision D coverage: D3 — re-presenting a committed counter is REPLAY,
// including at fwd == 0, the case today's firmware exempts.
// ---------------------------------------------------------------------------

static void test_d3_same_counter_replay(void) {
    const uint64_t base = 1000000u;

    // fwd == 0: the exempted case.
    {
        NonceState s;
        state_reset(&s, base);
        CHECK_RES(check(&s, base), NONCE_OK);  // virgin bit 0
        commit(&s, base);
        CHECK_RES(check(&s, base), NONCE_REPLAY);
        CHECK_RES(check(&s, base), NONCE_REPLAY);  // and it stays REPLAY
    }

    // Forward acceptance then immediate re-presentation: the accepted counter
    // becomes the new last_seen, so the replay lands on fwd == 0 again.
    {
        NonceState s;
        state_reset(&s, base);
        commit(&s, base);
        CHECK_RES(check_and_commit(&s, base + 5), NONCE_OK);
        CHECK(s.last_seen == base + 5);
        CHECK_RES(check(&s, base + 5), NONCE_REPLAY);
        // The old last_seen is now 5 behind and still recorded.
        CHECK_RES(check(&s, base), NONCE_REPLAY);
        // A gap counter between them was never committed.
        CHECK_RES(check(&s, base + 3), NONCE_OK);
    }

    // Backward, in-window acceptance then re-presentation.
    {
        NonceState s;
        state_reset(&s, base);
        commit(&s, base);
        CHECK_RES(check_and_commit(&s, base - 100), NONCE_OK);
        CHECK(s.last_seen == base);  // backward commits never move last_seen
        CHECK_RES(check(&s, base - 100), NONCE_REPLAY);
        CHECK_RES(check(&s, base - 99), NONCE_OK);
        CHECK_RES(check(&s, base - 101), NONCE_OK);
    }
}

// ---------------------------------------------------------------------------
// Decision D coverage: purity of od_nonce_check (D2)
//
// THE most important assertion in this file. "The tag is the only thing that may
// advance replay state" rests entirely on od_nonce_check writing nothing, so a
// pre-authentication call on attacker-controlled input cannot poison the window.
// Every result class is exercised against one snapshot and the whole state is
// compared byte-for-byte afterwards.
// ---------------------------------------------------------------------------

static void test_check_is_pure(void) {
    const uint64_t base = 500000u;

    NonceState s;
    state_reset(&s, base);
    // Build a non-trivial bitmap: bits at word boundaries and in between.
    commit(&s, base);
    const uint64_t seen_offsets[] = {1, 2, 63, 64, 65, 127, 128, 191, 192, 254, 255};
    for (size_t i = 0; i < sizeof(seen_offsets) / sizeof(seen_offsets[0]); ++i) {
        commit(&s, base - seen_offsets[i]);
    }

    unsigned char snapshot[sizeof(NonceState)];
    std::memcpy(snapshot, &s, sizeof(snapshot));

    // One input per result class produced by od_nonce_check.
    struct Case {
        uint64_t counter;
        NonceResult expected;
        const char* what;
    };
    const Case cases[] = {
        {base, NONCE_REPLAY, "equal, bit 0 set -> REPLAY"},
        {base + 1, NONCE_OK, "forward by 1 -> OK"},
        {base - 3, NONCE_OK, "backward, unseen -> OK"},
        {base - 64, NONCE_REPLAY, "backward, seen -> REPLAY"},
        {base - OD_NONCE_BACKWARD_BITS, NONCE_OUT_OF_WINDOW, "past the backward window"},
        // Forward is unbounded: these are the distances a forward cap would have
        // rejected, and every one of them must be OK.
        {base + 129, NONCE_OK, "just past where the old cap sat"},
        {base + 5000, NONCE_OK, "far ahead"},
        {base + (1ull << 32), NONCE_OK, "very far ahead"},
        {base + (1ull << 62), NONCE_OK, "absurdly far ahead"},
        // Genuinely behind: the distance must not exceed `base`, or the
        // subtraction underflows and the counter is numerically AHEAD -- which is
        // exactly the modular confusion this ordering removes.
        {base - 100000, NONCE_OUT_OF_WINDOW, "far behind"},
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        set_context("purity case: %s", cases[i].what);
        CHECK_RES(check(&s, cases[i].counter), cases[i].expected);
    }
    clear_context();

    CHECK(std::memcmp(snapshot, &s, sizeof(snapshot)) == 0);

    // The fwd == 0 OK class needs a state where bit 0 is clear; check purity there too.
    NonceState fresh;
    state_reset(&fresh, base);
    unsigned char fresh_snapshot[sizeof(NonceState)];
    std::memcpy(fresh_snapshot, &fresh, sizeof(fresh_snapshot));
    CHECK_RES(check(&fresh, base), NONCE_OK);
    CHECK(std::memcmp(fresh_snapshot, &fresh, sizeof(fresh_snapshot)) == 0);

    // Repeated calls on every input class, in bulk, still change nothing.
    for (int rep = 0; rep < 4; ++rep) {
        for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
            (void)check(&s, cases[i].counter);
        }
    }
    CHECK(std::memcmp(snapshot, &s, sizeof(snapshot)) == 0);
}

// ---------------------------------------------------------------------------
// Decision D coverage: shift edges
//
// For each forward delta d, seed a window with a spread of previously consumed
// counters, slide forward by d, and then require:
//   - every seeded counter whose new distance (d + i) is < OD_NONCE_BACKWARD_BITS
//     is still reported REPLAY (the bit moved with the window, it did not fall
//     off early and it did not land on the wrong index), and
//   - every seeded counter whose new distance is >= OD_NONCE_BACKWARD_BITS is
//     reported OUT_OF_WINDOW, never REPLAY.
//
// EVERY delta is driven through od_nonce_check, including the ones at and past
// the bitmap width. Forward acceptance is unbounded, so the wholesale-clear path
// is now reached the way production reaches it rather than by calling commit
// directly — which matters, because that path stopped being an unreachable guard
// and became the case this design exists to accept.
// ---------------------------------------------------------------------------

static const uint64_t kSeedOffsets[] = {0, 1, 2, 63, 64, 65, 127, 128, 129, 191, 192, 254, 255};
static const size_t kSeedCount = sizeof(kSeedOffsets) / sizeof(kSeedOffsets[0]);

static void seed_window(NonceState* s, uint64_t base) {
    state_reset(s, base);
    for (size_t i = 0; i < kSeedCount; ++i) {
        commit(s, base - kSeedOffsets[i]);
    }
    // Sanity: everything seeded reads back as consumed, before any slide.
    for (size_t i = 0; i < kSeedCount; ++i) {
        CHECK_RES(check(s, base - kSeedOffsets[i]), NONCE_REPLAY);
    }
    CHECK(s->last_seen == base);
}

static void run_shift_edge(uint64_t d) {
    const uint64_t base = 1000000u;
    NonceState s;
    seed_window(&s, base);

    set_context("shift edge d=%" PRIu64, d);

    // d == 0 lands on the equality branch, whose bit is already set by the seed.
    // Every other d is forward and accepted at any distance.
    CHECK_RES(check(&s, base + d), d == 0 ? NONCE_REPLAY : NONCE_OK);
    commit(&s, base + d);

    CHECK(s.last_seen == base + d);
    // The newly committed counter is always recorded.
    CHECK_RES(check(&s, base + d), NONCE_REPLAY);

    for (size_t i = 0; i < kSeedCount; ++i) {
        const uint64_t counter = base - kSeedOffsets[i];
        const uint64_t new_back = d + kSeedOffsets[i];
        const bool wholesale_clear = (d >= OD_NONCE_BACKWARD_BITS);
        set_context("shift edge d=%" PRIu64 ", seeded offset %" PRIu64
                    " (new back %" PRIu64 ")",
                    d, kSeedOffsets[i], new_back);
        if (new_back < OD_NONCE_BACKWARD_BITS && !wholesale_clear) {
            CHECK_RES(check(&s, counter), NONCE_REPLAY);
        } else {
            CHECK_RES(check(&s, counter), NONCE_OUT_OF_WINDOW);
        }
    }

    // Counters that were never seeded and are still inside the backward window
    // must read as unseen, not as bits smeared by the shift.
    for (uint64_t back = 1; back < OD_NONCE_BACKWARD_BITS; ++back) {
        const uint64_t counter = (base + d) - back;
        // Is this counter one of the seeded ones (or the pre-slide last_seen)?
        bool seeded = false;
        for (size_t i = 0; i < kSeedCount; ++i) {
            if (counter == base - kSeedOffsets[i]) seeded = true;
        }
        const bool expect_replay = seeded && (d < OD_NONCE_BACKWARD_BITS);
        set_context("shift edge d=%" PRIu64 ", back=%" PRIu64, d, back);
        CHECK_RES(check(&s, counter), expect_replay ? NONCE_REPLAY : NONCE_OK);
    }

    // Exactly at the window edge and beyond it: rejected on width.
    set_context("shift edge d=%" PRIu64 ", window edge", d);
    CHECK_RES(check(&s, (base + d) - OD_NONCE_BACKWARD_BITS), NONCE_OUT_OF_WINDOW);
    CHECK_RES(check(&s, (base + d) - (OD_NONCE_BACKWARD_BITS + 1)), NONCE_OUT_OF_WINDOW);

    clear_context();
}

static void test_shift_edges(void) {
    // Word boundaries, the exact width boundary (255 / 256, the shift-vs-clear
    // pair), and jumps far past it — all through od_nonce_check.
    const uint64_t deltas[] = {0,   1,    63,   64,     65,        127, 128, 129,
                               191, 192,  255,  256,    257,       300,
                               4096, (1ull << 32)};
    for (size_t i = 0; i < sizeof(deltas) / sizeof(deltas[0]); ++i) {
        run_shift_edge(deltas[i]);
    }
}

// ---------------------------------------------------------------------------
// Decision D coverage: wholesale slide
//
// A forward jump of at least OD_NONCE_BACKWARD_BITS clears the bitmap outright
// (x << 64 is UB, and shifting a 256-bit map by >= 256 has no surviving bits).
// Everything previously consumed must then come back OUT_OF_WINDOW, NOT REPLAY.
// Both reject, so conflating them is invisible in behaviour — and would hide a
// genuine slide bug where the map was cleared when it should not have been.
// ---------------------------------------------------------------------------

static void test_wholesale_slide(void) {
    const uint64_t base = 1000000u;
    const uint64_t jumps[] = {OD_NONCE_BACKWARD_BITS, OD_NONCE_BACKWARD_BITS + 1, 300, 100000,
                              (1ull << 40)};

    for (size_t j = 0; j < sizeof(jumps) / sizeof(jumps[0]); ++j) {
        const uint64_t d = jumps[j];
        NonceState s;
        seed_window(&s, base);
        set_context("wholesale slide d=%" PRIu64, d);

        commit(&s, base + d);
        CHECK(s.last_seen == base + d);

        // Bitmap holds exactly one bit: the counter just committed.
        CHECK(s.bm[0] == 1ull);
        for (size_t w = 1; w < OD_NONCE_BITMAP_WORDS; ++w) {
            CHECK(s.bm[w] == 0ull);
        }

        for (size_t i = 0; i < kSeedCount; ++i) {
            CHECK_RES(check(&s, base - kSeedOffsets[i]), NONCE_OUT_OF_WINDOW);
        }
        // The whole new backward window is unseen except bit 0.
        CHECK_RES(check(&s, base + d), NONCE_REPLAY);
        for (uint64_t back = 1; back < OD_NONCE_BACKWARD_BITS; ++back) {
            CHECK_RES(check(&s, (base + d) - back), NONCE_OK);
        }
    }
    clear_context();
}

// ---------------------------------------------------------------------------
// Decision D coverage: bit index correctness after a forward commit
//
// Direct statement of the shift invariant, below the od_nonce_check layer: the
// bit that denoted counter (L - i) under last_seen = L must denote exactly the
// same counter under L' = L + d, i.e. it must sit at index i + d. Bits pushed to
// index >= OD_NONCE_BACKWARD_BITS are gone.
// ---------------------------------------------------------------------------

static void test_bit_indices_after_shift(void) {
    const uint64_t base = 1000000u;
    const uint64_t deltas[] = {1, 63, 64, 65, 127};

    for (size_t k = 0; k < sizeof(deltas) / sizeof(deltas[0]); ++k) {
        const uint64_t d = deltas[k];
        NonceState s;
        seed_window(&s, base);

        bool before[OD_NONCE_BACKWARD_BITS];
        for (uint64_t i = 0; i < OD_NONCE_BACKWARD_BITS; ++i) {
            before[i] = od_nonce_bit_test(s.bm, i);
        }

        commit(&s, base + d);

        for (uint64_t i = 0; i < OD_NONCE_BACKWARD_BITS; ++i) {
            set_context("bit index d=%" PRIu64 ", old index %" PRIu64, d, i);
            if (i + d < OD_NONCE_BACKWARD_BITS) {
                // Same counter, new index.
                CHECK(od_nonce_bit_test(s.bm, i + d) == before[i]);
            }
            // The vacated low bits are all clear except bit 0, which the commit set.
            if (i < d) {
                CHECK(od_nonce_bit_test(s.bm, i) == (i == 0));
            }
        }
        clear_context();
    }
}

// ---------------------------------------------------------------------------
// Counter arithmetic at the extremes [M1]
//
// Expectations are derived from the numeric definition in the header, not from
// intuition: counters are plain monotonically increasing uint64_t values that do
// NOT wrap, and the ordered tests are counter == last_seen, counter > last_seen,
// back < BITS, else OUT_OF_WINDOW.
//
// UBSan makes the whole file self-checking against the signed formulation, in
// which these same inputs are undefined (int64_t conversion of values >= 2^63,
// signed overflow, negating INT64_MIN).
// ---------------------------------------------------------------------------

static void test_counter_arithmetic_extremes(void) {
    const uint64_t two63 = 1ull << 63;

    // counter = 2^63, last_seen = 1. The input that makes the signed form UB, and
    // the one whose answer INVERTED when ordering stopped being modular: it used
    // to be "out of window in both directions", because the modular fwd and back
    // were both enormous. Numerically it is simply ahead, so it is accepted --
    // and gated by the CCM tag, not by distance.
    {
        NonceState s;
        state_reset(&s, 1);
        CHECK(two63 > 1u);
        CHECK_RES(check(&s, two63), NONCE_OK);
    }
    // The mirror image is unchanged: last_seen = 2^63, counter = 1 is behind, far
    // past the width.
    {
        NonceState s;
        state_reset(&s, two63);
        CHECK_RES(check(&s, 1), NONCE_OUT_OF_WINDOW);
        CHECK_RES(check(&s, 0), NONCE_OUT_OF_WINDOW);
    }
    // From the very top of the range, everything is behind.
    {
        NonceState s;
        state_reset(&s, UINT64_MAX);
        CHECK_RES(check(&s, UINT64_MAX - two63), NONCE_OUT_OF_WINDOW);
        CHECK_RES(check(&s, UINT64_MAX - (uint64_t)OD_NONCE_BACKWARD_BITS), NONCE_OUT_OF_WINDOW);
        CHECK_RES(check(&s, UINT64_MAX - (uint64_t)(OD_NONCE_BACKWARD_BITS - 1)), NONCE_OK);
        CHECK_RES(check(&s, UINT64_MAX), NONCE_OK);  // equal, bit 0 clear
    }

    // Near UINT64_MAX. last_seen = UINT64_MAX - 2.
    //
    // This block used to assert that the window WRAPS -- that counter 0 is three
    // ahead of UINT64_MAX - 2 and acceptable. It no longer does, and that is the
    // deliberate contract: wrapping would reuse a (key, nonce) pair. Counters at
    // the low end of the range are simply behind, and the session must
    // re-authenticate rather than cycle.
    {
        const uint64_t L = UINT64_MAX - 2u;
        NonceState s;
        state_reset(&s, L);

        CHECK_RES(check(&s, L), NONCE_OK);  // equal, bit 0 clear
        commit(&s, L);
        CHECK_RES(check(&s, L), NONCE_REPLAY);

        // Forward, no wrap available: only two counters remain in the range.
        CHECK_RES(check(&s, UINT64_MAX - 1u), NONCE_OK);
        CHECK_RES(check(&s, UINT64_MAX), NONCE_OK);

        // Past the top there is nothing. 0 and 1 are far BEHIND, not ahead.
        CHECK_RES(check(&s, 0), NONCE_OUT_OF_WINDOW);
        CHECK_RES(check(&s, 1), NONCE_OUT_OF_WINDOW);

        // Backward across the top of the space behaves normally.
        CHECK_RES(check(&s, L - 1u), NONCE_OK);
        CHECK_RES(check(&s, L - (uint64_t)(OD_NONCE_BACKWARD_BITS - 1)), NONCE_OK);
        CHECK_RES(check(&s, L - (uint64_t)OD_NONCE_BACKWARD_BITS), NONCE_OUT_OF_WINDOW);

        // Consume the last counter in the range; the session is now exhausted.
        CHECK_RES(check_and_commit(&s, UINT64_MAX), NONCE_OK);
        CHECK(s.last_seen == UINT64_MAX);
        CHECK_RES(check(&s, UINT64_MAX), NONCE_REPLAY);
        CHECK_RES(check(&s, 0), NONCE_OUT_OF_WINDOW);
        // Everything below is either recorded or off the end -- never OK again.
        CHECK_RES(check(&s, L), NONCE_REPLAY);
        CHECK_RES(check(&s, UINT64_MAX - 1u), NONCE_OK);  // never committed, in window

        // Backward commits still do not move last_seen.
        CHECK_RES(check_and_commit(&s, UINT64_MAX - 1u), NONCE_OK);
        CHECK(s.last_seen == UINT64_MAX);
        CHECK_RES(check(&s, UINT64_MAX - 1u), NONCE_REPLAY);
    }
}

// ---------------------------------------------------------------------------
// Differential / property test against a naive oracle
//
// The oracle keeps the UNPRUNED set of every counter it has ever committed. That
// is the point: the previous version pruned counters as they fell out of the
// backward window, which meant it encoded the implementation's forgetting and
// could only ever agree with it. An unpruned set lets the oracle be written from
// the specification instead --
//
//   a counter is acceptable iff it has never been committed AND it is either
//   ahead of last_seen or within OD_NONCE_BACKWARD_BITS of it
//
// -- and it is what makes od_nonce_never_accepts_consumed() below possible, which
// is the one assertion in this file that does not restate the code.
//
// Counters stay clear of 0 and 2^64 so the walk never runs out of range; the
// extremes are covered by test_counter_arithmetic_extremes().
// ---------------------------------------------------------------------------

struct Oracle {
    std::set<uint64_t> ever_committed;   // never pruned
    uint64_t last_seen;
};

static NonceResult oracle_check(const Oracle& o, uint64_t counter) {
    const bool consumed = o.ever_committed.count(counter) != 0;
    if (counter > o.last_seen) return NONCE_OK;   // ahead: cannot have been consumed
    if (counter == o.last_seen) return consumed ? NONCE_REPLAY : NONCE_OK;
    const uint64_t back = o.last_seen - counter;
    if (back >= (uint64_t)OD_NONCE_BACKWARD_BITS) return NONCE_OUT_OF_WINDOW;
    return consumed ? NONCE_REPLAY : NONCE_OK;
}

static void oracle_commit(Oracle* o, uint64_t counter) {
    o->ever_committed.insert(counter);
    if (counter > o->last_seen) o->last_seen = counter;   // only ever upward
}

// THE security property, asserted directly and independently of how the window is
// represented: no counter that was ever consumed may be accepted again, at any
// distance, by any route. A regression that reintroduced modular overlap -- or a
// commit that rewound last_seen -- shows up here as an OK on a member of the set,
// whatever the bitmap happens to contain.
static void od_nonce_never_accepts_consumed(const NonceState* s, const Oracle& o) {
    for (std::set<uint64_t>::const_iterator it = o.ever_committed.begin();
         it != o.ever_committed.end(); ++it) {
        set_context("consumed counter %" PRIu64 " must never be OK (last_seen=%" PRIu64 ")",
                    *it, s->last_seen);
        CHECK(check(s, *it) != NONCE_OK);
    }
    clear_context();
}

static void test_differential_against_oracle(void) {
    std::mt19937_64 rng(0xD15EA5EULL);  // fixed seed: this test must be reproducible

    const int kSequences = 40;
    const int kStepsPerSequence = 200;

    for (int seq = 0; seq < kSequences; ++seq) {
        const uint64_t base = (1ull << 40) + (uint64_t)seq * 4096u;

        NonceState s;
        state_reset(&s, base);
        Oracle o;
        o.last_seen = base;

        for (int step = 0; step < kStepsPerSequence; ++step) {
            const unsigned bucket = (unsigned)(rng() % 100u);
            uint64_t counter;
            if (bucket < 40u) {
                // Forward by a small step, including 0 (a replay probe).
                counter = s.last_seen + (rng() % 129u);
            } else if (bucket < 60u) {
                // Forward by a large jump -- the case a forward cap used to reject
                // and which must now be accepted, including jumps wide enough to
                // clear the bitmap wholesale.
                counter = s.last_seen + 129u + (rng() % 100000u);
            } else if (bucket < 90u) {
                // Backward inside or just outside the window.
                counter = s.last_seen - (1u + rng() % (uint64_t)(OD_NONCE_BACKWARD_BITS + 64));
            } else {
                // Far behind.
                counter = s.last_seen - (uint64_t)OD_NONCE_BACKWARD_BITS -
                          (rng() % 100000u);
            }

            set_context("differential seq=%d step=%d counter=%" PRIu64
                        " last_seen=%" PRIu64,
                        seq, step, counter, s.last_seen);

            const NonceResult got = check(&s, counter);
            const NonceResult want = oracle_check(o, counter);
            CHECK_RES(got, want);
            CHECK(o.last_seen == s.last_seen);

            if (got == NONCE_OK) {
                commit(&s, counter);
                oracle_commit(&o, counter);
                CHECK(o.last_seen == s.last_seen);
            }
        }

        // End of sequence: sweep the entire backward window and compare, so that
        // any bit the two models disagree about is caught even if the random
        // walk never probed it.
        for (uint64_t back = 0; back < (uint64_t)OD_NONCE_BACKWARD_BITS + 8u; ++back) {
            const uint64_t counter = s.last_seen - back;
            set_context("differential sweep seq=%d back=%" PRIu64, seq, back);
            CHECK_RES(check(&s, counter), oracle_check(o, counter));
        }

        // ...and the security property over every counter this sequence consumed,
        // not just the ones still inside the window.
        od_nonce_never_accepts_consumed(&s, o);
    }
    clear_context();
}

// ---------------------------------------------------------------------------
// The cliff, as a regression test.
//
// A forward cap made this sequence unrecoverable: nothing commits, so last_seen
// never advances, and each retransmission's higher counter is rejected further
// out than the last. Asserted as a SEQUENCE rather than a point, because a point
// test at last_seen+129 would also pass under "cap = UINT64_MAX" -- one of the two
// shortcuts the header warns against.
// ---------------------------------------------------------------------------

static void test_forward_gap_is_not_a_cliff(void) {
    const uint64_t L = 1000u;

    // Distances that a cap would have rejected, each from a clean state.
    const uint64_t jumps[] = {129u, 300u, 5000u, 100000u, (1ull << 32), (1ull << 63)};
    for (size_t i = 0; i < sizeof(jumps) / sizeof(jumps[0]); ++i) {
        NonceState s;
        state_reset(&s, L);
        commit(&s, L);
        set_context("forward jump %" PRIu64, jumps[i]);

        CHECK_RES(check(&s, L + jumps[i]), NONCE_OK);
        commit(&s, L + jumps[i]);
        CHECK(s.last_seen == L + jumps[i]);
        CHECK_RES(check(&s, L + jumps[i]), NONCE_REPLAY);

        // The stream must KEEP flowing afterwards -- this is what a cap broke.
        for (uint64_t k = 1; k <= 8u; ++k) {
            CHECK_RES(check_and_commit(&s, L + jumps[i] + k), NONCE_OK);
        }
        CHECK(s.last_seen == L + jumps[i] + 8u);
    }
    clear_context();

    // The client's actual failure mode: escalating retransmissions. Nothing is
    // committed until the frame authenticates, so a run of checks at ever greater
    // distance must leave the state untouched AND stay acceptable.
    {
        NonceState s;
        state_reset(&s, L);
        commit(&s, L);
        unsigned char snapshot[sizeof(NonceState)];
        std::memcpy(snapshot, &s, sizeof(snapshot));

        uint64_t c = L + 129u;
        for (int i = 0; i < 32; ++i) {
            set_context("escalating retransmit %d counter=%" PRIu64, i, c);
            CHECK_RES(check(&s, c), NONCE_OK);
            CHECK(std::memcmp(snapshot, &s, sizeof(snapshot)) == 0);
            c += 1000u;
        }
        // The one that finally authenticates advances the window normally.
        CHECK_RES(check_and_commit(&s, c), NONCE_OK);
        CHECK(s.last_seen == c);
        clear_context();
    }
}

// ---------------------------------------------------------------------------
// The two shortcuts the header forbids.
//
// Setting a cap to UINT64_MAX, or treating "not within the backward window" as
// forward, both keep modular overlap: an ancient counter presents as an enormous
// forward jump and is accepted. Under numeric ordering it must be OUT_OF_WINDOW.
// This is the test that distinguishes the real fix from either shortcut.
// ---------------------------------------------------------------------------

static void test_far_behind_is_never_forward(void) {
    const uint64_t L = 1ull << 40;
    NonceState s;
    state_reset(&s, L);
    commit(&s, L);

    const uint64_t behind[] = {300u, 4096u, 100000u, (1ull << 32), (1ull << 39)};
    for (size_t i = 0; i < sizeof(behind) / sizeof(behind[0]); ++i) {
        set_context("far behind by %" PRIu64, behind[i]);
        CHECK_RES(check(&s, L - behind[i]), NONCE_OUT_OF_WINDOW);

        // And committing one anyway (the primitive is callable directly) must not
        // rewind last_seen or clear the bitmap -- the old sharp edge.
        NonceState t = s;
        commit(&t, L - behind[i]);
        CHECK(t.last_seen == L);
        CHECK(std::memcmp(&t, &s, sizeof(NonceState)) == 0);
    }
    clear_context();
}

// ---------------------------------------------------------------------------

int main(void) {
    test_fresh_session();
    test_d3_same_counter_replay();
    test_check_is_pure();
    test_shift_edges();
    test_wholesale_slide();
    test_bit_indices_after_shift();
    test_counter_arithmetic_extremes();
    test_forward_gap_is_not_a_cliff();
    test_far_behind_is_never_forward();
    test_differential_against_oracle();

    if (g_failures != 0u) {
        std::printf("FAILED %lu of %lu checks\n", g_failures, g_checks);
        return EXIT_FAILURE;
    }
    std::printf("PASSED %lu checks\n", g_checks);
    return EXIT_SUCCESS;
}
