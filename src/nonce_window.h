#ifndef NONCE_WINDOW_H
#define NONCE_WINDOW_H

// Anti-replay sliding window — pure state machine, ZERO dependencies.
//
// This header deliberately includes nothing from Arduino, mbedtls, or the
// firmware logging layer, and it touches no global state. Everything here
// operates on plain values passed in by the caller, so the whole state machine
// can be compiled and exercised on a host under UBSan/ASan by
// tools/test_nonce_window.cpp.
//
// Representation ("shifting" style, as opposed to the circular RFC 6479 /
// WireGuard style):
//
//   bit i of the bitmap == "counter (last_seen - i) has been consumed".
//   bit 0 is last_seen itself.
//
// The backward window is therefore implicitly OD_NONCE_BACKWARD_BITS - 1; there
// is no separate window constant to keep in step, and no insertion index to
// reset. "Not seen" is a clear bit rather than a reserved sentinel value, so a
// fresh session (last_seen = 0, all-zero bitmap) accepts counter 0 exactly once
// with no has_seen_counter flag.
//
// --- relationship to RFC 4303 -----------------------------------------------
//
// The decision rule is RFC 4303 Appendix A2's anti-replay window, and the three
// cases map one-to-one: right of the window -> authenticate, then slide (clearing
// wholesale when the jump exceeds the width); inside the window -> consult the
// bitmap; left of the window -> discard. So does the ordering that makes it safe,
// "if the MAC is valid, the window is updated" — here od_nonce_check() decides,
// CCM verifies, and only then does od_nonce_commit() run.
//
// Three deliberate departures, none of them semantic:
//
//   - Counters are 0-based. RFC 4303 §3.3.3 starts sequence numbers at 1; this
//     protocol's client sends counter 0 as its first command, so a 1-based rule
//     would reject every client's first frame.
//   - The full 64-bit counter is on the wire, so there is no ESN high-order-bit
//     inference to do. That machinery would be complexity with no function.
//   - The window is 256 bits rather than the RFC's 32 minimum / 64 default,
//     which §3.4.3 explicitly permits.
//
// Following §3.3.3, the counter does NOT wrap: it is treated as a plain
// monotonically increasing uint64_t, and a session that reached UINT64_MAX would
// have to re-authenticate rather than cycle. Wrapping would reuse a (key, nonce)
// pair, which is the precise CCM failure this file exists to prevent. Exhausting
// 2^64 counters inside one session is not reachable in practice.

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// Width of the backward (out-of-order / duplicate arrival) window, in bits.
// uint64_t[4] = 32 B.
//
// This width is ONLY reordering tolerance. Replay protection does not depend on
// it: a counter at or below last_seen is either caught by the bitmap (within the
// width) or rejected on width, and only counters above last_seen are ever
// accepted — and those, by the monotonicity argument on od_nonce_check(), were
// never consumed. Narrowing or widening it cannot create a replay hole.
//
// Nor is it a correctness cliff, because a backward rejection is self-healing:
// the client re-encrypts every retransmission with a fresh, HIGHER counter (it
// never resends the original ciphertext), and a higher counter is accepted
// unconditionally. A frame rejected here is re-sent under a counter that is not.
//
// So the exact value is not load-bearing, and there is deliberately no attempt to
// derive it from the client's pipe window, blocks_per_ack or retransmit budget —
// none of those bear on it. 256 is a generous margin over the zero reordering the
// transport actually produces (the client assigns counters synchronously
// immediately before each write, ATT preserves order, and both targets consume
// their RX rings FIFO), at a cost of 32 bytes per session.
#define OD_NONCE_BACKWARD_BITS 256

#define OD_NONCE_BITMAP_WORDS (OD_NONCE_BACKWARD_BITS / 64)

enum NonceResult {
    NONCE_OK = 0,
    NONCE_BAD_SESSION,
    NONCE_OUT_OF_WINDOW,
    NONCE_REPLAY
};

static inline bool od_nonce_bit_test(const uint64_t* bm, uint64_t bit) {
    return ((bm[(size_t)(bit >> 6)] >> (unsigned)(bit & 63u)) & 1ull) != 0ull;
}

static inline void od_nonce_bit_set(uint64_t* bm, uint64_t bit) {
    bm[(size_t)(bit >> 6)] |= (1ull << (unsigned)(bit & 63u));
}

// Pure: decides whether `counter` may be accepted. Writes nothing.
//
// There is NO forward bound, and its absence is the point. A counter above
// last_seen is accepted at any distance, exactly as RFC 4303 does, because the
// sender may legitimately have burned counters this receiver never saw — every
// PIPE retransmission spends a fresh one. A forward cap here would not bound an
// attacker (see the commit contract below: passing this check mutates nothing,
// and only a verified CCM tag commits), but it WOULD strand the session: once a
// gap exceeded the cap, nothing would commit, last_seen would never advance, and
// every subsequent frame — each carrying a still higher counter — would be
// rejected further out than the last, until re-authentication.
//
// The security invariant is "a consumed counter is never returned NONCE_OK
// again", and it rests on last_seen being monotonically non-decreasing: only the
// forward branch of od_nonce_commit() assigns it, and only upward. Every consumed
// counter is therefore <= last_seen forever, so the `counter > last_seen` arm
// cannot re-accept one. Below that, a consumed counter is bitmap-caught while it
// is represented and rejected on width once it is not — the region below the
// window is closed, never waved through.
//
// Comparison is plain numeric, NOT modular. Modular arithmetic makes a counter
// far behind indistinguishable from one far ahead (the two differences are
// complements mod 2^64), which is what allowed an ancient counter to present as
// an enormous forward jump. Numeric ordering removes that overlap structurally
// rather than relying on the caller to check before committing. For the same
// reason, do not reintroduce a bound by setting a cap to UINT64_MAX or by
// treating "not within the backward window" as forward — either restores the
// overlap this ordering exists to eliminate.
//
// Total over all 2^64 inputs and never UB: the counter is parsed off the wire
// BEFORE the CCM tag is verified, so an unauthenticated attacker controls both
// operands. Only unsigned comparison and one unsigned subtraction are used, and
// that subtraction is evaluated solely on the branch where counter < last_seen,
// so it cannot wrap. Converting a uint64_t >= 2^63 to int64_t is
// implementation-defined before C++20, the signed subtraction can overflow, and
// negating INT64_MIN is UB — three ways to be undefined on attacker-chosen input,
// all avoided.
static inline NonceResult od_nonce_check(const uint64_t* bm, uint64_t last_seen, uint64_t counter) {
    if (counter == last_seen) return od_nonce_bit_test(bm, 0) ? NONCE_REPLAY : NONCE_OK;
    if (counter > last_seen) return NONCE_OK;   /* ahead: never consumed, see above */

    const uint64_t back = last_seen - counter;  /* counter < last_seen, so no wrap */
    if (back < OD_NONCE_BACKWARD_BITS) return od_nonce_bit_test(bm, back) ? NONCE_REPLAY : NONCE_OK;
    return NONCE_OUT_OF_WINDOW;
}

// Shift the bitmap left by `shift` bits (bit i -> bit i + shift), clearing the
// vacated low bits. Handles shift == 0 and shift >= 64 explicitly: `x << 64` is
// undefined behaviour in C/C++ and is the classic bug in this pattern.
static inline void od_nonce_bitmap_shift_left(uint64_t* bm, uint64_t shift) {
    if (shift == 0u) return;
    if (shift >= OD_NONCE_BACKWARD_BITS) {
        memset(bm, 0, sizeof(uint64_t) * OD_NONCE_BITMAP_WORDS);
        return;
    }
    const size_t word_shift = (size_t)(shift >> 6);
    const unsigned bit_shift = (unsigned)(shift & 63u);
    for (size_t i = OD_NONCE_BITMAP_WORDS; i-- > 0;) {
        uint64_t v = 0u;
        if (i >= word_shift) {
            v = bm[i - word_shift] << bit_shift;
            if (bit_shift != 0u && i > word_shift) {
                v |= bm[i - word_shift - 1] >> (64u - bit_shift);
            }
        }
        bm[i] = v;
    }
}

// Records `counter` as consumed. MUST only be called after the frame carrying it
// has been authenticated (CCM tag verified) — that is the D2 fix, and it is what
// makes the unbounded forward arm of od_nonce_check() safe. All 8 counter bytes
// sit inside the CCM nonce, so a tampered counter changes the keystream and fails
// the tag: a committed counter is always exactly the one the key holder emitted.
//
// last_seen moves only upward, which is the whole security argument above. The
// wholesale-clear inside the shift is now an ORDINARY path, not an off-normal
// one: it fires whenever an authenticated frame lands more than
// OD_NONCE_BACKWARD_BITS ahead, which is the case this design exists to accept.
// It stays correct when it does, because every counter it discards is then at
// least OD_NONCE_BACKWARD_BITS behind the new last_seen and is rejected on width
// — never mis-reported as unseen.
//
// Defined (never UB) for every input, including inputs od_nonce_check() would
// have rejected, because the host test calls it directly. A counter further than
// OD_NONCE_BACKWARD_BITS behind is a deliberate no-op rather than a shift: under
// numeric ordering it can no longer masquerade as a forward jump, so the old
// hazard of such a counter rewinding last_seen and un-seeing the bitmap cannot
// arise — by construction, not by contract.
static inline void od_nonce_commit(uint64_t* bm, uint64_t* last_seen, uint64_t counter) {
    if (counter == *last_seen) {
        od_nonce_bit_set(bm, 0);
        return;
    }
    if (counter < *last_seen) {
        /* backward: last_seen does not move. Outside the width there is nowhere
           to record it, and nothing needs recording — od_nonce_check() already
           rejects everything that far behind. */
        const uint64_t back = *last_seen - counter;
        if (back < OD_NONCE_BACKWARD_BITS) od_nonce_bit_set(bm, back);
        return;
    }
    /* forward */
    od_nonce_bitmap_shift_left(bm, counter - *last_seen);
    *last_seen = counter;
    od_nonce_bit_set(bm, 0);
}

#endif  // NONCE_WINDOW_H
