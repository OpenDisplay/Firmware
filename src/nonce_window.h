#ifndef NONCE_WINDOW_H
#define NONCE_WINDOW_H

// Anti-replay sliding window — pure state machine, ZERO dependencies.
//
// This header deliberately includes nothing from Arduino, mbedtls, or the
// firmware logging layer, and it touches no global state. Everything here
// operates on plain values passed in by the caller, so the whole state machine
// can be compiled and exercised on a host under UBSan/ASan by
// tools/test_nonce_window.cpp. See docs/PLAN_PHASE1_NONCE_REPLAY_2026-07-26.md
// Decision D.
//
// Representation (RFC 4303 / RFC 6347 "shifting" style, as opposed to the
// circular RFC 6479 / WireGuard style — see Decision B):
//
//   bit i of the bitmap == "counter (last_seen - i) has been consumed".
//   bit 0 is last_seen itself.
//
// The backward window is therefore implicitly OD_NONCE_BACKWARD_BITS - 1; there
// is no separate window constant to keep in step, and no insertion index to
// reset. "Not seen" is a clear bit rather than a reserved sentinel value, so a
// fresh session (last_seen = 0, all-zero bitmap) accepts counter 0 exactly once
// with no has_seen_counter flag.

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// Width of the backward (out-of-order tolerance) window, in bits.
// uint64_t[4] = 32 B. Kept strictly greater than OD_NONCE_FORWARD_CAP so that a
// legal forward slide can never exceed the bitmap width — that keeps the
// wholesale-clear branch in od_nonce_commit() off the normal path.
#define OD_NONCE_BACKWARD_BITS 256

// Largest forward jump that is accepted. This is a HEURISTIC, not an invariant
// firmware can prove: the real bound lives in the client's retransmit budget
// (max_retx = max(3*W, n/2)) and its blocks_per_ack setting, both of which live
// in another repo and one of which is a user-facing Home Assistant option. See
// Decision A in docs/PLAN_PHASE1_NONCE_REPLAY_2026-07-26.md.
#define OD_NONCE_FORWARD_CAP 128

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
// The four tests are ORDERED and the order is load-bearing. fwd and back sum to
// zero mod 2^64, so they cannot both be small: with the current constants
// fwd <= 128 && back < 256 would require fwd + back <= 384 == 0 (mod 2^64),
// true only when both are zero — the case the first test already consumed. A
// counter far from the window in either direction leaves both huge and falls
// through to NONCE_OUT_OF_WINDOW. Do not reorder.
//
// Unsigned wrapping arithmetic only: the counter is parsed off the wire BEFORE
// the CCM tag is verified, so an unauthenticated attacker controls both
// operands. Converting a uint64_t >= 2^63 to int64_t is implementation-defined
// before C++20, the signed subtraction can overflow, and negating INT64_MIN is
// UB — three ways to be undefined on attacker-chosen input. Unsigned overflow
// is defined as modular arithmetic, making this total over all 2^64 inputs.
static inline NonceResult od_nonce_check(const uint64_t* bm, uint64_t last_seen, uint64_t counter) {
    const uint64_t fwd = counter - last_seen;   /* wraps; 0 when equal             */
    const uint64_t back = last_seen - counter;  /* wraps; fwd + back == 0 mod 2^64 */

    if (fwd == 0u) return od_nonce_bit_test(bm, 0) ? NONCE_REPLAY : NONCE_OK;
    if (fwd <= OD_NONCE_FORWARD_CAP) return NONCE_OK; /* ahead: cannot have been seen */
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

// Records `counter` as consumed. MUST only be called after the frame carrying
// it has been authenticated (CCM tag verified) — that is the D2 fix.
//
// Defined (never UB) for every input, including inputs od_nonce_check() would
// have rejected: the host test calls it directly, and a future
// OD_NONCE_FORWARD_CAP increase must not be able to produce an over-wide shift.
// The wholesale-clear path is unreachable through od_nonce_check() today (cap
// 128 < 256 bits) but is still correct when it fires on a FORWARD jump: every
// counter it discards is then >= 256 behind and is rejected on width, never
// mis-reported as unseen.
//
// Sharp edge, stated rather than hidden: a counter more than
// OD_NONCE_BACKWARD_BITS *behind* last_seen also lands in the forward branch
// (fwd and back are complements, so its fwd is enormous), clearing the bitmap
// and RE-WINDING last_seen to that counter — un-seeing everything. That is
// unreachable through od_nonce_check(), which returns NONCE_OUT_OF_WINDOW for
// such a counter so it is never committed, and the contract above is that
// commit runs only for frames that both checked OK and authenticated. It is the
// edge to watch if a future caller ever commits without checking first.
static inline void od_nonce_commit(uint64_t* bm, uint64_t* last_seen, uint64_t counter) {
    const uint64_t fwd = counter - *last_seen;
    const uint64_t back = *last_seen - counter;

    if (fwd == 0u) {
        od_nonce_bit_set(bm, 0);
        return;
    }
    if (back < OD_NONCE_BACKWARD_BITS) {
        /* backward, inside the window: last_seen does not move.
           Unambiguous: fwd and back are complements mod 2^64, so back < 256
           forces fwd >= 2^64 - 255 — they can never both be small. */
        od_nonce_bit_set(bm, back);
        return;
    }
    /* forward */
    od_nonce_bitmap_shift_left(bm, fwd);
    *last_seen = counter;
    od_nonce_bit_set(bm, 0);
}

#endif  // NONCE_WINDOW_H
