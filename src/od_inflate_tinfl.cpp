/*
 * od_inflate_tinfl — ESP32 inflate adapter backed by the ROM miniz `tinfl`.
 * See od_inflate_tinfl.h for the rationale, the RAM cost, and why the WiFi-keyed
 * build gate does NOT limit this engine to WiFi traffic (it serves every compressed
 * path, BLE included). Compiled to an empty TU when the gate is off (nRF52840 /
 * classic ESP32), so miniz.h is only pulled in on ROM-tinfl builds.
 */

#include "od_inflate_tinfl.h"

#if OPENDISPLAY_USE_TINFL

#include <string.h>
#include "miniz.h"   /* ROM tinfl on S3/C3/C6 (symbol via <chip>.rom.ld, header via esp_rom) */

/* ------------------------------------------------------- dictionary/ring size ---
 * This buffer serves TWO roles with DIFFERENT size requirements:
 *
 *  1) LZ77 history — the correctness FLOOR, equal to the DEFLATE window. A match can
 *     never reference further back than the window, so bytes beyond it are unreachable
 *     (never read as history). tinfl additionally REJECTS a stream whose CMF byte
 *     declares a window larger than this buffer — a clean decode error at the zlib
 *     header, never silent corruption — so the size must be >= the window.
 *
 *  2) Output staging ring — SPEED. tinfl's bulk paths need CONTIGUOUS headroom to the
 *     ring end: symbol decode wants a couple of bytes, and the 8-bytes-at-a-time match
 *     copy wants headroom >= the match length. DEFLATE's max match is 258 bytes
 *     (RFC 1951 length codes 257..285; cf. length_base[] in lib/uzlib/src/od_zlib_stream.c),
 *     so at a 512-byte ring a long match is often too close to the wrap and falls back
 *     to the slow byte-at-a-time copy. The share of output affected scales as
 *     match_len/ring_size, i.e. it halves per doubling with no threshold — ~50% at 512
 *     vs ~6% at 4096 for worst-case 258-byte matches. E-paper images have large uniform
 *     regions, which compress into long (often max-length) matches, so the worst case is
 *     representative here rather than rare.
 *
 * Sizing rule:
 *   - 9-bit window (512, the default): use 4096. The window alone leaves too little
 *     staging headroom; 4096 restores the bulk paths while still reclaiming ~28 KB of
 *     internal DRAM versus the old unconditional 32768.
 *   - window > 9 bits: use the window size exactly. It already provides ample headroom,
 *     so padding beyond the correctness floor would only waste DRAM.
 *     (env:esp32-s3-E1004 pins BITS=15 => 32768, unchanged.)
 *
 * Overridable per-env via -DOD_TINFL_DICT_SIZE=<power of 2> to trade RAM against speed.
 *
 * NOTE: the dictionary is OURS, not tinfl's — `tinfl_decompressor` holds only the
 * Huffman tables, and tinfl takes the ring as a caller-supplied buffer, so this array
 * is the whole cost. TINFL_LZ_DICT_SIZE (32768) is a bare SDK #define that allocates
 * nothing; it is deliberately NOT redefined here.
 */
#ifndef OD_TINFL_DICT_SIZE
#  if OPENDISPLAY_ZLIB_WINDOW_BITS == 9
#    define OD_TINFL_DICT_SIZE 4096u                          /* staging headroom */
#  else
#    define OD_TINFL_DICT_SIZE OPENDISPLAY_ZLIB_WINDOW_SIZE   /* window is ample */
#  endif
#endif
/* Power of 2: tinfl derives its wrap mask from the buffer geometry passed in.
 * >= window: else tinfl rejects the stream at the zlib header (see (1) above). */
#if (OD_TINFL_DICT_SIZE & (OD_TINFL_DICT_SIZE - 1u)) != 0
#error "OD_TINFL_DICT_SIZE must be a power of 2"
#endif
#if OD_TINFL_DICT_SIZE < OPENDISPLAY_ZLIB_WINDOW_SIZE
#error "OD_TINFL_DICT_SIZE must be >= OPENDISPLAY_ZLIB_WINDOW_SIZE (the DEFLATE window)"
#endif

/* ------------------------------------------------------------------ state ---
 * All static: plain arrays land in internal .bss/DRAM automatically, which is
 * exactly what the history/output ring needs (fast match reads). The framebuffer
 * is in PSRAM, so we decode into this SRAM ring and let the caller flush each
 * delivered burst sequentially to PSRAM — never decode directly into PSRAM.
 * s_dict is explicitly aligned: tinfl's fast match-copy path does 32-bit loads and
 * stores through this pointer, which a 32 KB array got incidentally but a 512-byte
 * one should not rely on.
 */
static tinfl_decompressor s_decomp;                 /* ~11 KB Huffman tables + bit buffer */
static uint8_t            s_dict[OD_TINFL_DICT_SIZE] __attribute__((aligned(16)));
                                                    /* LZ77 history AND output ring */

static size_t   s_dict_ofs;      /* tinfl next-write position in the ring */
static size_t   s_deliver_ofs;   /* start of the undelivered (pending) region */
static size_t   s_pending;       /* bytes decoded but not yet copied to the caller (contiguous) */

static const uint8_t *s_in;      /* staged input (current frame) */
static size_t   s_in_remaining;
static bool     s_more_input;    /* = !final of the current push (controls HAS_MORE_INPUT) */

static uint32_t s_expected;      /* expected decompressed size (parity check with uzlib) */
static uint32_t s_produced;      /* total bytes decoded so far */
static bool     s_done;
static bool     s_initialized;
static const char *s_error;

extern "C" {

void od_inflate_tinfl_reset(uint32_t expected_output_size) {
    tinfl_init(&s_decomp);
    s_dict_ofs = 0;
    s_deliver_ofs = 0;
    s_pending = 0;
    s_in = NULL;
    s_in_remaining = 0;
    s_more_input = true;
    s_expected = expected_output_size;
    s_produced = 0;
    s_done = false;
    s_error = NULL;
    s_initialized = true;
    /* s_dict is not cleared: tinfl only reads history bytes it has written. */
}

od_zlib_status_t od_inflate_tinfl_push(const uint8_t *input, size_t len, bool final) {
    if (!s_initialized) { s_error = "tinfl stream not initialized"; return OD_ZLIB_STATUS_ERROR; }
    if (s_error) return OD_ZLIB_STATUS_ERROR;
    if (len > 0 && input == NULL) { s_error = "tinfl stream input is null"; return OD_ZLIB_STATUS_ERROR; }
    if (s_in_remaining > 0) { s_error = "previous input not fully consumed"; return OD_ZLIB_STATUS_ERROR; }
    if (s_done) {
        if (len != 0) { s_error = "input after end of zlib stream"; return OD_ZLIB_STATUS_ERROR; }
        return OD_ZLIB_STATUS_DONE;
    }
    s_in = input;
    s_in_remaining = len;
    s_more_input = !final;
    return OD_ZLIB_STATUS_NEEDS_INPUT;
}

od_zlib_status_t od_inflate_tinfl_poll(uint8_t *output, size_t capacity, size_t *produced) {
    *produced = 0;
    if (!s_initialized) { s_error = "tinfl stream not initialized"; return OD_ZLIB_STATUS_ERROR; }
    if (s_error) return OD_ZLIB_STATUS_ERROR;

    for (;;) {
        /* 1) Deliver already-decoded bytes to the caller first. The pending region is
         *    always contiguous (each decode is bounded to the ring end), so this is a
         *    plain memcpy — no ring wrap to handle here. */
        if (s_pending > 0) {
            size_t room = capacity - *produced;
            if (room == 0) return OD_ZLIB_STATUS_OUTPUT_READY;
            size_t n = (s_pending < room) ? s_pending : room;
            memcpy(output + *produced, s_dict + s_deliver_ofs, n);
            *produced += n;
            s_deliver_ofs += n;
            s_pending -= n;
            if (s_pending > 0) return OD_ZLIB_STATUS_OUTPUT_READY;  /* caller's buffer is full */
        }

        /* pending == 0 here */
        if (s_done) {
            if (s_expected != 0 && s_produced != s_expected) {
                s_error = "tinfl decompressed output size mismatch";
                return OD_ZLIB_STATUS_ERROR;
            }
            return OD_ZLIB_STATUS_DONE;
        }
        if (*produced == capacity) return OD_ZLIB_STATUS_OUTPUT_READY;

        /* 2) Decode the next contiguous burst into the ring (only when drained, so tinfl
         *    can never overwrite bytes we have not delivered yet). Bound output to the
         *    room up to the ring end; the wrap is handled by re-entering the loop. */
        size_t in_bytes = s_in_remaining;
        size_t out_bytes = OD_TINFL_DICT_SIZE - s_dict_ofs;
        const mz_uint32 flags = (mz_uint32)(TINFL_FLAG_PARSE_ZLIB_HEADER |
                                            (s_more_input ? TINFL_FLAG_HAS_MORE_INPUT : 0));
        s_deliver_ofs = s_dict_ofs;  /* the pending region will start where tinfl writes */

        tinfl_status st = tinfl_decompress(&s_decomp,
                                           (const mz_uint8 *)s_in, &in_bytes,
                                           (mz_uint8 *)s_dict, (mz_uint8 *)(s_dict + s_dict_ofs),
                                           &out_bytes, flags);

        s_in += in_bytes;
        s_in_remaining -= in_bytes;
        s_produced += (uint32_t)out_bytes;
        s_pending = out_bytes;
        s_dict_ofs = (s_dict_ofs + out_bytes) & (OD_TINFL_DICT_SIZE - 1u);

        if (st < TINFL_STATUS_DONE) {  /* negative status codes are fatal */
            s_error = (st == TINFL_STATUS_ADLER32_MISMATCH) ? "tinfl adler32 mismatch"
                    : (st == TINFL_STATUS_BAD_PARAM)        ? "tinfl bad param"
                    :                                         "tinfl inflate failed";
            return OD_ZLIB_STATUS_ERROR;
        }
        if (st == TINFL_STATUS_DONE) {
            s_done = true;
            continue;  /* re-enter loop to deliver the final burst, then return DONE */
        }
        /* NEEDS_MORE_INPUT: staged frame consumed. If nothing was produced to deliver,
         * ask the caller for the next frame; otherwise loop to deliver first. */
        if (st == TINFL_STATUS_NEEDS_MORE_INPUT && s_pending == 0) {
            return OD_ZLIB_STATUS_NEEDS_INPUT;
        }
        /* HAS_MORE_OUTPUT (ring end reached) or NEEDS_MORE_INPUT with pending>0:
         * loop to deliver, then continue decoding / request input. */
    }
}

const char *od_inflate_tinfl_error(void) { return s_error ? s_error : ""; }

uint32_t od_inflate_tinfl_output_count(void) { return s_produced; }

}  /* extern "C" */

#endif /* OPENDISPLAY_USE_TINFL */
