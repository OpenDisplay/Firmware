#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zlib.h>

#include "uzlib.h"

static int make_zlib(const uint8_t *src, size_t src_len, int window_bits, int level, int strategy, uint8_t **out, size_t *out_len) {
    z_stream zs;
    size_t cap = src_len + 1024;
    int rc;
    memset(&zs, 0, sizeof(zs));
    *out = (uint8_t *)malloc(cap);
    if (!*out) return 0;

    rc = deflateInit2(&zs, level, Z_DEFLATED, window_bits, 8, strategy);
    if (rc != Z_OK) return 0;
    zs.next_in = (Bytef *)src;
    zs.avail_in = (uInt)src_len;
    zs.next_out = *out;
    zs.avail_out = (uInt)cap;
    rc = deflate(&zs, Z_FINISH);
    if (rc != Z_STREAM_END) {
        deflateEnd(&zs);
        return 0;
    }
    *out_len = zs.total_out;
    deflateEnd(&zs);
    return 1;
}

static int run_decode_case(const char *name, const uint8_t *compressed, size_t compressed_len,
                           const uint8_t *expected, size_t expected_len,
                           size_t input_chunk, size_t output_chunk) {
    uint8_t *actual = (uint8_t *)malloc(expected_len ? expected_len : 1);
    size_t actual_len = 0;
    size_t pos = 0;
    bool final_sent = false;
    od_zlib_status_t status = OD_ZLIB_STATUS_NEEDS_INPUT;

    if (!actual) return 0;
    od_zlib_stream_reset((uint32_t)expected_len);

    while (status != OD_ZLIB_STATUS_DONE) {
        if (status == OD_ZLIB_STATUS_NEEDS_INPUT) {
            if (pos < compressed_len) {
                size_t n = compressed_len - pos;
                if (n > input_chunk) n = input_chunk;
                status = od_zlib_stream_push(compressed + pos, n, false);
                pos += n;
            } else if (!final_sent) {
                status = od_zlib_stream_push(NULL, 0, true);
                final_sent = true;
            } else {
                fprintf(stderr, "%s: decoder requested input after final\n", name);
                free(actual);
                return 0;
            }
            if (status == OD_ZLIB_STATUS_ERROR) {
                fprintf(stderr, "%s: push error: %s\n", name, od_zlib_stream_error());
                free(actual);
                return 0;
            }
        }

        for (;;) {
            uint8_t outbuf[4096];
            size_t produced = 0;
            size_t cap = output_chunk < sizeof(outbuf) ? output_chunk : sizeof(outbuf);
            status = od_zlib_stream_poll(outbuf, cap, &produced);
            if (produced > 0) {
                if (actual_len + produced > expected_len) {
                    fprintf(stderr, "%s: produced too much output\n", name);
                    free(actual);
                    return 0;
                }
                memcpy(actual + actual_len, outbuf, produced);
                actual_len += produced;
            }
            if (status == OD_ZLIB_STATUS_OUTPUT_READY) continue;
            break;
        }

        if (status == OD_ZLIB_STATUS_ERROR) {
            fprintf(stderr, "%s: poll error: %s (out=%u expected=%zu)\n", name, od_zlib_stream_error(), od_zlib_stream_output_count(), expected_len);
            free(actual);
            return 0;
        }
    }

    if (actual_len != expected_len || memcmp(actual, expected, expected_len) != 0) {
        fprintf(stderr, "%s: output mismatch (%zu != %zu)\n", name, actual_len, expected_len);
        size_t limit = actual_len < expected_len ? actual_len : expected_len;
        for (size_t i = 0; i < limit; ++i) {
            if (actual[i] != expected[i]) {
                fprintf(stderr, "%s: first diff at %zu actual=%02x expected=%02x\n", name, i, actual[i], expected[i]);
                break;
            }
        }
        free(actual);
        return 0;
    }
    free(actual);
    return 1;
}

static int expect_error(const char *name, const uint8_t *compressed, size_t compressed_len, size_t expected_len) {
    size_t pos = 0;
    od_zlib_status_t status = OD_ZLIB_STATUS_NEEDS_INPUT;
    od_zlib_stream_reset((uint32_t)expected_len);
    while (status != OD_ZLIB_STATUS_DONE) {
        if (status == OD_ZLIB_STATUS_NEEDS_INPUT) {
            size_t n = compressed_len - pos;
            if (n > 3) n = 3;
            status = od_zlib_stream_push(pos < compressed_len ? compressed + pos : NULL, pos < compressed_len ? n : 0, pos >= compressed_len);
            pos += n;
            if (status == OD_ZLIB_STATUS_ERROR) return 1;
        }
        uint8_t outbuf[7];
        size_t produced = 0;
        status = od_zlib_stream_poll(outbuf, sizeof(outbuf), &produced);
        if (status == OD_ZLIB_STATUS_ERROR) return 1;
        if (pos >= compressed_len && status == OD_ZLIB_STATUS_NEEDS_INPUT) {
            status = od_zlib_stream_push(NULL, 0, true);
        }
    }
    fprintf(stderr, "%s: expected error but decode succeeded\n", name);
    return 0;
}

static int expect_error_one_push(const char *name, const uint8_t *compressed, size_t compressed_len, size_t expected_len) {
    od_zlib_status_t status;
    od_zlib_stream_reset((uint32_t)expected_len);
    status = od_zlib_stream_push(compressed, compressed_len, true);
    if (status == OD_ZLIB_STATUS_ERROR) return 1;
    while (status != OD_ZLIB_STATUS_DONE) {
        uint8_t outbuf[4096];
        size_t produced = 0;
        status = od_zlib_stream_poll(outbuf, sizeof(outbuf), &produced);
        if (status == OD_ZLIB_STATUS_ERROR) return 1;
    }
    fprintf(stderr, "%s: expected error but decode succeeded\n", name);
    return 0;
}

static void fill_fixture(uint8_t *buf, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        if ((i % 97) < 64) buf[i] = (uint8_t)('A' + (i % 5));
        else buf[i] = (uint8_t)((i * 37u + i / 3u) & 0xffu);
    }
}

int main(void) {
    uint8_t src[8192];
    uint8_t *compressed = NULL;
    size_t compressed_len = 0;
    int ok = 1;
    fill_fixture(src, sizeof(src));

    struct {
        const char *name;
        int level;
        int strategy;
    } cases[] = {
        {"stored", 0, Z_DEFAULT_STRATEGY},
        {"fixed", 6, Z_FIXED},
        {"dynamic", 9, Z_DEFAULT_STRATEGY},
    };

    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        if (!make_zlib(src, sizeof(src), 9, cases[i].level, cases[i].strategy, &compressed, &compressed_len)) {
            fprintf(stderr, "failed to create %s fixture\n", cases[i].name);
            return 1;
        }
        ok &= run_decode_case(cases[i].name, compressed, compressed_len, src, sizeof(src), 1, 1);
        ok &= run_decode_case(cases[i].name, compressed, compressed_len, src, sizeof(src), 23, 7);
        ok &= run_decode_case(cases[i].name, compressed, compressed_len, src, sizeof(src), compressed_len, 4096);
        free(compressed);
        compressed = NULL;
    }

    if (!make_zlib(src, sizeof(src), 10, 6, Z_DEFAULT_STRATEGY, &compressed, &compressed_len)) {
        fprintf(stderr, "failed to create ws10 fixture\n");
        return 1;
    }
    ok &= expect_error("oversized-window", compressed, compressed_len, sizeof(src));
    free(compressed);

    if (!make_zlib(src, sizeof(src), 9, 6, Z_DEFAULT_STRATEGY, &compressed, &compressed_len)) {
        fprintf(stderr, "failed to create corrupt fixtures\n");
        return 1;
    }
    ok &= expect_error("truncated", compressed, compressed_len / 2, sizeof(src));
    compressed[compressed_len - 1] ^= 0x55;
    ok &= expect_error("bad-adler", compressed, compressed_len, sizeof(src));
    free(compressed);

    if (!make_zlib(src, sizeof(src), 9, 6, Z_DEFAULT_STRATEGY, &compressed, &compressed_len)) {
        fprintf(stderr, "failed to create trailing fixture\n");
        return 1;
    }
    uint8_t *with_trailing = (uint8_t *)malloc(compressed_len + 1);
    if (!with_trailing) return 1;
    memcpy(with_trailing, compressed, compressed_len);
    with_trailing[compressed_len] = 0xaa;
    ok &= expect_error_one_push("trailing-input", with_trailing, compressed_len + 1, sizeof(src));
    free(with_trailing);
    free(compressed);

    if (!ok) return 1;
    printf("zlib stream tests passed\n");
    return 0;
}
