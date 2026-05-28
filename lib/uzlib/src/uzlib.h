/*
 * OpenDisplay streaming zlib inflater, based on uzlib/tinf.
 *
 * The original uzlib one-shot/callback inflater API is intentionally not
 * exposed in this vendored copy. Firmware uses one global streaming inflater.
 */

#ifndef UZLIB_H_INCLUDED
#define UZLIB_H_INCLUDED

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "uzlib_conf.h"

#ifndef OPENDISPLAY_ZLIB_WINDOW_BITS
#define OPENDISPLAY_ZLIB_WINDOW_BITS 9
#endif

#if OPENDISPLAY_ZLIB_WINDOW_BITS < 9 || OPENDISPLAY_ZLIB_WINDOW_BITS > 15
#error "OPENDISPLAY_ZLIB_WINDOW_BITS must be in range 9..15"
#endif

#define OPENDISPLAY_ZLIB_WINDOW_SIZE (1u << OPENDISPLAY_ZLIB_WINDOW_BITS)

typedef enum {
    OD_ZLIB_STATUS_NEEDS_INPUT = 0,
    OD_ZLIB_STATUS_OUTPUT_READY = 1,
    OD_ZLIB_STATUS_DONE = 2,
    OD_ZLIB_STATUS_ERROR = -1,
} od_zlib_status_t;

void od_zlib_stream_reset(uint32_t expected_output_size);
od_zlib_status_t od_zlib_stream_push(const uint8_t *input, size_t len, bool final);
od_zlib_status_t od_zlib_stream_poll(uint8_t *output, size_t capacity, size_t *produced);
const char *od_zlib_stream_error(void);
uint32_t od_zlib_stream_output_count(void);

uint32_t uzlib_adler32(const void *data, unsigned int length, uint32_t prev_sum);
uint32_t uzlib_crc32(const void *data, unsigned int length, uint32_t crc);

#ifdef __cplusplus
}
#endif

#endif /* UZLIB_H_INCLUDED */
