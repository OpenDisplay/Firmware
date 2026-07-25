/*
 * od_inflate_tinfl — ESP32 inflate adapter backed by the ROM miniz `tinfl`.
 *
 * Drop-in replacement for the uzlib streaming inflater (lib/uzlib,
 * od_zlib_stream_*). It exposes the SAME streaming contract (reset / push / poll /
 * error / output_count) and the SAME status type (od_zlib_status_t, reused from
 * uzlib.h) so display_service.cpp can bind its existing od_zlib_stream_* call sites
 * to this engine via a compile-time #define remap, with no changes to lib/uzlib and
 * no changes to the call sites.
 *
 * SCOPE — this serves EVERY compressed path, not just WiFi/LAN. The remap in
 * display_service.cpp is an unconditional compile-time #define, so when this engine
 * is enabled it replaces uzlib for direct-write, partial-region, AND PIPE_WRITE
 * regardless of transport. PIPE_WRITE is in fact BLE-only (see "NO PIPE ON LAN" in
 * opendisplay_protocol.h), so the busiest BLE transfer path decodes through tinfl.
 * The gate below keys off OPENDISPLAY_ENABLE_WIFI only as a proxy for "a build that
 * cares about inflate throughput and can spare the RAM" — it does NOT mean the
 * engine is limited to WiFi traffic. Do not read the gate as a transport filter.
 *
 * WHY: the uzlib engine is a bit-serial, byte-at-a-time resumable state machine —
 * tolerable for BLE (wire << inflate), but the LAN wire is ~10-100x faster so
 * software inflate becomes the bottleneck and compression turns into a net loss.
 * `tinfl` is word-at-a-time + table-driven and lives in mask ROM on S3/C3/C6 (fixed
 * addresses in <chip>.rom.ld), so it is faster at zero flash cost. BLE transfers get
 * the same speedup for free; the LAN wire is just what made it necessary.
 *
 * COST: ~11 KB of internal DRAM for the Huffman tables (tinfl_decompressor), plus a
 * caller-supplied LZ dictionary/output ring (see OD_TINFL_DICT_SIZE in
 * od_inflate_tinfl.cpp — 4 KB at the default 9-bit window, sized for decode-path
 * headroom rather than the 32 KB the TINFL_LZ_DICT_SIZE constant suggests; a window
 * wider than 9 bits sizes it to the window instead). vs ~2.5 KB for uzlib.
 *
 * The status enum (od_zlib_status_t, OD_ZLIB_STATUS_*) is defined by uzlib.h; we
 * only include it, never modify it.
 */

#ifndef OD_INFLATE_TINFL_H
#define OD_INFLATE_TINFL_H

#include "uzlib.h"   /* od_zlib_status_t + OD_ZLIB_STATUS_* (include only, not modified) */

/* Gate: which BUILDS use this engine — NOT which transports it serves (see SCOPE
 * above; once enabled it handles BLE too). Mirrors the condition that defines
 * OPENDISPLAY_HAS_WIFI (src/wifi_service.h) because a LAN-capable build is the one
 * that needs the throughput. ROM tinfl itself exists on S3/C3/C6 regardless.
 * Overridable via a build flag if a target ever needs to force one engine or the
 * other, e.g. -DOPENDISPLAY_USE_TINFL=0 to keep uzlib's smaller footprint. */
#if !defined(OPENDISPLAY_USE_TINFL)
#  if defined(TARGET_ESP32) && defined(OPENDISPLAY_ENABLE_WIFI)
#    define OPENDISPLAY_USE_TINFL 1
#  else
#    define OPENDISPLAY_USE_TINFL 0
#  endif
#endif

#if OPENDISPLAY_USE_TINFL

#ifdef __cplusplus
extern "C" {
#endif

/* Same semantics/signatures as od_zlib_stream_* in uzlib.h. */
void od_inflate_tinfl_reset(uint32_t expected_output_size);
od_zlib_status_t od_inflate_tinfl_push(const uint8_t *input, size_t len, bool final);
od_zlib_status_t od_inflate_tinfl_poll(uint8_t *output, size_t capacity, size_t *produced);
const char *od_inflate_tinfl_error(void);
uint32_t od_inflate_tinfl_output_count(void);

#ifdef __cplusplus
}
#endif

#endif /* OPENDISPLAY_USE_TINFL */

#endif /* OD_INFLATE_TINFL_H */
