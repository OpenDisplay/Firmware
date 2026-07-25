# Plan: Transparent swap to ROM `tinfl` for the ESP32-WiFi inflate path

## Context

On the WiFi/LAN transport, compressed image uploads are **slower** than uncompressed
despite a ~4:1 wire saving. Root cause (traced this session): the transfer is
**consumer-bound on software inflate**, not wire-bound. The active inflater is a
hand-rolled, fully bit-serial, resumable DEFLATE state machine
([lib/uzlib/src/od_zlib_stream.c](../lib/uzlib/src/od_zlib_stream.c), from PR #26):
per-bit reads, per-symbol Huffman walk, one-byte-at-a-time output, and a `% 65521`
Adler-32 reduction on *every* byte. It was designed BLE-first — where wire speed is far
below inflate speed, so decode was never the bottleneck. WiFi is ~10–100× faster, so
inflate becomes the limiter. Crossover: compression only wins when
`inflate_rate > ~1.33 × wire_rate`; the current engine fails that on LAN.

The three WiFi chips (ESP32-S3 / C3 / C6) carry **miniz `tinfl_decompress` in mask ROM**
(fixed addresses in `<chip>.rom.ld`; `esp_rom/include/miniz.h` ships with the framework)
— a word-at-a-time, table-driven inflater. Using it costs **0 bytes of flash**. The S3
framebuffer is in **PSRAM**, so the design keeps the 32 KB history/output ring in
**internal SRAM** (fast match reads) and flushes decoded bursts **sequentially** to the
PSRAM framebuffer — never decoding directly into PSRAM.

Intended outcome: on ESP32-WiFi builds, compressed transfers decode several× faster,
flipping compression back to a net win on LAN — with **no protocol/wire change** and no
flash growth. nRF52840 and classic ESP32 (no WiFi) keep uzlib.

## Hard constraint

**`lib/uzlib/` must not be modified at all** — no edits to `od_zlib_stream.c`, `uzlib.h`,
or any file under it, and no new files added there. The swap therefore happens **one level
up**, in the firmware's own adapter layer in `src/`. uzlib stays compiled and byte-for-byte
intact; it is simply not *called* on WiFi builds (unused `od_zlib_stream_*` get dropped by
`--gc-sections`). All firmware callers of the inflater live in **one file**,
[src/display_service.cpp](../src/display_service.cpp) (`od_zlib_stream_reset` ×3, and
`push`/`poll`/`error` inside `zlib_stream_to_direct_write` / `zlib_stream_to_partial_write`) —
confirmed the only call sites in `src/`.

## Approach: src-level tinfl implementation + compile-time remap in display_service.cpp

Provide a tinfl-backed implementation of the same streaming contract under new names in
`src/`, then bind the existing call sites to it on WiFi builds via a small `#define` remap —
so the ~11 existing call sites in `display_service.cpp` are **not edited**. The swap is
invisible above the adapter layer; the wire protocol, `communication.cpp` dispatch, framing,
and py-opendisplay are untouched. Because the gate is per-build (chip family), on WiFi-capable
ESP32 builds *all* compressed transfers (BLE + LAN) use tinfl — a superset of uzlib's behavior.

### The gate

Reuse the exact condition that already defines `OPENDISPLAY_HAS_WIFI`
([wifi_service.h:13](../src/wifi_service.h#L13)): `TARGET_ESP32 && OPENDISPLAY_ENABLE_WIFI`.
Covers all S3/C6/C3 WiFi envs (incl. `esp32-s3-E1004`, which extends `…-N32R8-extuart`) and
excludes `nrf52840custom` and `esp32-N4`. No `platformio.ini` change — `src/` files compile
automatically and the flag already exists.

## Files to change (2 new in `src/`, 1 edited; zero uzlib/platformio changes)

1. **`src/od_inflate_tinfl.h`** (new) — declares the 5 functions
   (`od_inflate_tinfl_reset/push/poll/error/output_count`) and the `OPENDISPLAY_USE_TINFL`
   gate macro. Reuses `od_zlib_status_t` / `OD_ZLIB_STATUS_*` by `#include "uzlib.h"` (include
   only — no modification), so the tinfl path returns the identical status type the callers
   already switch on.

2. **`src/od_inflate_tinfl.cpp`** (new) — body wrapped in `#if OPENDISPLAY_USE_TINFL`.
   `#include "miniz.h"`. Implements the tinfl wrapper over ROM `tinfl_decompress`.

3. **[src/display_service.cpp](../src/display_service.cpp)** — add, right after the includes
   block (after line 18), ~8 lines:
   ```c
   #include "od_inflate_tinfl.h"
   #if OPENDISPLAY_USE_TINFL
   // Route the inflate adapter to the ROM-tinfl engine on ESP32-WiFi builds; uzlib
   // (lib/uzlib) is left untouched and unused here. See od_inflate_tinfl.h.
   #define od_zlib_stream_reset        od_inflate_tinfl_reset
   #define od_zlib_stream_push         od_inflate_tinfl_push
   #define od_zlib_stream_poll         od_inflate_tinfl_poll
   #define od_zlib_stream_error        od_inflate_tinfl_error
   #endif
   ```
   The macro is placed *after* `#include "uzlib.h"` (line 15) so the existing
   `od_zlib_stream_*` call sites (2076, 2245, 2778, 3120–3177) bind to the tinfl impl with **no
   edits to those lines**. `od_zlib_status_t` and `OD_ZLIB_STATUS_*` stay as-is (shared type).
   Gate off (nRF / classic ESP32): the macros vanish and everything calls uzlib exactly as today.

### tinfl wrapper design (`od_inflate_tinfl.cpp`; static BSS — chosen)

File-scope `static` state (plain arrays land in internal `.bss`/DRAM automatically, satisfying
"dict must be internal SRAM" with zero effort; ~43 KB permanent on gated builds):
- `tinfl_decompressor s_decomp;` (~11 KB)
- `uint8_t s_dict[TINFL_LZ_DICT_SIZE];` (32768 — history + output ring)
- ring/delivery cursors `s_dict_ofs`, `s_deliver_ofs`, `s_pending`; input staging
  `s_in`, `s_in_remaining`, `s_more_input`; bookkeeping `s_expected`, `s_produced`,
  `s_done`, `s_initialized`, `s_error`.

- **`od_inflate_tinfl_reset(expected)`**: `tinfl_init(&s_decomp)`; zero cursors/counters;
  store `s_expected`; clear error/done; `s_initialized=true`.
- **`od_inflate_tinfl_push(input,len,final)`**: mirror uzlib semantics (error if not
  initialized / previous input unconsumed); stash input; `s_more_input = !final` → maps the
  `final` flag to clearing `TINFL_FLAG_HAS_MORE_INPUT` so tinfl finalizes + verifies Adler-32
  on the last frame (the empty `push(NULL,0,true)` at
  [display_service.cpp:2368](../src/display_service.cpp#L2368) resolves to DONE).
- **`od_inflate_tinfl_poll(output,capacity,produced)`** — rate-matching core:
  - **Deliver first, decode only when drained.** While `s_pending>0`, `memcpy` a contiguous run
    from `s_dict+s_deliver_ofs` into `output` (bounded by `capacity`); advance cursors; return
    `OD_ZLIB_STATUS_OUTPUT_READY` when `output` fills.
  - When `s_pending==0`, call `tinfl_decompress(&s_decomp, s_in, &in_bytes, s_dict,
    s_dict+s_dict_ofs, &out_bytes, TINFL_FLAG_PARSE_ZLIB_HEADER | (s_more_input ?
    TINFL_FLAG_HAS_MORE_INPUT : 0))` with `out_bytes` bounded to **contiguous room to ring end**
    (`32768 - s_dict_ofs`). One contiguous burst per decode, fully delivered before the next
    decode → tinfl never overwrites undelivered bytes and delivery stays a simple contiguous copy.
  - Advance `s_in`, add to `s_produced`/`s_pending`, wrap `s_dict_ofs &= 32767`.
  - Status map: `<0` → `OD_ZLIB_STATUS_ERROR` (set `s_error`); `TINFL_STATUS_DONE(0)` → mark done,
    return DONE when drained; `NEEDS_MORE_INPUT(1)` drained → `OD_ZLIB_STATUS_NEEDS_INPUT`;
    `HAS_MORE_OUTPUT(2)` → loop to deliver.
- **`od_inflate_tinfl_error()` / `_output_count()`**: return `s_error` / `s_produced`.

tinfl API confirmed present in the ROM header: `tinfl_init` macro (miniz.h:587), status enum
`DONE=0 / NEEDS_MORE_INPUT=1 / HAS_MORE_OUTPUT=2 / negatives` (miniz.h:578–583),
`TINFL_LZ_DICT_SIZE 32768`, `TINFL_FLAG_*`, full `tinfl_decompressor` struct.

### Intended, documented behavior differences (benign / desirable)
- **Window ceiling relaxed.** tinfl always supports up to a 32 KB window, so it accepts any
  standard zlib stream regardless of `OPENDISPLAY_ZLIB_WINDOW_BITS`. Today's sender emits
  `wbits=9`, which still decodes; this additionally *unlocks* a future `wbits=15` sender change
  for a better ratio (py-opendisplay, out of scope).
- **Adler-32 handled by tinfl** via `TINFL_FLAG_PARSE_ZLIB_HEADER` (no per-byte modulo).

## Out of scope
- No changes to `lib/uzlib/` (hard constraint), `communication.cpp`, the wire protocol, framing,
  `platformio.ini`, or py-opendisplay.
- No BLE-only carve-out — the build gate naturally includes BLE on WiFi builds (superset; no regress).

## Verification

1. **Include-path smoke test (first).** Confirm `#include "miniz.h"` resolves for a `src/`
   `.cpp` on the S3 env (ROM header at `.../esp32s3/include/esp_rom/include/miniz.h`; symbol
   guaranteed by `<chip>.rom.ld`). If bare include fails, add the `esp_rom/include` dir to the
   env `build_flags` `-I`, or include via its resolvable path.
2. **Builds (keep CI green).**
   - `pio run -e esp32-s3-N16R8` and `pio run -e esp32-c3-N16` → compile with tinfl remap;
     `.bss` grows ~43 KB, flash does **not** grow for inflate code.
   - `pio run -e nrf52840custom` → gate off; compiles/links against uzlib exactly as today.
3. **uzlib untouched.** `git diff` shows zero changes under `lib/uzlib/`. Host unit test
   `tools/test_zlib_stream.c` (uzlib impl, gate off on host) still builds/passes.
4. **Functional on hardware (S3, PSRAM framebuffer).** Flash `esp32-s3-N16R8`; send a compressed
   LAN direct-write image via py-opendisplay. Confirm correct render (Adler-32 passes;
   `directWriteBytesWritten == directWriteDecompressedTotal`) and that the
   `DW complete … zlib <N> B on wire (<x>x)` log ([display_service.cpp:1884](../src/display_service.cpp#L1884))
   shows the expected ratio. Also exercise the **partial** (0x76) and **pipe** (0x80) compressed
   paths, and a plain **BLE** compressed transfer (now also tinfl on this build).
5. **Perf confirmation (no new instrumentation).** Use the existing `DW complete … chunks …
   <rate> KB/s` log ([display_service.cpp:1873](../src/display_service.cpp#L1873)): send the same
   image on a tinfl build vs a uzlib build and confirm the compressed-LAN rate rises and now
   beats the uncompressed-LAN rate (`inflate_rate > ~1.33 × wire_rate`).
