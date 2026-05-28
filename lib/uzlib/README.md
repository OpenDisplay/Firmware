# uzlib

Vendored `pfalcon/uzlib` snapshot for OpenDisplay firmware.

Upstream: https://github.com/pfalcon/uzlib
Base commit: `6d60d651a4499a64f2e5b21b4cc08d98cb84b5c1`

This is no longer a full upstream uzlib checkout. It is a trimmed firmware
library with a local OpenDisplay streaming inflater API for chunked BLE/WiFi
image transfers.

## Files Kept

- `src/od_zlib_stream.c` - OpenDisplay push/poll zlib inflater.
- `src/uzlib.h` - public OpenDisplay API.
- `src/uzlib_conf.h` - retained compatibility config header.
- `src/adler32.c` - upstream Adler-32 helper.
- `src/crc32.c` - upstream CRC32 helper.
- `LICENSE` - upstream zlib license.
- `library.json` - PlatformIO library manifest.

## Files Removed

The original upstream inflate/compress APIs and their support files were
removed from this vendored copy. In particular, this folder does not expose the
upstream one-shot/callback decompressor, gzip parser, raw deflate entry points,
or compressor APIs.

Firmware callers should not use upstream symbols such as `uzlib_uncompress()`,
`uzlib_zlib_parse_header()`, `uzlib_gzip_parse_header()`, or `uzlib_compress()`.
Only the OpenDisplay API in `src/uzlib.h` is supported.

## OpenDisplay Streaming API

The firmware uses one global streaming inflater instance:

```c
void od_zlib_stream_reset(uint32_t expected_output_size);
od_zlib_status_t od_zlib_stream_push(const uint8_t *input, size_t len, bool final);
od_zlib_status_t od_zlib_stream_poll(uint8_t *output, size_t capacity, size_t *produced);
const char *od_zlib_stream_error(void);
uint32_t od_zlib_stream_output_count(void);
```

The intended flow is:

1. Call `od_zlib_stream_reset(expected_output_size)`.
2. Call `od_zlib_stream_push(input, len, false)` for each incoming compressed
   chunk.
3. Call `od_zlib_stream_poll(output, capacity, &produced)` until it returns
   `OD_ZLIB_STATUS_NEEDS_INPUT`, `OD_ZLIB_STATUS_DONE`, or
   `OD_ZLIB_STATUS_ERROR`.
4. Pass the last chunk with `final = true`, then keep polling until
   `OD_ZLIB_STATUS_DONE` or `OD_ZLIB_STATUS_ERROR`.
5. On error, use `od_zlib_stream_error()` for a stable diagnostic string.

`od_zlib_stream_push()` rejects new input while previous input is still
unconsumed. Callers must drain with `od_zlib_stream_poll()` before pushing the
next chunk.

## Supported Format

The local inflater accepts zlib-wrapped DEFLATE streams. It supports:

- stored blocks
- fixed Huffman blocks
- dynamic Huffman blocks
- Adler-32 trailer validation
- exact decompressed-size validation against `expected_output_size`

It intentionally rejects:

- gzip streams
- raw DEFLATE streams without a zlib header/trailer
- zlib streams with preset dictionaries
- zlib streams that declare a window larger than the configured firmware limit
- any output beyond `expected_output_size`
- trailing input after a completed zlib stream

## Memory Model

The inflater is stateful and not reentrant. It keeps a single global state
object and avoids allocating a full compressed image buffer or a full
decompressed image buffer.

`OPENDISPLAY_ZLIB_USE_HEAP_WINDOW` controls where the LZ77 history window lives:

- `0` keeps the window in static storage.
- `1` allocates the window from the heap when the stream is reset.

Current ESP32 targets set `OPENDISPLAY_ZLIB_USE_HEAP_WINDOW=1`, which keeps
large backwards-compatible windows out of static DRAM. The nRF52840 target sets
it to `0` for deterministic static allocation.

`OPENDISPLAY_ZLIB_WINDOW_BITS` controls the history window size and defaults to
`9` (`512` bytes). Valid values are `9..15`. Increasing it permits streams
created with larger zlib windows, but increases static RAM use.

The firmware direct-write path currently polls into small caller-owned output
chunks and writes those chunks directly to the display driver.

## Checksum Helpers

`uzlib_adler32()` and `uzlib_crc32()` remain available because they are small
upstream helpers and may be useful for validation. The streaming zlib inflater
maintains its own Adler-32 state internally and validates the zlib trailer.

## Maintenance Notes

- Keep local inflater changes in this folder so firmware code and the modified
  vendored library can be reviewed together.
- Do not add `https://github.com/pfalcon/uzlib` to `lib_deps` for this firmware;
  upstream uzlib does not provide the `od_zlib_stream_*` API.
- If upstream uzlib is refreshed, re-check this README, `src/uzlib.h`, and
  `src/od_zlib_stream.c` together. This folder intentionally diverges from
  upstream API shape.
