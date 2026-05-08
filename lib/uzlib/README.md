# uzlib

Vendored copy of `pfalcon/uzlib` for OpenDisplay firmware.

Upstream: https://github.com/pfalcon/uzlib
Commit: `6d60d651a4499a64f2e5b21b4cc08d98cb84b5c1`

Only the files needed for current zlib/DEFLATE inflate usage are included:

- `tinflate.c`
- `tinfzlib.c`
- `adler32.c`
- `crc32.c`
- public headers required by those sources

Local changes should be kept in this folder so the firmware build is
self-contained and future streaming/nonblocking inflater patches can be
reviewed together with the firmware code.
