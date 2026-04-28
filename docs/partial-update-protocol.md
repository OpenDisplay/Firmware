# Partial Update Protocol

Partial updates use a streamed single-rectangle protocol:

```text
0x76 PARTIAL_IMAGE_START
0x71 DATA...
0x72 END + partial refresh
```

Full uploads continue to use `0x70`, `0x71`, and `0x72`. `0x77` is unused.

## `0x76` Partial Start

```text
[0x0076]
[version:1 = 0x01]
[flags:2 BE]
[old_etag:4 BE]
[x:2 BE][y:2 BE][width:2 BE][height:2 BE]
[interleave_span_pixels:2 BE]
[uncompressed_size:4 LE]
[initial_stream_bytes...]
```

The stream bytes are zlib bytes when `flags & 0x0004` is set, otherwise raw
logical bytes. `uncompressed_size` is always `rect_bytes * 2`.

Flags:

```text
bits 0..1: plane order, 0 = old PLANE_1 then new PLANE_0
bit 2: stream is zlib-compressed
bit 3: 0x72 includes new_etag to store after successful refresh
bit 4: keep panel awake hint
bits 5..15: reserved, must be 0
```

The rectangle must be in bounds. `x` and `width` must be aligned to the active
packed-pixel byte boundary: 8 pixels for 1 bpp, 4 for 2 bpp, 2 for 4 bpp, and
1 for 8 bpp.

## Stream Body

The logical stream contains both old and new rectangle images. In the default
plane order:

```text
old group 0 bytes for PLANE_1
new group 0 bytes for PLANE_0
old group 1 bytes for PLANE_1
new group 1 bytes for PLANE_0
...
```

`interleave_span_pixels` defines each group. Clients initially use row bands,
usually `width * 8` pixels, so each group maps to a simple rectangle.

## `0x71` Data

After `0x76`, `0x71` carries the remaining partial stream bytes. It has no
partial metadata:

```text
[0x0071][stream_bytes...]
```

Current firmware buffers compressed partial stream bytes just like compressed
full uploads, then inflates at `0x72`. Raw partial streams are consumed as
`0x76`/`0x71` bytes arrive.

## `0x72` End

When `flags & 0x0008` was set on `0x76`, the end payload is:

```text
[0x0072][refresh_mode:1][new_etag:4 BE]
```

Firmware validates the logical byte count and per-plane byte counts, then
refreshes with a partial-capable refresh mode. The new etag is stored only
after refresh completion.

Known partial NACK error codes use `{0xFF, opcode, error, 0x00}`:

```text
0x01: etag mismatch
0x02: mixed full/partial data
0x03: rectangle out of bounds
0x04: unsupported partial protocol version
0x05: rectangle alignment error
0x06: unsupported or reserved flags
0x07: uncompressed_size mismatch
0x08: invalid interleave_span_pixels
0x09: stream byte count or content error
```
