# Partial Update Protocol

Partial updates use the direct-write command family with two additional
messages. The current partial protocol version is `1`.

## `0x76` Partial Start

```text
[0x0076][version:1][old_etag:4 BE]
```

The device ACKs with `0x0076` when `old_etag` matches the image currently on
the panel. A NACK `ff 76 01 00` means the client must fall back to a full
upload.

## `0x77` Partial Data

Each `0x77` packet contains one or more complete segments:

```text
[0x0077][segment...]

segment:
x:u16BE y:u16BE width:u16BE height:u16BE flags:u8 payload:N
```

The geometry implies the uncompressed payload size from the active display
encoding. Segments must have `x` and `width` aligned to 8 pixels.

Segment flags:

```text
bit 0: plane select, 0 = PLANE_0/new image, 1 = PLANE_1/old image
bit 1: payload is one complete zlib stream
bits 2-7: reserved, must be 0
```

When bit 1 is clear, `payload` is the raw packed segment bytes. When bit 1 is
set, `payload` is a zlib-compressed stream whose decompressed size must exactly
match the size implied by the segment geometry.

Known partial NACK error codes:

```text
0x01: etag mismatch on 0x76
0x02: mixed full/partial data in one transfer
0x03: invalid segment, out of bounds segment, truncated segment, or malformed compressed payload
0x04: unsupported partial protocol version
0x05: segment x/width alignment error
```
