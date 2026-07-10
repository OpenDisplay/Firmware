# BWR/BWY Direct Write Audit — Findings

**Date:** 2026-07-05
**Scope:** Full code audit of the BWR/BWY (black/white/red, black/white/yellow) BLE direct-write
path in the Firmware repo, verification of external review finding CR-5, and git archaeology to
identify the breaking change.
**Revalidated against `975f14a` (2026-07-05):** After syncing local/origin to upstream
`975f14a` (28 commits ahead of the original audit point `3bf22d3`), the bitplane direct-write
code is **byte-identical** to what was audited — none of the 28 new commits (PRs #50–#81) touched
it. The bug and the fix plan stand unchanged; all line numbers below are updated to `975f14a`.

## Verdict

**CR-5 is CONFIRMED.** BWR/BWY direct write is broken at HEAD and in release 2.0: the firmware
accepts only the black/white plane and discards the red/yellow plane, then refreshes the panel
with stale colour RAM. Compressed BWR/BWY uploads are rejected outright at START.

**The breaking change is `fd0d73a` ("Big Refactor + QR code on boot screen + fix for HA"),
shipped as release 1.3 (2026-03-24).** It was a direct push, not a PR. **No PR since 2.0 broke
the path** — only one boot-screen commit (`3bf22d3`) exists after the 2.0 tag, and the
direct-write code in 2.0 is identical to HEAD. The last release with a working BWR/BWY
direct-write path is **1.2** (`b04a22b`, 2026-03-10).

## Evidence (all verified against current HEAD `3bf22d3`)

### F1. Upload sized at one plane, senders transmit two

`handleDirectWriteStart` sets `directWriteTotalBytes = (pixels + 7) / 8` for bitplane schemes
(`src/display_service.cpp:1474`), i.e. a single 1bpp plane. All three senders transmit **two**
concatenated planes (B/W plane, then R/Y plane):

- **py-opendisplay:** `plane1 + plane2` (`src/opendisplay/device.py:247-249`; encoder
  `src/opendisplay/encoding/bitplanes.py:14-68`)
- **opendisplay-js:** `plane1 ++ plane2` (`src/encoding/images.ts:189-196`; encoder
  `src/encoding/bitplanes.ts:25-61`)
- **website:** `byteData.push(...byteDataPlane1); push(...byteDataPlane2)`
  (`opendisplay.org/httpdocs/js/ble-common.js:3461-3499`)

### F2. Plane switch never happens

The firmware opens `PLANE_0` only (`src/display_service.cpp:1513`). The state flag
`directWritePlane2` (`src/main.h:165`, commented "True when writing plane 2 (R/Y)") is set
`false` at `display_service.cpp:1427` and `:1469` and **never set true anywhere** — dead state.
`git log --all -S 'directWritePlane2'` shows only three commits ever touched the symbol:
`5dcb60b` (introduced, release 0.5), `33f1ff8` (adjusted), `fd0d73a` (logic removed, variable
left behind).

### F3. Uncompressed path silently truncates and auto-refreshes

`handleDirectWriteData` clamps writes to `remainingBytes` (`display_service.cpp:1636-1637`) and
auto-fires `handleDirectWriteEnd` once one plane's worth of bytes arrives (`:1652`). The panel
refreshes with **stale red/yellow RAM**. Subsequent second-plane chunks are ignored without a
response (`directWriteActive` false, `:1631`).

### F4. Compressed path NACKs at START; Python fallback masks the error

The client's `uncompressed_size` (2×plane) fails the equality check against the firmware's
1×plane expectation → `0xFF 0xFF` NACK (`display_service.cpp:1488`). py-opendisplay then
**falls back to uncompressed** (`device.py:1489-1504`) and hits F3 — so Python users get a
silently wrong image rather than an error. The JS clients surface the NACK as a failure.

### F5. Plane order / mapping sanity check

bb_epaper `bbepStartWrite` maps `PLANE_0` → B/W RAM (`SSD1608_WRITE_RAM`, or UC81xx DTM per
`BBEP_RED_SWAPPED`) and `PLANE_1` → red/alt RAM (`bb_ep.inl:4140-4164`). Senders transmit the
B/W plane first, so PLANE_0-then-PLANE_1 streaming order is correct for the fix.

### F6. Not affected

MONO; GRAY4 (has its own correct two-plane streamer `streamGray4Bytes`,
`display_service.cpp:1401-1419`); BWRY/ACeP (single packed 2bpp plane); the Seeed driver path
(no BWR panels); the partial-refresh path (separate context, mono-only by design); firmware-side
rendering (boot screen) which writes both planes via the bb_epaper framebuffer.

## Regression timeline

| Release | Commit | BWR/BWY direct write |
|---|---|---|
| 0.5 – 1.2 | `5dcb60b` … `b04a22b` | **Working** — `main.cpp` handler switches to `PLANE_1` after plane 1, refreshes only after both planes (`5dcb60b:src/main.cpp:2972-3025`) |
| 1.3 | `fd0d73a` (Big Refactor, direct push) | **Broken** — handlers moved to `display_service.cpp`, plane-switch block dropped, `directWritePlane2` kept as dead state |
| 1.4 – 2.0 | `8405d93` … `974e51b` | Still broken; no PR restored it |
| HEAD | `3bf22d3` | Still broken (boot-screen change only) |

Two PRs in the broken range are notable:

- **PR #33** (`e5314eb`, "Fix 4-gray rendering") fixed the *analogous* bug for GRAY4 by adding
  the two-plane `streamGray4Bytes` streamer — gated on GRAY4 only, leaving BWR/BWY behind.
- **PR #26** (`d7045d1`, "Implement streaming decompression") added the START size check,
  changing compressed BWR/BWY from "decompress and silently truncate" to "NACK at START".

### Why release 2.0 can *appear* to work

On common BWR panels (e.g. 296×128, width divisible by 8) the B/W plane lands perfectly
aligned, so uploads look correct — black and white render fine; only red/yellow pixels are
missing or stale. A test image with little or no red passes a casual check.

## Additional findings beyond CR-5

- **A1 — plane sizing must be row-padded.** py-opendisplay and opendisplay-js both pack each
  plane as `ceil(w/8) * h` (row-aligned, matching panel RAM). The firmware's bitplane sizing
  `(pixels + 7) / 8` only agrees when `width % 8 == 0`. The fix must use the row-padded formula
  — the same one GRAY4 already uses (`display_service.cpp:1485`).
- **A2 — website encoder disagrees with the other senders** (`ble-common.js:3461-3499`): packs
  bits continuously across row boundaries (not row-padded) and sets the B/W-plane bit for red
  pixels (py/TS leave it 0). Harmless when `width % 8 == 0`, wrong otherwise. Needs a separate
  fix in the website repo.

## Recommended fix (firmware)

BWR/BWY is structurally identical to GRAY4: two concatenated row-padded 1bpp planes streamed to
PLANE_0 then PLANE_1. In `src/display_service.cpp`:

1. Generalize `streamGray4Bytes` (`:1401-1419`) into a two-plane streamer shared by GRAY4 and
   BWR/BWY; route both the uncompressed (`:1645`) and compressed (`:1928`) sinks through it via
   a `directWriteIsTwoPlane()` predicate.
2. Size bitplane uploads as `2 * ((w + 7) / 8) * h` (`:1474`, merge with the GRAY4 branch at
   `:1485`). This also makes the compressed START size check accept the clients' payloads.
3. Extend the END underflow guard (`:1701-1712`) from GRAY4 to all two-plane uploads so a short
   upload NACKs instead of refreshing stale colour RAM.
4. Delete dead `directWritePlane2` (`main.h:165`, `display_service.cpp:52`, `:1427`, `:1469`).

**Verification:** build `nrf52840custom` + one ESP32 env; on a BWR panel upload a
black/white/red test image via py-opendisplay, compressed (START must ACK, no fallback warning)
and uncompressed; regression-check a MONO and a GRAY4 panel.
