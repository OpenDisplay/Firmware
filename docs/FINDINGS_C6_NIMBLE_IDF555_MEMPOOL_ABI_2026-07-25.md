# Do not move past IDF 5.5.4 until NimBLE-Arduino catches up (ESP32-C6)

**Date:** 2026-07-25
**Applies to:** `esp32-c6-N4` (and any future C6/H2/C2 target)
**Short version:** pin pioarduino **55.03.39** (Arduino 3.3.9 / IDF 5.5.4) or older.
**55.03.311** (Arduino 3.3.11 / IDF 5.5.5) does not link on C6 with
NimBLE-Arduino 2.5.0. Do not bump the pin until NimBLE-Arduino ships a release
built against IDF 5.5.5.

## The break

IDF 5.5.5 renamed the NimBLE OS-porting mempool exports for the C6 BLE
controller. They **lost their `r_` prefix and moved archives**:

| Symbol NimBLE-Arduino 2.5.0 calls | IDF 5.5.4 (`55.03.39`) | IDF 5.5.5 (`55.03.311`) |
|---|---|---|
| `r_os_mempool_init` | `libble_app.a(os_mempool.c.o)` | **not defined anywhere** |
| `r_os_memblock_get`  | `libble_app.a(os_mempool.c.o)` | **not defined anywhere** |
| `r_os_memblock_put`  | `libble_app.a(os_mempool.c.o)` | **not defined anywhere** |

Under 5.5.5 the same functions exist as `os_mempool_init` / `os_memblock_get` /
`os_memblock_put` in `libbt.a`. Neighbouring families (`r_os_mbuf_*`,
`r_os_cputime_*`) kept their prefix, so this is a targeted rename, not a
blanket one.

NimBLE-Arduino 2.5.0 (2026-04-02, the latest release as of this writing) still
emits calls to the `r_`-prefixed names, so the C6 link fails with a wall of:

```
ble_att_svr.c:3287: undefined reference to `r_os_mempool_init'
ble_gap.c:9045:     undefined reference to `r_os_mempool_init'
ble_hs.c:784:       undefined reference to `r_os_mempool_init'
... (ble_gatts, ble_gattc, ble_hs_conn, ble_l2cap, ble_l2cap_sig, ble_sm)
collect2: error: ld returned 1 exit status
```

There is no library-side fix available: 2.5.0 *is* current. This has to be
resolved upstream in NimBLE-Arduino.

## Why only C6

C6 is the only target in this repo whose BLE controller ships as a
**precompiled blob** (`libble_app.a`) carrying its own copy of the NimBLE
OS-porting layer. Its sdkconfig therefore sets
`CONFIG_BT_LE_CONTROLLER_NPL_OS_PORTING_SUPPORT=y`, which tells the NimBLE host
*not* to compile mempool/mbuf/cputime itself and to call the blob's exports
instead. Measured across `framework-arduinoespressif32-libs`:

| Chip | `..._NPL_OS_PORTING_SUPPORT` | `libble_app.a` |
|---|---|---|
| esp32 | 0 | absent |
| esp32s3 | 0 | absent |
| esp32c3 | 0 | absent |
| **esp32c6** | **1** | **present** |

ESP32/S3/C3 compile the porting layer from source, reference plain
`os_mempool_init`, and are unaffected by the rename. All ten non-C6
environments build clean on `55.03.311`.

## What was verified (2026-07-25)

All eleven environments, built locally:

| Platform | C6 without the script | C6 with the script | Other 10 envs |
|---|---|---|---|
| 55.03.32 (Arduino 3.3.2 / IDF 5.5.1) | links | links (no-op, +28 B) | OK |
| **55.03.39 (3.3.9 / IDF 5.5.4)** — current pin | links | links (no-op, +148 B) | OK |
| 55.03.311 (3.3.11 / IDF 5.5.5) | **link fails** | **link fails** | OK |

A linker-alias workaround does link:

```ini
-Wl,--defsym,r_os_mempool_init=os_mempool_init
-Wl,--defsym,r_os_memblock_get=os_memblock_get
-Wl,--defsym,r_os_memblock_put=os_memblock_put
```

It is **not applied**, and should not be without on-device validation: it
aliases BLE memory-pool allocation to symbols assumed ABI-identical from their
names alone. A signature mismatch corrupts the BLE heap at runtime rather than
failing at link time.

## `scripts/esp32c6_nimble_mempool_link.py`

The script extracts `os_mempool.c.o` from `libble_app.a` and links it as a
plain object, because CI's `--as-needed` plus single-pass archive scan was
reported to drop the member. On both `.32` and `.39` the ordinary link resolves
`r_os_mempool_init` unaided, so locally the script is belt-and-braces; it is
kept for CI, which is where the ordering problem was observed.

Its failure paths were changed on 2026-07-25 from `sys.exit(1)` to
**warn-and-skip**. Under `.311` the hard exit aborted with
`no entry os_mempool.c.o in archive`, which hid the actual cause (the rename)
behind a missing-member error. Skipping lets the link proceed and report the
genuine `undefined reference to r_os_mempool_init`, which names the real
problem.

## Before bumping the pin

1. Confirm NimBLE-Arduino has a release built against IDF 5.5.5 — i.e. one
   that calls the unprefixed `os_mempool_*` names on C6.
2. Build `esp32-c6-N4` both with and without the script; both should link.
3. Flash a real C6 and confirm BLE advertises, connects, and completes an
   image transfer. A mempool ABI problem will not show up at build time.

## Related

- `platformio.ini` — the pin and a condensed version of this warning.
- Upstream commit `0d95b37` — introduced the version pin (at `55.03.32`) and
  the current form of the C6 force-link script.
