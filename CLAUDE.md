# CLAUDE.md — Firmware

Guidance for Claude Code when working in the **OpenDisplay `Firmware`** repo
(nRF52840 + ESP32-S3/C6/C3 BLE e-paper tags). This is one repo inside the
larger OpenDisplay multi-repo workspace; the workspace-level `../CLAUDE.md`
covers cross-repo layout and the end-to-end pipeline. Prefer this file for
anything Firmware-specific.

## Build & flash

PlatformIO (Arduino framework), using the `bb_epaper` display library.
Environments are defined in `platformio.ini`.

```bash
pio run -e <env>                 # build one environment
pio run -e <env> -t upload       # build + flash
pio run                          # build every environment
```

Common envs: `nrf52840custom`, `esp32-s3-N16R8`, `esp32-s3-N8R8`,
`esp32-c3-N16`, `esp32-c6-N4`. CI (`.github/workflows/main.yaml`) builds every
environment in `.github/firmware-targets.json` on every push — **12** of them —
keep them all green. Note `platformio.ini`'s `default_envs` lists only 11:
`esp32-wrover-e-N4R8` ships but is NOT in it, so a bare `pio run` silently
skips the target most likely to catch a broken `#ifndef OPENDISPLAY_HAS_WIFI`
path. Build it explicitly (`pio run -e esp32-wrover-e-N4R8`) before claiming a
clean sweep.

Factory provisioning: `OPENDISPLAY_FACTORY_CONFIG_HEX="..." pio run -e <env>`
(or `tools/provision_firmware.py`). `scripts/factory_config_gen.py` runs as a
pre-build step.

### Do NOT bump the ESP platform past IDF 5.5.4 (C6 / NimBLE-Arduino)

Every ESP env pins pioarduino **`55.03.39`** (Arduino 3.3.9 / **IDF 5.5.4**) by
exact version, never the floating `stable` URL. **Do not move to `55.03.311`
(IDF 5.5.5) or later until NimBLE-Arduino ships a release built against it.**

**NimBLE-Arduino 2.5.0 depends on IDF ≤ 5.5.4.** IDF 5.5.5 dropped the `r_`
prefix on the C6 BLE controller's mempool exports and moved them from
`libble_app.a` to `libbt.a` (`r_os_mempool_init` → `os_mempool_init`, same for
`os_memblock_get`/`_put`). NimBLE-Arduino 2.5.0 — the latest release,
2026-04-02 — still calls the `r_` names, so `esp32-c6-N4` fails to link with a
wall of `undefined reference to r_os_mempool_init`. There is no library-side
fix; it has to land upstream in NimBLE-Arduino.

Only C6 is affected: it is the sole target whose BLE controller ships as a
precompiled blob carrying its own NimBLE porting layer
(`CONFIG_BT_LE_CONTROLLER_NPL_OS_PORTING_SUPPORT=y`). ESP32/S3/C3 compile that
layer from source and build fine on IDF 5.5.5.

`scripts/esp32c6_nimble_mempool_link.py` force-links the mempool object for C6
and **warns-and-skips on every failure path** — never abort the build from it,
or a future platform change hides the real linker error behind a missing-member
message. Full analysis, including a `--defsym` workaround that links but is
unvalidated on hardware:
`docs/FINDINGS_C6_NIMBLE_IDF555_MEMPOOL_ABI_2026-07-25.md`.

Note for whenever the bump does happen: `55.03.311` also requires PlatformIO
Core ≥ 6.1.19 (it silently uninstalls itself on older Core with an
`IncompatiblePlatform` error). The pinned `.39` has no such requirement.

## The vendored protocol header (critical invariant)

`include/opendisplay_protocol.h` is a **byte-for-byte vendored copy** of the
canonical wire-protocol header. The canonical source lives in a sibling repo:

```
../opendisplay-protocol/src/opendisplay_protocol.h   ← single source of truth
include/opendisplay_protocol.h                        ← vendored copy (this repo)
```

**Never hand-edit `include/opendisplay_protocol.h`.** To change the protocol,
edit the canonical file, then propagate and verify from the protocol repo:

```bash
cd ../opendisplay-protocol
tools/sync_protocol_header.py --push  --only Firmware   # canonical → this repo
tools/sync_protocol_header.py --check --only Firmware   # fail if the copy drifted
```

What the header contains and what it does **not**:

- It is **pure `#define` constants only** — protocol version, command opcodes
  (`CMD_*`), response codes (`RESP_*`), auth/error/NFC/pipe/config-limit
  constants, encryption sizes.
- It deliberately contains **no structs, enums, typedefs, or functions**. Those
  are per-repo and live in `src/structs.h` (e.g. `SystemConfig`, `PowerOption`,
  `PipeWriteState`). Do not move struct definitions into the header.

### Rules when touching protocol code

- Include the constants via `src/structs.h`, which is the common include hub
  (`#include "opendisplay_protocol.h"`). Most translation units get them
  transitively; add a direct include only where `structs.h` isn't already
  pulled in.
- **Do not redefine** any macro the canonical header provides. Firmware
  previously hand-defined some of these (`RESP_*`, `CONFIG_CHUNK_SIZE*`,
  `MAX_CONFIG_CHUNKS`, `MAX_RESPONSE_DATA_SIZE`, `PIPE_*`); they now come from
  the header only. A local `#define` of the same name is a redefinition error
  waiting to happen.
- Prefer named constants over magic numbers: use `CMD_PIPE_WRITE_START`,
  `CMD_CONFIG_WRITE`, etc. instead of raw `0x0080` / `0x0041` in switch/case
  dispatch (see `src/communication.cpp`).
- Opcode/response values must always match the canonical spec — if a value
  looks wrong, fix it in `../opendisplay-protocol` and re-`--push`, never
  locally.

### Keep `tools/od-device-cli.py` in sync

`tools/od-device-cli.py`'s `BLOCKS` dict duplicates the config-packet field
layout (names, sizes, enum values) from `include/opendisplay_structs.h` so it
can decode/encode config packets offline. It is **not** auto-generated.
Whenever a config struct's fields are renamed, resized, or added/removed in
`opendisplay_structs.h`, update `BLOCKS` (and `VALID_ENUMS` for new enum
values) in the same change. A stale field name that happens to start with
`reserved` silently vanishes from decoded YAML when its value is 0 — treat
drift here as a correctness bug, not a cosmetic one.

## Source layout (`src/`)

- `main.cpp` / `main.h` — entry point, boot, main loop.
- `communication.cpp` — BLE command dispatch (opcode switch on `CMD_*`).
- `structs.h` — config-packet structs + Firmware-local constants; includes the
  vendored protocol header.
- `config_parser.*` — parse the factory/BLE config blob into the structs.
- `display_service.*`, `display_fastepd.*` — rendering + panel drive
  (PIPE/DIRECT/PARTIAL write paths).
- `encryption.*`, `encryption_state.h` — BLE session auth + CCM.
- `ble_init.*`, `esp32_ble_callbacks.h` — NimBLE-Arduino setup and callbacks
  (ESP32 migrated Bluedroid → NimBLE).
- Peripherals: `buzzer_*`, `sensor_*`, `touch_input.*`, `wake_button.*`,
  `power_latch.*`, `wifi_service.*`.

## Coding rules

- **No `extern` for cross-file global access.** Don't reach into another
  file's global variable with `extern`. Expose a getter (and setter, if
  mutation is needed) instead. This keeps ownership of the variable in one
  translation unit and makes reads/writes greppable and interceptable.
- **Avoid heap allocation where possible.** These are memory-constrained
  MCUs (nRF52840 / ESP32-S3 / C6 / C3) — prefer stack allocation, static
  buffers, or fixed-size structs over `malloc`/`new`/dynamic containers.
  If a heap allocation is unavoidable, call it out explicitly and justify it.
- **Performance is a first-class constraint.** Before proposing or starting
  any implementation plan, explicitly consider its performance impact (CPU
  cycles, RAM/flash footprint, BLE/display timing) as part of the plan, not
  as an afterthought.

These rules apply to new and changed code. Don't mass-refactor pre-existing
`extern` globals or heap usage as a side effect of unrelated work — flag it
instead.

## Notes

- `FINDINGS.md` and `docs/` hold working design/investigation notes (BLE
  throughput, pipe-write protocol, deep-sleep). Read the relevant one before
  changing that subsystem.
- Run git commands inside this repo, not the workspace root. Branch + PR per
  repo.
