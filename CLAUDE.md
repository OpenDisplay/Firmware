# CLAUDE.md — Firmware

Guidance for Claude Code when working in the **OpenDisplay `Firmware`** repo (nRF52840 + ESP32-S3/C6/C3 BLE e-paper tags). This is one repo inside the larger OpenDisplay multi-repo workspace; the workspace-level `../CLAUDE.md` covers cross-repo layout and the end-to-end pipeline. Prefer this file for anything Firmware-specific.

## Build & flash

PlatformIO (Arduino framework), using the `bb_epaper` display library. Environments are defined in `platformio.ini`.

```bash
pio run -e <env>                 # build one environment
pio run -e <env> -t upload       # build + flash
pio run                          # build every environment
```

Common envs: `nrf52840custom`, `esp32-s3-N16R8`, `esp32-s3-N8R8`, `esp32-c3-N16`, `esp32-c6-N4`. CI (`.github/workflows/main.yaml`) builds `nrf52840custom`, `esp32-s3-N16R8`, and `esp32-c3-N16` on every push — keep those three green.

Factory provisioning: `OPENDISPLAY_FACTORY_CONFIG_HEX="..." pio run -e <env>` (or `tools/provision_firmware.py`). `scripts/factory_config_gen.py` runs as a pre-build step.

## The vendored protocol header (critical invariant)

`include/opendisplay_protocol.h` is a **byte-for-byte vendored copy** of the canonical wire-protocol header. The canonical source lives in a sibling repo:

```
../opendisplay-protocol/src/opendisplay_protocol.h   ← single source of truth
include/opendisplay_protocol.h                        ← vendored copy (this repo)
```

**Never hand-edit `include/opendisplay_protocol.h`.** To change the protocol, edit the canonical file, then propagate and verify from the protocol repo:

```bash
cd ../opendisplay-protocol
tools/sync_protocol_header.py --push  --only Firmware   # canonical → this repo
tools/sync_protocol_header.py --check --only Firmware   # fail if the copy drifted
```

What the header contains and what it does **not**:

- It is **pure `#define` constants only** — protocol version, command opcodes (`CMD_*`), response codes (`RESP_*`), auth/error/NFC/pipe/config-limit constants, encryption sizes.
- It deliberately contains **no structs, enums, typedefs, or functions**. Those are per-repo and live in `src/structs.h` (e.g. `SystemConfig`, `PowerOption`, `PipeWriteState`). Do not move struct definitions into the header.

### Rules when touching protocol code

- Include the constants via `src/structs.h`, which is the common include hub (`#include "opendisplay_protocol.h"`). Most translation units get them transitively; add a direct include only where `structs.h` isn't already pulled in.
- **Do not redefine** any macro the canonical header provides. Firmware previously hand-defined some of these (`RESP_*`, `CONFIG_CHUNK_SIZE*`, `MAX_CONFIG_CHUNKS`, `MAX_RESPONSE_DATA_SIZE`, `PIPE_*`); they now come from the header only. A local `#define` of the same name is a redefinition error waiting to happen.
- Prefer named constants over magic numbers: use `CMD_PIPE_WRITE_START`, `CMD_CONFIG_WRITE`, etc. instead of raw `0x0080` / `0x0041` in switch/case dispatch (see `src/communication.cpp`).
- Opcode/response values must always match the canonical spec — if a value looks wrong, fix it in `../opendisplay-protocol` and re-`--push`, never locally.

## Source layout (`src/`)

- `main.cpp` / `main.h` — entry point, boot, main loop.
- `communication.cpp` — BLE command dispatch (opcode switch on `CMD_*`).
- `structs.h` — config-packet structs + Firmware-local constants; includes the vendored protocol header.
- `config_parser.*` — parse the factory/BLE config blob into the structs.
- `display_service.*`, `display_seeed_gfx.*` — rendering + panel drive (PIPE/DIRECT/PARTIAL write paths).
- `encryption.*`, `encryption_state.h` — BLE session auth + CCM.
- `ble_init.*`, `esp32_ble_callbacks.h` — NimBLE-Arduino setup and callbacks (ESP32 migrated Bluedroid → NimBLE).
- Peripherals: `buzzer_*`, `sensor_*`, `touch_input.*`, `wake_button.*`, `power_latch.*`, `wifi_service.*`.

## Notes

- `FINDINGS.md` and `docs/` hold working design/investigation notes (BLE throughput, pipe-write protocol, deep-sleep). Read the relevant one before changing that subsystem.
- Run git commands inside this repo, not the workspace root. Branch + PR per repo.
