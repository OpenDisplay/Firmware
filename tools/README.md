# Tools

Helper scripts for building, provisioning, and talking to OpenDisplay devices.
These are advanced tools. They assume you know the wire protocol and the
firmware's config layout.

- `od-device-cli.py` — read, edit, and write a device's config over BLE.
- `provision_firmware.py` — build firmware with a config baked in, or flash a
  config-clearing image.
- `convert_logo.py` — convert `od_logo.svg` into a packed 1-bit C header for
  the boot screen.
- `uf2conv.py` / `uf2families.json` — convert `.bin`/`.hex` images to UF2 for
  boards that flash via USB mass storage.
- `test_zlib_stream.c` — standalone test harness for the firmware's streaming
  zlib/uzlib decoder (`lib/uzlib`). Not part of the PlatformIO build.
- `ble_crypto.py` / `config_packet.py` — shared helpers. `ble_crypto.py` is
  inlined into `od-device-cli.py` and kept here for reference. `config_packet.py`
  is imported by `provision_firmware.py`.

## od-device-cli.py

Reads, edits, and writes an OpenDisplay device's config packet.

The config is a small binary blob. It holds display pins, sensor wiring,
Wi-Fi credentials, and similar settings. This tool decodes it to YAML so you
can read and edit it by hand, then encodes it back to binary.

It's self-contained. Run it with `uv run`, or execute it directly if `uv` is
on your `PATH` — the shebang invokes `uv run --script` and installs
dependencies (`bleak`, `pyyaml`, `cryptography`) automatically.

```sh
uv run tools/od-device-cli.py --help
# or, if executable:
./tools/od-device-cli.py --help
```

Commands: `read-config`, `write-config`, `decode-config`, `encode-config`,
`add-sensor`, `read-msd`. Run any command with `--help` for its full options.

### Typical workflow: read, edit, write

Read the config from a device and save it as YAML:

```sh
./tools/od-device-cli.py read-config --addr AA:BB:CC:DD:EE:FF -o config.yaml
```

Edit `config.yaml` in any text editor — change a pin number, add a sensor
block, whatever you need.

Write the edited YAML back to the device:

```sh
./tools/od-device-cli.py write-config --addr AA:BB:CC:DD:EE:FF -i config.yaml
```

### Offline decode/encode

No device needed. Useful for inspecting a saved hex dump, or building a
config to embed at factory-flash time.

```sh
./tools/od-device-cli.py decode-config --config-hex "1D 00 01 ... EC 58"
./tools/od-device-cli.py encode-config --input config.yaml
```

### Other commands

Add a sensor block without hand-editing YAML:

```sh
./tools/od-device-cli.py add-sensor --addr AA:BB:CC:DD:EE:FF --sensor-type 0x0003 --bus-id 0
```

Read live battery voltage and chip temperature (no config change):

```sh
./tools/od-device-cli.py read-msd --addr AA:BB:CC:DD:EE:FF
```

### Encrypted devices

If `security_config.encryption_enabled` is set, pass `--key` with the
device's 16-byte master key (32 hex chars):

```sh
./tools/od-device-cli.py read-config --addr AA:BB:CC:DD:EE:FF --key 1e6d01ca00803339d31ee98ca052da71
```
