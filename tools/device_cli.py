#!/usr/bin/env python3
"""
Read/edit/write an OpenDisplay device's config, live over BLE or offline as hex.

Decodes the binary config packet (see loadGlobalConfig(), src/config_parser.cpp) into
YAML with named fields for every block type in src/structs.h, and re-encodes edited YAML
back to the same wire format:
    [2B LE length] [1B version] { [1B separator] [1B tag] [NB struct] }* [2B LE CRC-16]

Commands: read, write, decode, encode, add-sensor, read-msd
Run `device_cli.py <command> --help` for command-specific examples.

Dependencies:
    - bleak         BLE communication (read/write/read-msd/add-sensor over --addr)
    - pyyaml        YAML config I/O (read/write/decode/encode)
    - cryptography  encrypted BLE sessions (--key)

Offline hex-only usage (decode/encode via --config-hex) needs none of the above.

Setup:
    python3 -m venv .venv
    source .venv/bin/activate
    pip install bleak pyyaml cryptography
"""

from __future__ import annotations

import argparse
import asyncio
import os
import sys
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any, AsyncIterator, Callable, NamedTuple

TOOL_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOL_DIR))

from ble_crypto import (  # noqa: E402
    BleSession,
    compute_challenge_response,
    compute_server_response,
    derive_session_id,
    derive_session_key,
)
from config_packet import outer_packet_crc, read_hex_arg, validate_packet  # noqa: E402

CHAR_UUID = "00002446-0000-1000-8000-00805f9b34fb"
WRITE_CHUNK_SIZE = 200


class Field(NamedTuple):
    name: str
    kind: str  # "u8" | "u16" | "u24" | "u32" | "hex" | "str"
    size: int  # exact byte width of this field
    needs_nul: bool = True  # "str" fields only: must leave room for a NUL terminator


class Block(NamedTuple):
    key: str
    singleton: bool
    fields: list[Field]
    size: int
    max_instances: int


# Field layouts mirror the packed structs in src/structs.h exactly (byte-for-byte,
# verified against sizeof() for each struct). "reserved*" fields are omitted from
# decoded YAML when all-zero, and default to all-zero when absent on encode.
BLOCKS: dict[int, Block] = {
    0x01: Block(
        "system_config",
        True,
        [
            Field("ic_type", "u16", 2),
            Field("communication_modes", "u8", 1),
            Field("device_flags", "u8", 1),
            Field("pwr_pin", "u8", 1),
            Field("reserved", "hex", 15),
            Field("pwr_pin_2", "u8", 1),
            Field("pwr_pin_3", "u8", 1),
        ],
        22,
        1,
    ),
    0x02: Block(
        "manufacturer_data",
        True,
        [
            Field("manufacturer_id", "u16", 2),
            Field("board_type", "u8", 1),
            Field("board_revision", "u8", 1),
            Field("reserved", "hex", 18),
        ],
        22,
        1,
    ),
    0x04: Block(
        "power_option",
        True,
        [
            Field("power_mode", "u8", 1),
            # assumed little-endian; firmware declares this as a raw uint8_t[3] and never
            # composes it as an integer, so this can't be fully verified against src/
            Field("battery_capacity_mah", "u24", 3),
            Field("sleep_timeout_ms", "u16", 2),
            Field("tx_power", "u8", 1),
            Field("sleep_flags", "u8", 1),
            Field("battery_sense_pin", "u8", 1),
            Field("battery_sense_enable_pin", "u8", 1),
            Field("battery_sense_flags", "u8", 1),
            Field("capacity_estimator", "u8", 1),
            Field("voltage_scaling_factor", "u16", 2),
            Field("deep_sleep_current_ua", "u32", 4),
            Field("deep_sleep_time_seconds", "u16", 2),
            Field("charge_enable_pin", "u8", 1),
            Field("charge_state_pin", "u8", 1),
            Field("charger_flags", "u8", 1),
            Field("min_wake_time_seconds", "u16", 2),
            Field("screen_timeout_seconds", "u8", 1),
            Field("reserved", "hex", 4),
        ],
        30,
        1,
    ),
    0x20: Block(
        "displays",
        False,
        [
            Field("instance_number", "u8", 1),
            Field("display_technology", "u8", 1),
            Field("panel_ic_type", "u16", 2),
            Field("pixel_width", "u16", 2),
            Field("pixel_height", "u16", 2),
            Field("active_width_mm", "u16", 2),
            Field("active_height_mm", "u16", 2),
            Field("tag_type", "u16", 2),
            Field("rotation", "u8", 1),
            Field("reset_pin", "u8", 1),
            Field("busy_pin", "u8", 1),
            Field("dc_pin", "u8", 1),
            Field("cs_pin", "u8", 1),
            Field("data_pin", "u8", 1),
            Field("partial_update_support", "u8", 1),
            Field("color_scheme", "u8", 1),
            Field("transmission_modes", "u8", 1),
            Field("clk_pin", "u8", 1),
            Field("reserved_pin_2", "u8", 1),
            Field("reserved_pin_3", "u8", 1),
            Field("reserved_pin_4", "u8", 1),
            Field("reserved_pin_5", "u8", 1),
            Field("reserved_pin_6", "u8", 1),
            Field("reserved_pin_7", "u8", 1),
            Field("reserved_pin_8", "u8", 1),
            Field("full_update_mC", "u16", 2),
            Field("reserved", "hex", 13),
        ],
        46,
        4,
    ),
    0x21: Block(
        "leds",
        False,
        [
            Field("instance_number", "u8", 1),
            Field("led_type", "u8", 1),
            Field("led_1_r", "u8", 1),
            Field("led_2_g", "u8", 1),
            Field("led_3_b", "u8", 1),
            Field("led_4", "u8", 1),
            Field("led_flags", "u8", 1),
            Field("reserved", "hex", 15),
        ],
        22,
        4,
    ),
    0x23: Block(
        "sensors",
        False,
        [
            Field("instance_number", "u8", 1),
            Field("sensor_type", "u16", 2),
            Field("bus_id", "u8", 1),
            Field("i2c_addr_7bit", "u8", 1),
            Field("msd_data_start_byte", "u8", 1),
            Field("reserved", "hex", 24),
        ],
        30,
        4,
    ),
    0x24: Block(
        "data_buses",
        False,
        [
            Field("instance_number", "u8", 1),
            Field("bus_type", "u8", 1),
            Field("pin_1", "u8", 1),
            Field("pin_2", "u8", 1),
            Field("pin_3", "u8", 1),
            Field("pin_4", "u8", 1),
            Field("pin_5", "u8", 1),
            Field("pin_6", "u8", 1),
            Field("pin_7", "u8", 1),
            Field("bus_speed_hz", "u32", 4),
            Field("bus_flags", "u8", 1),
            Field("pullups", "u8", 1),
            Field("pulldowns", "u8", 1),
            Field("reserved", "hex", 14),
        ],
        30,
        4,
    ),
    0x25: Block(
        "binary_inputs",
        False,
        [
            Field("instance_number", "u8", 1),
            Field("input_type", "u8", 1),
            Field("display_as", "u8", 1),
            Field("reserved_pin_1", "u8", 1),
            Field("reserved_pin_2", "u8", 1),
            Field("reserved_pin_3", "u8", 1),
            Field("reserved_pin_4", "u8", 1),
            Field("reserved_pin_5", "u8", 1),
            Field("reserved_pin_6", "u8", 1),
            Field("reserved_pin_7", "u8", 1),
            Field("reserved_pin_8", "u8", 1),
            Field("input_flags", "u8", 1),
            Field("invert", "u8", 1),
            Field("pullups", "u8", 1),
            Field("pulldowns", "u8", 1),
            Field("button_data_byte_index", "u8", 1),
            Field("power_off_flags", "u8", 1),
            Field("power_off_hold_sec", "u8", 1),
            Field("reserved", "hex", 12),
        ],
        30,
        4,
    ),
    0x26: Block(
        "wifi_config",
        True,
        [
            # firmware copies the full 32 bytes into a separate 33-byte buffer and
            # NUL-terminates independently (config_parser.cpp) - no terminator needed on the wire
            Field("ssid", "str", 32, needs_nul=False),
            Field("password", "str", 32, needs_nul=False),
            Field("encryption_type", "u8", 1),
            Field("reserved", "hex", 95),
        ],
        160,
        1,
    ),
    0x27: Block(
        "security_config",
        True,
        [
            Field("encryption_enabled", "u8", 1),
            Field("encryption_key", "hex", 16),
            Field("session_timeout_seconds", "u16", 2),
            Field("flags", "u8", 1),
            Field("reset_pin", "u8", 1),
            Field("reserved", "hex", 43),
        ],
        64,
        1,
    ),
    0x28: Block(
        "touch_controllers",
        False,
        [
            Field("instance_number", "u8", 1),
            Field("touch_ic_type", "u16", 2),
            Field("bus_id", "u8", 1),
            Field("i2c_addr_7bit", "u8", 1),
            Field("int_pin", "u8", 1),
            Field("rst_pin", "u8", 1),
            Field("display_instance", "u8", 1),
            Field("flags", "u8", 1),
            Field("poll_interval_ms", "u8", 1),
            Field("touch_data_start_byte", "u8", 1),
            Field("enable_pin", "u8", 1),
            Field("reserved", "hex", 20),
        ],
        32,
        4,
    ),
    0x29: Block(
        "passive_buzzers",
        False,
        [
            Field("instance_number", "u8", 1),
            Field("drive_pin", "u8", 1),
            Field("enable_pin", "u8", 1),
            Field("flags", "u8", 1),
            Field("duty_percent", "u8", 1),
            Field("reserved", "hex", 27),
        ],
        32,
        4,
    ),
    0x2B: Block(
        "flash_configs",
        False,
        [
            Field("instance_number", "u8", 1),
            Field("flash_ic_type", "u8", 1),
            Field("bus_instance", "u8", 1),
            Field("flags", "u8", 1),
            Field("mosi_pin", "u8", 1),
            Field("sck_pin", "u8", 1),
            Field("cs_pin", "u8", 1),
            Field("power_pin", "u8", 1),
            Field("power_active", "u8", 1),
            Field("power_on_delay_ms", "u8", 1),
            Field("power_off_delay_ms", "u8", 1),
            Field("mode", "u8", 1),
            Field("reserved", "hex", 20),
        ],
        32,
        2,
    ),
    0x2C: Block(
        "data_extended",
        True,
        [
            Field("manufacturer_name", "str", 32),
            Field("model_name", "str", 32),
            Field("serial_number", "str", 32),
            Field("friendly_name", "str", 32),
            Field("device_location", "str", 32),
            Field("device_id", "str", 32),
            Field("custom_string_1", "str", 32),
            Field("custom_string_2", "str", 32),
            Field("custom_string_3", "str", 32),
        ],
        288,
        1,
    ),
}

for _tag, _block in BLOCKS.items():
    _computed = sum(f.size for f in _block.fields)
    if _computed != _block.size:
        raise AssertionError(f"tag 0x{_tag:02X} ({_block.key}): fields sum to {_computed} bytes, expected {_block.size}")

# Known enum values, from the #define constants in src/structs.h. Values outside these
# sets only trigger a warning (not an error) on encode - the firmware may define values
# this tool doesn't know about yet.
VALID_ENUMS: dict[tuple[str, str], set[int]] = {
    ("sensors", "sensor_type"): {1, 2, 3, 4, 5},  # SENSOR_TYPE_TEMPERATURE..BQ27220
    ("displays", "color_scheme"): {0, 1, 2, 3, 4, 5, 6, 7},  # COLOR_SCHEME_MONO..GRAY8
    ("touch_controllers", "touch_ic_type"): {0, 1},  # TOUCH_IC_NONE, TOUCH_IC_GT911
}


def decode_fields(fields: list[Field], payload: bytes) -> dict[str, Any]:
    result: dict[str, Any] = {}
    offset = 0
    for f in fields:
        raw = payload[offset : offset + f.size]
        offset += f.size
        if f.kind == "hex":
            value: Any = raw.hex()
            nonzero = any(raw)
        elif f.kind == "str":
            value = raw.split(b"\x00", 1)[0].decode("utf-8", errors="replace")
            nonzero = bool(value)
        else:
            value = int.from_bytes(raw, "little")
            nonzero = value != 0
        if f.name.startswith("reserved") and not nonzero:
            continue
        result[f.name] = value
    return result


def encode_fields(block_key: str, fields: list[Field], data: dict[str, Any], expected_size: int) -> bytes:
    known_names = {f.name for f in fields}
    unknown = set(data) - known_names
    if unknown:
        raise ValueError(f"{block_key}: unknown field(s) {sorted(unknown)!r}")

    out = bytearray()
    for f in fields:
        value = data.get(f.name)
        if f.kind == "hex":
            raw = bytes.fromhex(value) if value is not None else bytes(f.size)
        elif f.kind == "str":
            encoded = str(value if value is not None else "").encode("utf-8")
            max_len = f.size - 1 if f.needs_nul else f.size
            if len(encoded) > max_len:
                raise ValueError(f"{block_key}.{f.name}: value too long for {f.size} bytes")
            raw = encoded + bytes(f.size - len(encoded))
        else:
            int_value = int(value if value is not None else 0)
            if int_value < 0:
                raise ValueError(f"{block_key}.{f.name}: value {int_value} is negative, field is unsigned")
            try:
                raw = int_value.to_bytes(f.size, "little")
            except OverflowError:
                max_value = (1 << (f.size * 8)) - 1
                raise ValueError(
                    f"{block_key}.{f.name}: value {int_value} does not fit in {f.size} byte(s) "
                    f"({f.kind}, max {max_value})"
                ) from None
            enum_values = VALID_ENUMS.get((block_key, f.name))
            if enum_values is not None and int_value not in enum_values:
                print(
                    f"WARNING: {block_key}.{f.name}: value {int_value} is not a known enum value "
                    f"({sorted(enum_values)})",
                    file=sys.stderr,
                )
        if len(raw) != f.size:
            raise ValueError(f"{block_key}.{f.name}: expects {f.size} bytes, got {len(raw)}")
        out += raw
    if len(out) != expected_size:
        raise AssertionError(f"encoded size {len(out)} != expected {expected_size}")
    return bytes(out)


def decode_packet(packet: bytes) -> dict[str, Any]:
    singles: dict[int, dict[str, Any]] = {}
    lists: dict[int, list[dict[str, Any]]] = {}
    offset = 3  # 2-byte length + 1 version byte
    end = len(packet) - 2  # trailing 2-byte CRC
    while offset < end:
        block_start = offset
        offset += 1  # separator
        if offset >= end:
            raise ValueError(f"truncated block header at offset {block_start}")
        tag = packet[offset]
        offset += 1
        block = BLOCKS.get(tag)
        if block is None:
            raise ValueError(f"unknown tag 0x{tag:02X} at offset {block_start}")
        if offset + block.size > end:
            raise ValueError(f"truncated payload for tag 0x{tag:02X} at offset {block_start}")
        payload = packet[offset : offset + block.size]
        offset += block.size
        fields = decode_fields(block.fields, payload)
        if block.singleton:
            if tag in singles:
                print(f"WARNING: duplicate singleton tag 0x{tag:02X} ({block.key}), using the last one", file=sys.stderr)
            singles[tag] = fields
        else:
            lists.setdefault(tag, []).append(fields)

    doc: dict[str, Any] = {"version": packet[2]}
    for tag in sorted(BLOCKS):
        block = BLOCKS[tag]
        if block.singleton:
            if tag in singles:
                doc[block.key] = singles[tag]
        elif tag in lists:
            doc[block.key] = lists[tag]
    return doc


def repack(packet: bytes) -> bytes:
    """Recompute the length prefix and CRC-16 trailer for a packet body."""
    body = bytearray(packet)
    total_len = len(body)
    body[0] = total_len & 0xFF
    body[1] = (total_len >> 8) & 0xFF
    crc = outer_packet_crc(bytes(body[:-2]))
    body[-2] = crc & 0xFF
    body[-1] = (crc >> 8) & 0xFF
    return bytes(body)


def encode_packet(doc: dict[str, Any]) -> bytes:
    known_keys = {"version"} | {block.key for block in BLOCKS.values()}
    unknown_keys = set(doc) - known_keys
    if unknown_keys:
        raise ValueError(f"unknown top-level key(s) {sorted(unknown_keys)!r}")

    body = bytearray([0x00, 0x00, int(doc.get("version", 1)) & 0xFF])
    for tag in sorted(BLOCKS):
        block = BLOCKS[tag]
        if block.singleton:
            value = doc.get(block.key)
            if value is not None:
                body += bytes([0x00, tag])
                body += encode_fields(block.key, block.fields, value, block.size)
        else:
            items = doc.get(block.key) or []
            if len(items) > block.max_instances:
                raise ValueError(f"{block.key}: {len(items)} instances exceeds max {block.max_instances}")
            for item in items:
                body += bytes([0x00, tag])
                body += encode_fields(block.key, block.fields, item, block.size)
    body += b"\x00\x00"
    if len(body) > 0xFFFF:
        raise ValueError(f"resulting packet too large ({len(body)} bytes)")
    packet = repack(bytes(body))
    validate_packet(packet)
    return packet


def decode_msd_payload(payload: bytes) -> dict[str, Any]:
    """Decode the 16-byte MSD buffer built by updatemsdata() (display_service.cpp)."""
    if len(payload) != 16:
        raise ValueError(f"MSD payload must be 16 bytes, got {len(payload)}")
    company_id = payload[0] | (payload[1] << 8)
    status = payload[15]
    battery_raw_10mv = ((status & 0x01) << 8) | payload[14]
    return {
        "company_id": company_id,
        "dynamic_data_hex": payload[2:13].hex(),
        "temperature_c": (payload[13] / 2.0) - 40.0,
        "battery_voltage_v": battery_raw_10mv / 100.0 if battery_raw_10mv > 0 else None,
        "battery_raw_10mv": battery_raw_10mv,
        "reboot_flag": bool((status >> 1) & 0x01),
        "connection_requested": bool((status >> 2) & 0x01),
        "loop_counter": (status >> 4) & 0x0F,
    }


def format_hex(packet: bytes) -> str:
    return " ".join(f"{b:02X}" for b in packet)


def auto_int(value: str) -> int:
    return int(value, 0)


def _require_yaml() -> Any:
    try:
        import yaml  # ty: ignore[unresolved-import]
    except ImportError:
        print("ERROR: this command requires PyYAML - run: pip install pyyaml", file=sys.stderr)
        sys.exit(1)
    return yaml


def dump_yaml(doc: dict[str, Any]) -> str:
    yaml = _require_yaml()

    class IndentedDumper(yaml.SafeDumper):
        def increase_indent(self, flow: bool = False, indentless: bool = False) -> None:
            return super().increase_indent(flow, False)

    return "---\n" + yaml.dump(doc, Dumper=IndentedDumper, sort_keys=False, default_flow_style=False)


def load_yaml_doc(text: str) -> dict[str, Any]:
    yaml = _require_yaml()
    data = yaml.safe_load(text)
    if not isinstance(data, dict):
        raise ValueError("YAML document must be a mapping at the top level")
    return data


def _require_bleak() -> Any:
    try:
        import bleak  # ty: ignore[unresolved-import]
    except ImportError:
        print("ERROR: this command requires bleak - run: pip install bleak", file=sys.stderr)
        sys.exit(1)
    return bleak


def _status(message: str) -> None:
    print(message, file=sys.stderr, flush=True)


class _BleCtx:
    """A connected BLE session: raw transport plus optional encryption state."""

    def __init__(self, client: Any, session: BleSession | None):
        self.client = client
        self.session = session
        self.notify_handler: Callable[[bytes], None] | None = None

    def _on_notify(self, _sender: object, payload: bytearray) -> None:
        data = bytes(payload)
        if self.session and self.session.needs_decryption(data):
            try:
                data = self.session.decrypt(data)
            except Exception:
                return
        if self.notify_handler:
            self.notify_handler(data)

    async def send_command(self, cmd_hi: int, cmd_lo: int, payload: bytes = b"") -> None:
        if self.session and self.session.authenticated:
            wire = self.session.encrypt(cmd_hi, cmd_lo, payload)
        else:
            wire = bytes([cmd_hi, cmd_lo]) + payload
        await self.client.write_gatt_char(CHAR_UUID, wire)


async def _ble_authenticate(ctx: _BleCtx, session: BleSession, timeout: float) -> None:
    response_box: dict[str, bytes] = {}
    event = asyncio.Event()
    ctx.notify_handler = lambda data: (response_box.__setitem__("data", data), event.set())

    async def exchange(payload: bytes) -> bytes:
        event.clear()
        await ctx.client.write_gatt_char(CHAR_UUID, payload)
        try:
            await asyncio.wait_for(event.wait(), timeout=timeout)
        except asyncio.TimeoutError:
            raise RuntimeError("timed out waiting for authentication response") from None
        return response_box["data"]

    try:
        resp1 = await exchange(bytes([0x00, 0x50, 0x00]))
        status1 = resp1[2] if len(resp1) > 2 else 0xFF
        if status1 == 0x03:
            raise RuntimeError("device does not have encryption enabled")
        if status1 == 0x04:
            raise RuntimeError("authentication rate-limited by device, try again later")
        if status1 != 0x00 or len(resp1) < 23:
            raise RuntimeError(f"authentication challenge failed (status 0x{status1:02X})")
        server_nonce = resp1[3:19]
        device_id = resp1[19:23]

        client_nonce = os.urandom(16)
        challenge_response = compute_challenge_response(session.master_key, server_nonce, client_nonce, device_id)
        resp2 = await exchange(bytes([0x00, 0x50]) + client_nonce + challenge_response)
        status2 = resp2[2] if len(resp2) > 2 else 0xFF
        if status2 == 0x01:
            raise RuntimeError("authentication failed (wrong key)")
        if status2 != 0x00 or len(resp2) < 19:
            raise RuntimeError(f"authentication response failed (status 0x{status2:02X})")
        server_response = resp2[3:19]

        session.session_key = derive_session_key(session.master_key, client_nonce, server_nonce, device_id)
        session.session_id = derive_session_id(session.session_key, client_nonce, server_nonce)
        expected = compute_server_response(session.session_key, server_nonce, client_nonce, device_id)
        if expected != server_response:
            raise RuntimeError("mutual authentication failed (unexpected server response)")
        session.authenticated = True
        session.counter = 0
        _status("Authenticated (encrypted session established).")
    finally:
        ctx.notify_handler = None


@asynccontextmanager
async def _ble_connection(addr: str, key: bytes | None = None, auth_timeout: float = 10.0) -> AsyncIterator[_BleCtx]:
    bleak = _require_bleak()
    session = BleSession(master_key=key) if key else None
    _status(f"Connecting to {addr}...")
    async with bleak.BleakClient(addr) as client:
        _status("Connected.")
        ctx = _BleCtx(client, session)
        await client.start_notify(CHAR_UUID, ctx._on_notify)
        try:
            if session:
                _status("Authenticating...")
                await _ble_authenticate(ctx, session, auth_timeout)
            yield ctx
        finally:
            await client.stop_notify(CHAR_UUID)


async def _do_read_config(ctx: _BleCtx, timeout: float) -> bytes:
    chunks: dict[int, bytes] = {}
    state: dict[str, int] = {}
    done = asyncio.Event()
    error = False

    def handle_notify(payload: bytes) -> None:
        nonlocal error
        if len(payload) < 1:
            return
        if payload[0] == 0xFF:
            error = True
            done.set()
            return
        if len(payload) < 4:
            return
        chunk_num = payload[2] | (payload[3] << 8)
        if chunk_num == 0:
            if len(payload) < 6:
                return
            state["total_len"] = payload[4] | (payload[5] << 8)
            chunks[0] = bytes(payload[6:])
        else:
            chunks[chunk_num] = bytes(payload[4:])
        if "total_len" in state and sum(len(b) for b in chunks.values()) >= state["total_len"]:
            done.set()

    ctx.notify_handler = handle_notify
    _status("Reading config...")
    try:
        await ctx.send_command(0x00, 0x40)
        try:
            await asyncio.wait_for(done.wait(), timeout=timeout)
        except asyncio.TimeoutError:
            raise RuntimeError("timed out waiting for config read response") from None
    finally:
        ctx.notify_handler = None

    if error:
        raise RuntimeError("device reported an error reading config (0x0040)")
    if "total_len" not in state:
        raise RuntimeError("no response from device (config read failed)")
    if sorted(chunks) != list(range(len(chunks))):
        raise RuntimeError(f"missing chunk(s) in config read response, got chunk numbers {sorted(chunks)}")
    ordered = bytearray()
    for i in sorted(chunks):
        ordered += chunks[i]
    packet = bytes(ordered[: state["total_len"]])
    validate_packet(packet)
    _status(f"Read {len(packet)} bytes ({len(chunks)} chunk(s)).")
    return packet


async def _do_write_config(ctx: _BleCtx, config: bytes, timeout: float) -> None:
    if len(config) > 4096:
        raise ValueError(f"config too large ({len(config)} bytes, max 4096)")
    ack = asyncio.Event()
    nack = False

    def handle_notify(payload: bytes) -> None:
        nonlocal nack
        if len(payload) < 1:
            return
        nack = payload[0] == 0xFF
        ack.set()

    ctx.notify_handler = handle_notify

    async def send_and_wait(cmd_byte: int, body: bytes) -> None:
        nonlocal nack
        ack.clear()
        nack = False
        await ctx.send_command(0x00, cmd_byte, body)
        try:
            await asyncio.wait_for(ack.wait(), timeout=timeout)
        except asyncio.TimeoutError:
            raise RuntimeError(f"timed out waiting for ack on command 0x{cmd_byte:02X}") from None
        if nack:
            raise RuntimeError(f"device rejected command 0x{cmd_byte:02X}")

    _status(f"Writing config ({len(config)} bytes)...")
    try:
        if len(config) <= WRITE_CHUNK_SIZE:
            await send_and_wait(0x41, config)
        else:
            total = len(config)
            num_chunks = 1 + -(-(total - WRITE_CHUNK_SIZE) // WRITE_CHUNK_SIZE)
            header = bytes([total & 0xFF, (total >> 8) & 0xFF]) + config[:WRITE_CHUNK_SIZE]
            await send_and_wait(0x41, header)
            _status(f"Sent chunk 1/{num_chunks}.")
            offset = WRITE_CHUNK_SIZE
            chunk_num = 1
            while offset < total:
                chunk = config[offset : offset + WRITE_CHUNK_SIZE]
                await send_and_wait(0x42, chunk)
                offset += len(chunk)
                chunk_num += 1
                _status(f"Sent chunk {chunk_num}/{num_chunks}.")
    finally:
        ctx.notify_handler = None
    _status("Write complete.")


async def _do_read_msd(ctx: _BleCtx, timeout: float) -> bytes:
    response_box: dict[str, bytes] = {}
    done = asyncio.Event()

    def handle_notify(payload: bytes) -> None:
        if len(payload) < 1:
            return
        response_box["data"] = payload
        done.set()

    ctx.notify_handler = handle_notify
    _status("Reading MSD data...")
    try:
        await ctx.send_command(0x00, 0x44)
        try:
            await asyncio.wait_for(done.wait(), timeout=timeout)
        except asyncio.TimeoutError:
            raise RuntimeError("timed out waiting for MSD read response") from None
    finally:
        ctx.notify_handler = None

    data = response_box.get("data")
    if data is None:
        raise RuntimeError("no response from device (MSD read failed)")
    # Two known error shapes: a bare 0xFF generic-error byte (per the 0x0040 convention),
    # or a 3-byte {cmd_hi, cmd_lo, status} NACK with status 0xFE/0xFF (e.g. not authenticated).
    if len(data) == 1 and data[0] == 0xFF:
        raise RuntimeError("device reported a generic error reading MSD data (0x0044)")
    if len(data) == 3 and data[0] == 0x00 and data[1] == 0x44 and data[2] in (0xFE, 0xFF):
        raise RuntimeError(f"device rejected MSD read (status 0x{data[2]:02X} - possibly not authenticated)")
    if len(data) < 2 + 16 or data[0] != 0x00 or data[1] != 0x44:
        raise RuntimeError(f"unexpected MSD response (expected 18 bytes starting with 00 44), got: {data.hex()}")
    return data[2:18]


async def ble_read_msd(addr: str, timeout: float = 10.0, key: bytes | None = None) -> bytes:
    async with _ble_connection(addr, key=key) as ctx:
        return await _do_read_msd(ctx, timeout)


async def ble_read_config(addr: str, timeout: float = 10.0, key: bytes | None = None) -> bytes:
    async with _ble_connection(addr, key=key) as ctx:
        return await _do_read_config(ctx, timeout)


async def ble_write_config(addr: str, config: bytes, timeout: float = 5.0, key: bytes | None = None) -> None:
    async with _ble_connection(addr, key=key) as ctx:
        await _do_write_config(ctx, config, timeout)


async def _ble_read_modify_write(
    addr: str, modify_fn: Callable[[dict[str, Any]], None], key: bytes | None = None, timeout: float = 10.0
) -> bytes:
    async with _ble_connection(addr, key=key) as ctx:
        packet = await _do_read_config(ctx, timeout)
        doc = decode_packet(packet)
        modify_fn(doc)
        new_packet = encode_packet(doc)
        await _do_write_config(ctx, new_packet, timeout)
        return new_packet


def _parse_key_arg(key_hex: str | None) -> bytes | None:
    if not key_hex:
        return None
    try:
        key = bytes.fromhex(key_hex)
    except ValueError:
        raise ValueError("--key must be a hex string") from None
    if len(key) != 16:
        raise ValueError(f"--key must be exactly 32 hex characters (16 bytes), got {len(key_hex)}")
    return key


def _write_output(text: str, output: Path | None) -> None:
    if output:
        output.write_text(text)
    else:
        print(text, end="" if text.endswith("\n") else "\n")


def cmd_read(args: argparse.Namespace) -> int:
    key = _parse_key_arg(args.key)
    packet = asyncio.run(ble_read_config(args.addr, key=key))
    doc = decode_packet(packet)
    _write_output(dump_yaml(doc), args.output)
    return 0


def cmd_write(args: argparse.Namespace) -> int:
    key = _parse_key_arg(args.key)
    doc = load_yaml_doc(args.input.read_text())
    packet = encode_packet(doc)
    asyncio.run(ble_write_config(args.addr, packet, key=key))
    print(f"Wrote config to {args.addr} ({len(packet)} bytes)")
    return 0


def cmd_decode(args: argparse.Namespace) -> int:
    packet = read_hex_arg(args.config_hex, str(args.config_file) if args.config_file else None)
    doc = decode_packet(packet)
    _write_output(dump_yaml(doc), args.output)
    return 0


def cmd_encode(args: argparse.Namespace) -> int:
    doc = load_yaml_doc(args.input.read_text())
    packet = encode_packet(doc)
    print(format_hex(packet))
    return 0


def cmd_read_msd(args: argparse.Namespace) -> int:
    key = _parse_key_arg(args.key)
    payload = asyncio.run(ble_read_msd(args.addr, key=key))
    if args.raw:
        print(payload.hex())
        return 0
    info = decode_msd_payload(payload)
    voltage = info["battery_voltage_v"]
    print(f"Battery voltage: {f'{voltage:.2f} V' if voltage is not None else 'unknown (raw=0 - unconfigured or not yet sampled)'}")
    print(f"Chip temperature: {info['temperature_c']:.1f} C")
    print(f"Reboot flag: {info['reboot_flag']}")
    print(f"Connection requested: {info['connection_requested']}")
    print(f"Loop counter: {info['loop_counter']}")
    print(f"Dynamic data (bytes 2-12): {info['dynamic_data_hex']}")
    return 0


def cmd_add_sensor(args: argparse.Namespace) -> int:
    if not args.addr and not (args.config_hex or args.config_file):
        raise ValueError("specify --addr or --config-hex/--config-file")
    key = _parse_key_arg(args.key)

    def add_sensor(doc: dict[str, Any]) -> None:
        sensors = doc.setdefault("sensors", [])
        if len(sensors) >= BLOCKS[0x23].max_instances:
            raise ValueError(f"packet already has {len(sensors)} sensors (max {BLOCKS[0x23].max_instances})")
        sensors.append(
            {
                "instance_number": args.instance,
                "sensor_type": args.sensor_type,
                "bus_id": args.bus_id,
                "i2c_addr_7bit": args.i2c_addr,
                "msd_data_start_byte": args.msd_start,
            }
        )

    if args.addr:
        new_packet = asyncio.run(_ble_read_modify_write(args.addr, add_sensor, key=key))
        doc = decode_packet(new_packet)
        print(f"Wrote updated config to {args.addr} ({len(new_packet)} bytes, {len(doc.get('sensors', []))} sensor(s))")
    else:
        packet = read_hex_arg(args.config_hex, str(args.config_file) if args.config_file else None)
        doc = decode_packet(packet)
        add_sensor(doc)
        new_packet = encode_packet(doc)
        print(format_hex(new_packet))
    return 0


def add_hex_input_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--config-hex", help="Existing config packet as hex bytes")
    parser.add_argument("--config-file", type=Path, help="Read the existing packet from a file")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    p_read = sub.add_parser(
        "read",
        help="Read a device's config over BLE and print it as YAML\n"
        "  e.g. device_cli.py read --addr AA:BB:CC:DD:EE:FF -o config.yaml\n",
        description="Read a device's config over BLE (command 0x0040) and decode it to YAML.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
Examples:
    device_cli.py read --addr AA:BB:CC:DD:EE:FF -o config.yaml
        Read the config and save it as YAML instead of printing to stdout.

    device_cli.py read --addr AA:BB:CC:DD:EE:FF --key 1e6d01ca00803339d31ee98ca052da71
        Same, for a device with BLE encryption enabled (security_config.encryption_enabled) -
        authenticates via the 0x0050 handshake first, then prints YAML to stdout.
""",
    )
    p_read.add_argument("--addr", required=True, help="BLE device address")
    p_read.add_argument("-o", "--output", type=Path, help="Write YAML to a file instead of stdout")
    p_read.add_argument("--key", metavar="HEX", help="16-byte master key (32 hex chars) for encrypted BLE")
    p_read.set_defaults(func=cmd_read)

    p_write = sub.add_parser(
        "write",
        help="Write a YAML config to a device over BLE\n"
        "  e.g. device_cli.py write --addr AA:BB:CC:DD:EE:FF --input config.yaml\n",
        description="Encode a YAML config and push it to a device over BLE (commands 0x0041/0x0042).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
Examples:
    device_cli.py write --addr AA:BB:CC:DD:EE:FF --input config.yaml
        Push config.yaml to the device, chunked automatically if it exceeds 200 bytes.

    device_cli.py write --addr AA:BB:CC:DD:EE:FF --input config.yaml --key 1e6d01ca00803339d31ee98ca052da71
        Same, authenticating first for a device with BLE encryption enabled.
""",
    )
    p_write.add_argument("--addr", required=True, help="BLE device address")
    p_write.add_argument("--input", required=True, type=Path, help="YAML config file")
    p_write.add_argument("--key", metavar="HEX", help="16-byte master key (32 hex chars) for encrypted BLE")
    p_write.set_defaults(func=cmd_write)

    p_decode = sub.add_parser(
        "decode",
        help="Decode a hex config packet to YAML (offline)\n"
        '  e.g. device_cli.py decode --config-hex "1D 00 01 ... EC 58"\n',
        description="Decode a config packet's hex bytes to YAML, offline - no device or BLE required.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
Examples:
    device_cli.py decode --config-hex "1D 00 01 ... EC 58"
        Decode a hex string pasted directly on the command line.

    device_cli.py decode --config-file dump.bin -o config.yaml
        Decode a binary config dump file and save the result as YAML.
""",
    )
    add_hex_input_args(p_decode)
    p_decode.add_argument("-o", "--output", type=Path, help="Write YAML to a file instead of stdout")
    p_decode.set_defaults(func=cmd_decode)

    p_encode = sub.add_parser(
        "encode",
        help="Encode a YAML config to a hex packet (offline)\n"
        "  e.g. device_cli.py encode --input config.yaml\n",
        description="Encode a YAML config to hex packet bytes, offline - no device or BLE required.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
Examples:
    device_cli.py encode --input config.yaml
        Print the encoded packet as hex bytes to stdout, e.g. for OPENDISPLAY_FACTORY_CONFIG_HEX.
""",
    )
    p_encode.add_argument("--input", required=True, type=Path, help="YAML config file")
    p_encode.set_defaults(func=cmd_encode)

    p_add = sub.add_parser(
        "add-sensor",
        help="Add a sensor_data (0x23) block, over BLE or offline\n"
        "  e.g. device_cli.py add-sensor --addr AA:BB:CC:DD:EE:FF --sensor-type 0x0003 --bus-id 0\n",
        description="Add a sensor_data (0x23) block to a config, over a live BLE device or offline hex.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
Examples:
    device_cli.py add-sensor --addr AA:BB:CC:DD:EE:FF --sensor-type 0x0003 --bus-id 0
        Read the device's config over BLE, add an AXP2101 sensor entry, and write it back
        in one step.

    device_cli.py add-sensor --config-hex "1D 00 01 ... EC 58" --sensor-type 0x0005 --bus-id 0
        Offline: add a sensor entry to a hex packet and print the updated hex, without
        touching a live device.
""",
    )
    p_add.add_argument("--addr", help="BLE device address (reads and writes back over BLE)")
    add_hex_input_args(p_add)
    p_add.add_argument("--instance", type=auto_int, default=0, help="instance_number (default: 0)")
    p_add.add_argument("--sensor-type", type=auto_int, required=True, help="sensor_type, e.g. 0x0003 for AXP2101")
    p_add.add_argument("--bus-id", type=auto_int, required=True, help="I2C bus instance id")
    p_add.add_argument("--i2c-addr", type=auto_int, default=0xFF, help="i2c_addr_7bit (default: 0xFF)")
    p_add.add_argument("--msd-start", type=auto_int, default=0xFF, help="msd_data_start_byte (default: 0xFF)")
    p_add.add_argument("--key", metavar="HEX", help="16-byte master key (32 hex chars) for encrypted BLE")
    p_add.set_defaults(func=cmd_add_sensor)

    p_msd = sub.add_parser(
        "read-msd",
        help="Read live MSD data (battery voltage, chip temperature) via command 0x0044\n"
        "  e.g. device_cli.py read-msd --addr AA:BB:CC:DD:EE:FF\n",
        description="Read the device's live 16-byte MSD buffer (command 0x0044) and decode battery "
        "voltage, chip temperature, and status bits.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
Examples:
    device_cli.py read-msd --addr AA:BB:CC:DD:EE:FF
        Print decoded battery voltage/temperature/status for an unencrypted device.

    device_cli.py read-msd --addr AA:BB:CC:DD:EE:FF --key 1e6d01ca00803339d31ee98ca052da71 --raw
        Same, for a device with BLE encryption enabled, printing the raw 16-byte payload
        as hex instead of decoding it.
""",
    )
    p_msd.add_argument("--addr", required=True, help="BLE device address")
    p_msd.add_argument("--key", metavar="HEX", help="16-byte master key (32 hex chars) for encrypted BLE")
    p_msd.add_argument("--raw", action="store_true", help="Print the raw 16-byte MSD payload as hex instead of decoding")
    p_msd.set_defaults(func=cmd_read_msd)

    args = parser.parse_args()
    try:
        return args.func(args)
    except (ValueError, RuntimeError, OSError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
