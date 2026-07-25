#!/usr/bin/env python3
"""
Factory firmware provisioning for OpenDisplay test3 firmware.

Config is compiled into the firmware image (nRF and ESP32). First boot copies
it to /config.bin when none is stored.

Examples:
  ./tools/provision_firmware.py build -e nrf52840custom --config-hex "BD 00 01 ..."
  ./tools/provision_firmware.py build --env nrf52840custom --version 1.4.0
  ./tools/provision_firmware.py build --env nrf52840custom --version 1.4.0 --upload
  ./tools/provision_firmware.py clear --env nrf52840custom --upload
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

TOOL_DIR = Path(__file__).resolve().parent
FW_DIR = TOOL_DIR.parent
sys.path.insert(0, str(TOOL_DIR))

from config_packet import read_hex_arg  # noqa: E402


def git_sha() -> str:
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=FW_DIR,
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode == 0:
            return result.stdout.strip()
    except OSError:
        pass
    return ""


def append_build_flags(extra: dict[str, str], version: str | None, sha: str | None) -> None:
    parts: list[str] = []
    if version:
        parts.append(f'-D BUILD_VERSION=\\"{version}\\"')
    if sha is not None and sha != "":
        parts.append(f"-D SHA={sha}")
    if not parts:
        return
    existing = extra.get("PLATFORMIO_BUILD_FLAGS", os.environ.get("PLATFORMIO_BUILD_FLAGS", "")).strip()
    merged = " ".join(p for p in [existing, *parts] if p)
    extra["PLATFORMIO_BUILD_FLAGS"] = merged


def find_pio() -> str:
    override = os.environ.get("PLATFORMIO_CORE_DIR", "").strip()
    if override:
        candidate = Path(override) / "penv" / "bin" / "pio"
        if candidate.is_file():
            return str(candidate)
    candidate = Path.home() / ".platformio" / "penv" / "bin" / "pio"
    if candidate.is_file():
        return str(candidate)
    return "pio"


def firmware_artifacts(env: str) -> list[Path]:
    build_dir = FW_DIR / ".pio" / "build" / env
    names = ("firmware.uf2", "firmware.hex", "firmware.bin", "firmware.zip")
    return [build_dir / name for name in names if (build_dir / name).is_file()]


def run_pio(env: str, extra_env: dict[str, str], upload: bool) -> int:
    pio = find_pio()
    cmd = [pio, "run", "-e", env]
    if upload:
        cmd.extend(["-t", "upload"])
    env_vars = os.environ.copy()
    env_vars.update(extra_env)
    print(" ".join(cmd), file=sys.stderr)
    if extra_env.get("PLATFORMIO_BUILD_FLAGS"):
        print(f"PLATFORMIO_BUILD_FLAGS={extra_env['PLATFORMIO_BUILD_FLAGS']}", file=sys.stderr)
    rc = subprocess.call(cmd, cwd=FW_DIR, env=env_vars)
    if rc == 0:
        for path in firmware_artifacts(env):
            print(path, file=sys.stderr)
    return rc


def add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("-e", "--env", required=True, help="PlatformIO environment name")
    parser.add_argument(
        "--upload",
        action="store_true",
        help="Flash after build via PlatformIO (default: compile only)",
    )
    parser.add_argument(
        "--version",
        metavar="VER",
        help='Firmware BUILD_VERSION string (major.minor.patch, e.g. "1.4.0"; two-part "1.4" implies patch 0)',
    )
    parser.add_argument(
        "--sha",
        metavar="HEX",
        help="Git SHA for BUILD stamp (default: current git HEAD when --version is set)",
    )
    parser.add_argument(
        "--no-sha",
        action="store_true",
        help="Do not pass SHA even when --version is set",
    )


def build_extra_env(args: argparse.Namespace) -> dict[str, str]:
    extra: dict[str, str] = {}
    sha: str | None = None
    if args.version and not args.no_sha:
        sha = args.sha if args.sha is not None else git_sha()
    elif args.sha is not None:
        sha = args.sha
    append_build_flags(extra, args.version, sha)
    return extra


def cmd_build(args: argparse.Namespace) -> int:
    extra = build_extra_env(args)
    if args.config_hex or args.config_file:
        packet = read_hex_arg(args.config_hex, str(args.config_file) if args.config_file else None)
        extra["OPENDISPLAY_FACTORY_CONFIG_HEX"] = " ".join(f"{b:02X}" for b in packet)
    return run_pio(args.env, extra, args.upload)


def cmd_clear(args: argparse.Namespace) -> int:
    extra = build_extra_env(args)
    extra["OPENDISPLAY_FACTORY_CLEAR_CONFIG"] = "1"
    return run_pio(args.env, extra, args.upload)


def main() -> int:
    parser = argparse.ArgumentParser(description="OpenDisplay factory firmware provisioning")
    sub = parser.add_subparsers(dest="command", required=True)

    p_build = sub.add_parser("build", help="Compile firmware with optional embedded config")
    add_common_args(p_build)
    p_build.add_argument("--config-hex", help="Toolbox outer packet as hex bytes")
    p_build.add_argument("--config-file", type=Path, help="Read packet bytes from a file")
    p_build.set_defaults(func=cmd_build)

    p_clear = sub.add_parser(
        "clear",
        help="Build firmware that deletes /config.bin once on boot (skips loading)",
    )
    add_common_args(p_clear)
    p_clear.set_defaults(func=cmd_clear)

    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
