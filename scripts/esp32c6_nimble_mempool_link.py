"""Force-link libble_app os_mempool for ESP32-C6 + NimBLE-Arduino.

C6 sdkconfig sets CONFIG_BT_LE_CONTROLLER_NPL_OS_PORTING_SUPPORT, so
NimBLE-Arduino skips compiling os_mempool and calls r_os_mem* instead.
Those symbols live in libble_app.a(os_mempool.c.o). Archive scan order on
CI often never pulls that member; extract it and link as a plain object.

Must run as post: (after framework sets _LIBFLAGS), not pre:.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

Import("env")  # noqa: F821

MEMBER = "os_mempool.c.o"
NEEDED_SYM = "r_os_mempool_init"


def _die(msg: str) -> None:
    sys.stderr.write(f"esp32c6_nimble_mempool_link: {msg}\n")
    sys.exit(1)


def _defines_sym(nm_out: str, sym: str) -> bool:
    for line in nm_out.splitlines():
        parts = line.split()
        # e.g. "00000000 T r_os_mempool_init" or "T r_os_mempool_init"
        if len(parts) >= 2 and parts[-1] == sym and "T" in parts[:-1]:
            return True
    return False


libs_dir = env.PioPlatform().get_package_dir("framework-arduinoespressif32-libs")
if not libs_dir:
    _die("package framework-arduinoespressif32-libs not found")

archive = Path(libs_dir) / "esp32c6" / "lib" / "libble_app.a"
if not archive.is_file():
    _die(f"missing {archive}")

build_dir = Path(env.subst("$BUILD_DIR"))
build_dir.mkdir(parents=True, exist_ok=True)

ar = env.subst("$AR") or "ar"
subprocess.check_call([ar, "x", str(archive), MEMBER], cwd=build_dir)

extracted = build_dir / MEMBER
if not extracted.is_file():
    _die(f"ar x did not produce {extracted}")

forced = build_dir / "forced_r_os_mempool.c.o"
if forced.exists():
    forced.unlink()
extracted.replace(forced)

nm = env.subst("$NM") or "nm"
try:
    nm_out = subprocess.check_output([nm, str(forced)], text=True, stderr=subprocess.STDOUT)
except (subprocess.CalledProcessError, FileNotFoundError) as exc:
    _die(f"nm failed on {forced}: {exc}")

if not _defines_sym(nm_out, NEEDED_SYM):
    _die(f"{forced} does not define {NEEDED_SYM}")

# Inject into the lib-flags region (after objects), not early LINKFLAGS.
# Plain .o is always fully linked; follow with libble_app for r_ble_log_*/r_npl_funcs.
if "_LIBFLAGS" not in env:
    _die("_LIBFLAGS not set; use post: script after framework init")

prefix = f" {forced} -Wl,--start-group {archive} -Wl,--end-group "
env["_LIBFLAGS"] = prefix + str(env["_LIBFLAGS"])
print(f"esp32c6_nimble_mempool_link: force-linking {forced.name} from {archive.name}")
