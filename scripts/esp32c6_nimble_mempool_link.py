"""Force-link libble_app os_mempool for ESP32-C6 + NimBLE-Arduino.

C6 sdkconfig sets CONFIG_BT_LE_CONTROLLER_NPL_OS_PORTING_SUPPORT, so
NimBLE-Arduino skips compiling os_mempool and calls r_os_mem* instead.
Those live in libble_app.a(os_mempool.c.o).

On Ubuntu (GitHub Actions), gcc uses --as-needed: an early -lble_app with
no pending undefs is dropped, and a later single-pass never pulls
os_mempool.c.o. Fix:
  1) Prepend -u so the undefs exist before any archive scan
  2) Append libble_app.a by absolute path after all objects/libs
"""

from pathlib import Path

Import("env")  # noqa: F821

env.Prepend(
    LINKFLAGS=[
        "-Wl,-u,r_os_mempool_init",
        "-Wl,-u,r_os_memblock_get",
        "-Wl,-u,r_os_memblock_put",
    ]
)

libs_dir = env.PioPlatform().get_package_dir("framework-arduinoespressif32-libs")
ble_app = Path(libs_dir) / "esp32c6" / "lib" / "libble_app.a"
if ble_app.is_file():
    env.Append(LINKFLAGS=[str(ble_app)])
else:
    env.Append(LIBS=["ble_app"])
