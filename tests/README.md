# Tests

Hardware-in-the-loop tests. These need a real tag attached; there is no unit
test suite in this repo. Unit-style harnesses that run on the host live in
`tools/` (e.g. `test_zlib_stream.c`).

- `serial_stall_test.py` — reproduces the USB CDC log-write hang.

## serial_stall_test.py

Holds the serial port open with DTR asserted and stops reading it, which is the
one host condition that traps `Adafruit_USBD_CDC::write()`:

```c
while (remain && tud_cdc_n_connected(_instance)) {   // no timeout, no cap
  ...
  if (remain) { yield(); }
}
```

`tud_cdc_n_connected()` is just "DTR is high", so a host that keeps the port
open but stops draining the IN endpoint spins that loop forever. The log write
blocks, `loop()` blocks with it, and on nRF — which has no watchdog — the tag is
gone until it is power-cycled. `epdSessionTick()` stops running, so the giveaway
in a capture is a `[EPD session] release: panel warm-idle, off in 30000 ms` with
no `keep-alive expired` line 30 s later.

Closing the port, `Ctrl+C`, or unplugging all drop DTR and let `write()` return.
Those are the passing case and will not reproduce anything.

The fix does not guard that loop — it leaves it unreachable. `od_log` writes
through `tud_cdc_write()` on nRF, a bare `tu_fifo_write_n` that takes what fits
and returns the count, so a stalled host is structurally incapable of blocking
`loop()` rather than merely prevented from doing so by a correct check. You can
confirm the wrapper is out of the path without hardware:

```bash
arm-none-eabi-nm -C .pio/build/nrf52840custom/src/od_log.cpp.o | grep -i cdc
# expect only tud_cdc_n_write / _available / _flush -- no Adafruit_USBD_CDC::write
```

```bash
./tests/serial_stall_test.py --port /dev/ttyACM0
./tests/serial_stall_test.py --port /dev/ttyACM0 --stall 45 --expect-drop-notice
```

Trigger an image push during the stall window — the script prints a prompt when
it starts. The Linux `cdc-acm` driver keeps issuing IN transfers until its tty
buffer fills and it throttles, so the device needs a genuine burst of output
before its own 256-byte FIFO backs up. The ~300 progress lines of one push do it
immediately; idle MSD lines every 40 s do not.

Exit codes: `0` pass, `1` fail (device hung, or the drop report was required and
missing), `2` inconclusive (no output during warmup).

### Which build to test

Use a `-debug` env. `OD_LOG_LEVEL` defaults to `INFO`, and the per-frame `DEBUG`
lines are what produce enough output to fill the FIFO:

```bash
pio run -e nrf52840custom-debug -t upload
```

### Expected results

| Firmware | Result |
|---|---|
| Before `fix(log): make od_log non-blocking` | **FAIL** — silence after the stall. This is the bug. |
| After | **PASS** — the push completes during the stall, and on resume the first record carries `[DROP: n]` after its level, reporting what was lost. |

### Run it on nRF

On ESP32 the same shape of block lives in `HWCDC::write()`, but a locally
patched `~/.platformio/packages/framework-arduinoespressif32` may already bound
it with a `max_consec_timeouts` cap that is **not** reproduced by anything in
this repo — so a local ESP32 build can pass while a CI build of the same commit
would not. Until that patch is captured as a build step, ESP32 results from this
test say more about the machine than about the firmware. nRF has no such
divergence.
