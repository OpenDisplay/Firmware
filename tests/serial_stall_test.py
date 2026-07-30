#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.9"
# dependencies = ["pyserial"]
# ///
"""
Hardware-in-the-loop test for the non-blocking logger (src/od_log.cpp).

Reproduces the field failure: a USB CDC host that keeps the port open -- DTR
asserted -- but stops draining the IN endpoint. Adafruit_USBD_CDC::write() spins
on that condition with no timeout and no iteration cap, so a log line written
during the stall blocks loop() forever. On nRF there is no watchdog, so the tag
never comes back: epdSessionTick() stops, the keep-alive never expires, and the
device goes silent mid-session. Two field captures end exactly that way.

Closing the port does NOT reproduce it. That drops DTR, tud_cdc_n_connected()
goes false, and write() exits cleanly -- the passing case. The failure needs the
port held open and unread, which is what this script does.

    Phase 1  warmup   read normally, prove the device is talking
    Phase 2  stall    stop reading, port still open, DTR still high
    Phase 3  recover  read again and see whether the device survived

PASS means output resumed after the stall. FAIL means silence: the device is
hung, which is the bug.

Run it against nRF. On ESP32 the equivalent block lives in HWCDC::write(), and
a locally patched Arduino core may already bound it -- see the note in
tests/README.md.

Examples:
  ./tests/serial_stall_test.py --port /dev/ttyACM0
  ./tests/serial_stall_test.py --port /dev/ttyACM0 --stall 45 --expect-drop-notice
"""

from __future__ import annotations

import argparse
import sys
import time

import serial

# Spliced in by od_emit() after the level on the first record that gets through
# following a drop, e.g. "[0416.213|C0] I: [DROP: 214] DW complete: ...".
DROP_NOTICE = "[DROP:"


def drain(port: serial.Serial, seconds: float, echo: bool) -> bytes:
    """Read for `seconds`, returning everything seen. Never blocks past the deadline."""
    deadline = time.monotonic() + seconds
    chunks = []
    while time.monotonic() < deadline:
        waiting = port.in_waiting
        data = port.read(waiting) if waiting else port.read(1)  # read(1) honours timeout
        if data:
            chunks.append(data)
            if echo:
                sys.stdout.write(data.decode("utf-8", "replace"))
                sys.stdout.flush()
    return b"".join(chunks)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="serial device, e.g. /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--warmup", type=float, default=5.0,
                    help="seconds of normal reading before the stall (default: 5)")
    ap.add_argument("--stall", type=float, default=30.0,
                    help="seconds to hold the port open without reading (default: 30)")
    ap.add_argument("--recover", type=float, default=20.0,
                    help="seconds to read after the stall before calling it hung (default: 20)")
    ap.add_argument("--expect-drop-notice", action="store_true",
                    help="also require the '[DROP: n]' report. Off by "
                         "default because it only appears if the device actually tried "
                         "to log during the stall")
    ap.add_argument("--quiet", action="store_true", help="do not echo device output")
    args = ap.parse_args()

    echo = not args.quiet

    # dsrdtr=False keeps pyserial from using DTR/DSR as flow control; we drive DTR
    # ourselves below because it is the whole point of the test.
    with serial.Serial(args.port, args.baud, timeout=0.1, dsrdtr=False) as port:
        # DTR high is the entire premise: tud_cdc_n_connected() is literally
        # "DTR is asserted", and it is the only thing keeping write() in its
        # spin. Ports that cannot do the ioctl (a pty, some virtual ports) can
        # still exercise this script's own logic, but they cannot reproduce the
        # firmware failure -- so say so loudly rather than reporting a pass that
        # means nothing.
        try:
            port.dtr = True
        except OSError as exc:
            print(f"[test] WARNING: could not assert DTR on {args.port} ({exc}).", file=sys.stderr)
            print("[test] WARNING: this port cannot reproduce the CDC stall; any PASS "
                  "below is meaningless.", file=sys.stderr)

        print(f"[test] phase 1: reading for {args.warmup:.0f}s to confirm the device is alive")
        warm = drain(port, args.warmup, echo)
        if not warm:
            print("[test] INCONCLUSIVE: no output during warmup. Is the device running a "
                  "-debug build, and is anything driving it?", file=sys.stderr)
            return 2

        print(f"\n[test] phase 2: STALLING for {args.stall:.0f}s -- port open, DTR high, not reading.")
        print("[test]          >>> trigger an image push NOW <<<")
        print("[test]          The kernel tty buffer has to fill before the host stops")
        print("[test]          issuing IN transfers, so the device needs a real burst of")
        print("[test]          output; idle MSD lines every 40s will not do it.")
        # Deliberately no read() of any kind in here. in_waiting is not polled
        # either -- on some platforms that can nudge the driver into servicing the
        # endpoint, which would defeat the test.
        time.sleep(args.stall)

        print(f"[test] phase 3: resuming reads for up to {args.recover:.0f}s")
        recovered = drain(port, args.recover, echo)

    text = recovered.decode("utf-8", "replace")

    print("\n[test] " + "-" * 60)
    if not recovered:
        print("[test] FAIL: no output after the stall -- the device is hung.")
        print("[test] This is the bug: a log write blocked inside the USB CDC")
        print("[test] driver and took loop() with it. On nRF nothing recovers it.")
        return 1

    if args.expect_drop_notice and DROP_NOTICE not in text:
        print(f"[test] FAIL: device survived, but no {DROP_NOTICE!r} tag was seen.")
        print("[test] Either nothing was logged during the stall (no push triggered,")
        print("[test] or too short), or the drop accounting is not reporting.")
        return 1

    print(f"[test] PASS: device survived the stall ({len(recovered)} bytes after resume).")
    if DROP_NOTICE in text:
        hits = [ln.strip() for ln in text.splitlines() if DROP_NOTICE in ln]
        for line in hits[:3]:
            print(f"[test]   {line}")
        if len(hits) > 3:
            print(f"[test]   ... and {len(hits) - 3} more")
    else:
        print("[test] note: no drop report seen. Expected if the device had nothing to")
        print("[test]       log while stalled -- pass --expect-drop-notice to require it.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
