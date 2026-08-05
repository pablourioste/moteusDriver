#!/usr/bin/env python3
"""Capture a raw binary telemetry stream to disk.

Runs on WINDOWS today, because COM9 is a Windows device.  Invoke it from
the WSL terminal -- no separate PowerShell window needed:

    python.exe tools/telemetry/capture.py --port COM9 --out logs/t.bin -s 60

After the Xiao is in place the same script reads UDP instead, and then it
runs in WSL like everything else:

    ./moteus-venv/bin/python tools/telemetry/capture.py --udp 0.0.0.0:5005 \
        --out logs/t.bin

THIS SCRIPT IS DELIBERATELY DUMB.  It never parses a frame.  Its only job
is to get bytes onto disk intact, and every feature that could jeopardise
that is either absent or wrapped so it cannot take the capture down with
it.

Requires only pyserial (Windows: ``python.exe -m pip install pyserial``).
The decoding side needs pandas/pyarrow, but that runs in WSL -- keeping
this script dependency-light is what lets it run under a bare Windows
Python.
"""

import argparse
import signal
import socket
import sys
import time
from datetime import datetime
from pathlib import Path

FRAME_SIZE = 118

_stop = False


def _handle_signal(signum, frame):    # noqa: ARG001 -- signal API
    global _stop
    _stop = True


def default_output() -> Path:
    """logs/telemetry_YYYYMMDDTHHMMSS.bin

    Matches the timestamp convention CsvLogger uses for motor_test_*.csv
    (src/testing/CsvLogger.cpp), which in turn matches moteus_tool's
    calibration logs.  One convention across every artefact the project
    produces.
    """
    stamp = datetime.now().strftime("%Y%m%dT%H%M%S")
    return Path("logs") / f"telemetry_{stamp}.bin"


class LiveView:
    """Best-effort terminal display.

    Isolated behind this class so the capture loop can call it inside a
    try/except and keep writing bytes even if the display throws.  The log
    is the artefact that matters; the display is a convenience.
    """

    def __init__(self, interval: float = 0.1):
        self.interval = interval
        self.last_draw = 0.0
        self.decoder = None
        self.pending = b""
        self.frames = 0
        self.crc_errors = 0
        self.last_record = None
        self.first_seq = None
        self.last_seq = None
        self.lost = 0
        self.started = time.time()
        self.lines_drawn = 0

        # Import lazily: capture must work on a Windows Python that only
        # has pyserial, with no numpy or pandas.
        try:
            sys.path.insert(0, str(Path(__file__).parent))
            import decode
            self.decoder = decode
        except Exception as exc:                      # noqa: BLE001
            print(f"live view unavailable ({exc}); capturing anyway")

    def feed(self, chunk: bytes):
        if self.decoder is None:
            return
        self.pending += chunk
        # Bound the working buffer: a live view that silently grows to
        # gigabytes would eventually be the thing that kills the capture.
        if len(self.pending) > 64 * FRAME_SIZE:
            self.pending = self.pending[-64 * FRAME_SIZE:]

        consumed_to = 0
        for record in self.decoder.decode_bytes(self.pending):
            self.last_record = record
            self.frames += 1
            seq = record["seq"]
            if self.first_seq is None:
                self.first_seq = seq
            if self.last_seq is not None:
                gap = (seq - self.last_seq) & 0xFFFFFFFF
                if gap > 1:
                    self.lost += gap - 1
            self.last_seq = seq
            consumed_to += FRAME_SIZE
        self.pending = self.pending[consumed_to:] if consumed_to else self.pending

        now = time.time()
        if now - self.last_draw >= self.interval:
            self.last_draw = now
            self.draw()

    def draw(self):
        r = self.last_record
        if r is None:
            return
        elapsed = max(time.time() - self.started, 1e-6)
        rate = self.frames / elapsed
        expected = self.frames + self.lost
        loss = (100.0 * self.lost / expected) if expected else 0.0

        flags = r["flags"]
        overflow = "OVF" if flags & (1 << 6) else "   "
        safety = self.decoder.SAFETY_NAMES.get(r["safety"], "UNKNOWN")

        lines = [
            f"seq {r['seq']:<10d} loss {loss:5.2f}%  rate {rate:6.1f} Hz  {overflow}",
            f"accel  {r['accel_x']:+7.3f} {r['accel_y']:+7.3f} {r['accel_z']:+7.3f} m/s^2"
            f"   |a| {(r['accel_x']**2 + r['accel_y']**2 + r['accel_z']**2) ** 0.5:6.3f}",
            f"gyro   {r['gyro_x']:+7.3f} {r['gyro_y']:+7.3f} {r['gyro_z']:+7.3f} rad/s"
            f"   T {r['imu_temp_c']:5.1f} C",
            f"theta  {r['theta']:+7.3f} rad  theta_dot {r['theta_dot']:+7.3f}"
            f"  wheel {r['wheel_omega']:+8.2f} rad/s",
            f"torque cmd {r['cmd_torque_nm']:+6.3f}  meas {r['motor_torque_nm']:+6.3f} Nm"
            f"  bus {r['bus_voltage_v']:5.1f} V  iq {r['q_current_a']:+5.2f} A",
            f"dt {r['dt_s'] * 1000:6.3f} ms  slack {r['slack_s'] * 1000:+7.3f} ms"
            f"  SAFETY: {safety}",
        ]
        # Redraw in place: move up, and clear each line to the end so a
        # shorter line never leaves debris from the previous frame.
        if self.lines_drawn:
            sys.stdout.write(f"\033[{self.lines_drawn}A")
        for line in lines:
            sys.stdout.write("\033[K" + line + "\n")
        sys.stdout.flush()
        self.lines_drawn = len(lines)


def capture_serial(port: str, baud: int, out: Path, seconds: float,
                   live: LiveView = None) -> int:
    try:
        import serial
    except ImportError:
        print("pyserial missing.  Install with:", file=sys.stderr)
        print("  python.exe -m pip install pyserial", file=sys.stderr)
        return 1

    try:
        handle = serial.Serial(port, baud, timeout=0.1)
    except Exception as exc:                          # noqa: BLE001
        print(f"cannot open {port}: {exc}", file=sys.stderr)
        print("Is the Teensy attached, and is another program "
              "(PuTTY, a serial monitor) holding the port?", file=sys.stderr)
        return 1

    print(f"capturing {port} @ {baud} -> {out}")
    print("Ctrl-C to stop." if seconds <= 0 else f"stopping after {seconds:.0f}s")

    total = 0
    started = time.time()
    with handle, out.open("wb") as sink:
        while not _stop:
            if seconds > 0 and (time.time() - started) >= seconds:
                break
            chunk = handle.read(4096)
            if not chunk:
                continue
            # Bytes hit the disk FIRST and unconditionally.  Everything
            # below this line is optional; this line is not.
            sink.write(chunk)
            total += len(chunk)
            if live is not None:
                try:
                    live.feed(chunk)
                except Exception:                     # noqa: BLE001
                    # A broken display must never cost us the capture.
                    live = None
    return _finish(out, total, started)


def capture_udp(bind: str, out: Path, seconds: float,
                live: LiveView = None) -> int:
    host, _, port = bind.rpartition(":")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # Generous receive buffer: WiFi delivers in bursts, and a burst that
    # overflows the socket buffer is loss the frame CRC cannot recover.
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    sock.bind((host or "0.0.0.0", int(port)))
    sock.settimeout(0.5)

    print(f"listening on udp/{bind} -> {out}")
    print("Ctrl-C to stop." if seconds <= 0 else f"stopping after {seconds:.0f}s")

    total = 0
    started = time.time()
    with sock, out.open("wb") as sink:
        while not _stop:
            if seconds > 0 and (time.time() - started) >= seconds:
                break
            try:
                datagram, _addr = sock.recvfrom(65535)
            except socket.timeout:
                continue
            # Datagram boundaries are discarded deliberately: framing lives
            # in the magic + CRC, so the decoder neither knows nor cares
            # how the bytes were packetised.  That is what lets the Xiao
            # batch frames per datagram without any host-side change.
            sink.write(datagram)
            total += len(datagram)
            if live is not None:
                try:
                    live.feed(datagram)
                except Exception:                     # noqa: BLE001
                    live = None
    return _finish(out, total, started)


def _finish(out: Path, total: int, started: float) -> int:
    elapsed = time.time() - started
    frames = total // FRAME_SIZE
    print()
    print(f"captured {total} bytes (~{frames} frames) in {elapsed:.1f}s")
    if total == 0:
        print("NOTHING CAPTURED.  Check the port, and that the sketch "
              "flashed is telemetry_test.")
        return 1
    print(f"  {out}")
    print(f"  decode with: ./moteus-venv/bin/python "
          f"tools/telemetry/decode.py {out} --parquet")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="serial port, e.g. COM9 or /dev/ttyACM0")
    source.add_argument("--udp", metavar="HOST:PORT",
                        help="listen for UDP datagrams, e.g. 0.0.0.0:5005")
    parser.add_argument("--baud", type=int, default=115200,
                        help="ignored by Teensy USB CDC; matters only for a "
                             "real UART (default: 115200)")
    parser.add_argument("--out", type=Path, default=None,
                        help="output .bin (default: logs/telemetry_<stamp>.bin)")
    parser.add_argument("-s", "--seconds", type=float, default=0,
                        help="stop after N seconds (default: run until Ctrl-C)")
    parser.add_argument("--live", action="store_true",
                        help="decode and display while capturing (best effort)")
    args = parser.parse_args()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    out = args.out or default_output()
    out.parent.mkdir(parents=True, exist_ok=True)

    live = LiveView() if args.live else None

    if args.udp:
        return capture_udp(args.udp, out, args.seconds, live)
    return capture_serial(args.port, args.baud, out, args.seconds, live)


if __name__ == "__main__":
    sys.exit(main())
