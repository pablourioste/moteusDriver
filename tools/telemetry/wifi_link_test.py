#!/usr/bin/env python3
"""Measure the WiFi telemetry link and give it a pass/fail verdict.

    ./moteus-venv/bin/python tools/telemetry/wifi_link_test.py --announce
    ./moteus-venv/bin/python tools/telemetry/wifi_link_test.py -s 60

This is the acceptance test for the Xiao hop.  ``capture.py`` deliberately
never parses a frame -- its one job is getting bytes onto disk intact.
This script is the opposite: it parses everything, keeps nothing, and
answers the question capture.py cannot, which is *how good is the link*.

Stdlib only, plus decode.py from this directory.  It therefore runs under a
bare ``python3`` as well as under moteus-venv -- useful because the whole
point is to run it while the laptop is on the cube's own network, which is
not the moment to discover a missing dependency.

WHY IT ALSO SENDS
-----------------
The bridge learns where to send telemetry from whoever last sent it a
control packet -- there is no hardcoded laptop IP anywhere, so a DHCP lease
change never becomes a firmware edit.  The consequence is that telemetry
does not start until the laptop says something first.  ``--announce`` is
that first word; the default mode keeps saying it at ``--rate`` Hz, which
doubles as a load test of the control direction while telemetry is flowing
the other way.

HOW LOSS IS COUNTED, AND WHY NOT THE WAY decode.py COUNTS IT
------------------------------------------------------------
decode.py derives loss from consecutive sequence gaps, which is exactly
right for a file: a serial capture cannot reorder.  UDP can.  A datagram
that arrives late makes ``seq`` step backwards, and a backwards step read
as an unsigned gap looks like four billion lost frames.

So this script collects the sequence numbers it saw and computes

    lost = (max_seq - min_seq + 1) - unique_seqs_received

which is insensitive to arrival order, and reports reordering and
duplication separately.  On a link that never reorders the two methods
agree; when they disagree, the difference is itself the finding.
"""

import argparse
import socket
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from decode import FRAME_SIZE, MAGIC, VERSION, crc16_ccitt   # noqa: E402

DEFAULT_CUBE_IP = "192.168.4.1"
CONTROL_PORT = 5005
TELEMETRY_PORT = 5006

# Marks our probe packets so they are recognisable on the bridge's hexdump
# and on a Teensy that is listening.  Deliberately NOT a valid control
# command: this tool measures the pipe, it does not fly the cube.
PROBE_MAGIC = b"CUBEPROBE"


def build_probe(seq: int) -> bytes:
    """PROBE_MAGIC + seq(u32) + host monotonic time in nanoseconds(u64).

    The timestamp rides along so that a future Teensy-side echo can turn
    this into a true round-trip measurement.  Nothing echoes it today, so
    the field is carried and not yet used -- which is cheaper than adding
    it later and invalidating every log taken before.
    """
    return PROBE_MAGIC + struct.pack("<IQ", seq & 0xFFFFFFFF,
                                     time.monotonic_ns())


class LinkStats:
    """Everything measured, and the verdict derived from it."""

    def __init__(self):
        self.datagrams = 0
        self.bytes_rx = 0
        self.frames_ok = 0
        self.crc_errors = 0
        self.version_errors = 0
        self.resync_bytes = 0
        self.reordered = 0
        self.duplicates = 0
        self.probes_sent = 0
        self.seqs = set()
        self.first_seq = None
        self.last_seq = None
        self.max_seq = None
        self.min_seq = None
        # Frames per datagram, as a histogram.  A bridge that batches is
        # visible here; so is one whose batching is being defeated by a
        # flush timeout that fires on every single frame.
        self.per_datagram = {}
        self.first_rx_monotonic = None
        self.last_rx_monotonic = None
        # Wall-clock gap between consecutive datagrams.  The maximum is
        # the number that matters: WiFi power save shows up as an
        # occasional 100 ms+ stall, not as a raised average.
        self.max_gap_s = 0.0
        self.gap_sum_s = 0.0
        self.gap_count = 0

    def note_datagram(self, size: int, frames: int, now: float):
        self.datagrams += 1
        self.bytes_rx += size
        self.per_datagram[frames] = self.per_datagram.get(frames, 0) + 1
        if self.first_rx_monotonic is None:
            self.first_rx_monotonic = now
        else:
            gap = now - self.last_rx_monotonic
            self.max_gap_s = max(self.max_gap_s, gap)
            self.gap_sum_s += gap
            self.gap_count += 1
        self.last_rx_monotonic = now

    def note_frame(self, seq: int):
        self.frames_ok += 1
        if seq in self.seqs:
            self.duplicates += 1
        else:
            self.seqs.add(seq)
        if self.last_seq is not None and seq < self.last_seq:
            self.reordered += 1
        if self.first_seq is None:
            self.first_seq = seq
            self.min_seq = seq
            self.max_seq = seq
        self.min_seq = min(self.min_seq, seq)
        self.max_seq = max(self.max_seq, seq)
        self.last_seq = seq

    @property
    def expected(self) -> int:
        if self.min_seq is None:
            return 0
        return self.max_seq - self.min_seq + 1

    @property
    def lost(self) -> int:
        return max(0, self.expected - len(self.seqs))

    @property
    def loss_pct(self) -> float:
        return (100.0 * self.lost / self.expected) if self.expected else 0.0

    @property
    def elapsed_s(self) -> float:
        if self.first_rx_monotonic is None:
            return 0.0
        return max(1e-9, self.last_rx_monotonic - self.first_rx_monotonic)


def parse_datagram(payload: bytes, stats: LinkStats) -> int:
    """Decode frames out of one datagram.  Returns the count decoded.

    Resyncs one byte at a time on any failure, the same rule decode.py
    uses.  A datagram is NOT assumed to start on a frame boundary --
    nothing in the bridge guarantees that, and assuming it would hide
    exactly the truncation this test exists to catch.
    """
    offset = 0
    decoded = 0
    limit = len(payload)

    while offset + FRAME_SIZE <= limit:
        if (payload[offset] != (MAGIC & 0xFF) or
                payload[offset + 1] != (MAGIC >> 8)):
            offset += 1
            stats.resync_bytes += 1
            continue

        chunk = payload[offset:offset + FRAME_SIZE]
        length = chunk[2]
        version = chunk[3]

        if length != FRAME_SIZE:
            offset += 1
            stats.resync_bytes += 1
            continue
        if version != VERSION:
            stats.version_errors += 1
            offset += 1
            stats.resync_bytes += 1
            continue
        if crc16_ccitt(chunk[:-2]) != struct.unpack_from("<H", chunk, 116)[0]:
            stats.crc_errors += 1
            offset += 1
            stats.resync_bytes += 1
            continue

        stats.note_frame(struct.unpack_from("<I", chunk, 4)[0])
        decoded += 1
        offset += FRAME_SIZE

    # Trailing bytes that are not a whole frame.  With batching this should
    # be zero: the bridge only ever puts whole frames in a datagram, so a
    # non-zero count here means something is splitting frames across
    # packets, which the resync-byte counter above will already be
    # reporting.
    return decoded


def report(stats: LinkStats, args) -> int:
    print()
    print("=== wifi link test ===")
    print(f"  duration        {stats.elapsed_s:.1f} s")
    print(f"  probes sent     {stats.probes_sent} "
          f"({args.rate:.1f} Hz to {args.cube}:{CONTROL_PORT})")
    print(f"  datagrams       {stats.datagrams}")
    print(f"  bytes received  {stats.bytes_rx}")
    print()
    print(f"  frames decoded  {stats.frames_ok}")
    print(f"  unique seqs     {len(stats.seqs)}")
    print(f"  seq range       {stats.min_seq} .. {stats.max_seq} "
          f"({stats.expected} expected)")
    print(f"  frames lost     {stats.lost}  ({stats.loss_pct:.2f}%)")
    print(f"  duplicates      {stats.duplicates}")
    print(f"  reordered       {stats.reordered}")
    print()
    print(f"  CRC failures    {stats.crc_errors}")
    print(f"  version skew    {stats.version_errors}")
    print(f"  resync bytes    {stats.resync_bytes}")

    if stats.elapsed_s > 0:
        rate = stats.frames_ok / stats.elapsed_s
        thruput = stats.bytes_rx / stats.elapsed_s
        print()
        print(f"  frame rate      {rate:.1f} Hz")
        print(f"  throughput      {thruput / 1000.0:.1f} kB/s")

    if stats.gap_count:
        mean_gap = stats.gap_sum_s / stats.gap_count
        print(f"  datagram gap    mean {mean_gap * 1000:.1f} ms, "
              f"max {stats.max_gap_s * 1000:.1f} ms")

    if stats.per_datagram:
        summary = ", ".join(
            f"{k}x{v}" for k, v in sorted(stats.per_datagram.items()))
        print(f"  frames/datagram {summary}")

    print()
    problems = []
    if stats.frames_ok == 0:
        problems.append(
            "no frames decoded -- is the source flashed, and did the "
            "announce reach it?  Check the bridge's USB status line: "
            "uart_rx should be climbing and peer should not be 'none'.")
    if stats.loss_pct > args.max_loss_pct:
        problems.append(
            f"loss {stats.loss_pct:.2f}% exceeds --max-loss-pct "
            f"{args.max_loss_pct:.2f}%")
    if stats.crc_errors > 0:
        problems.append(
            f"{stats.crc_errors} CRC failures -- corruption, not loss.  "
            "A dropped datagram costs whole frames and never fails a CRC, "
            "so this points at the UART (baud mismatch or RX overrun), "
            "not at the radio.")
    if stats.version_errors > 0:
        problems.append(
            f"{stats.version_errors} frames with the wrong version -- the "
            "source firmware and this decoder disagree about the layout.  "
            "Reflash, do not widen the decoder.")
    if stats.max_gap_s > args.max_gap_ms / 1000.0:
        problems.append(
            f"worst datagram gap {stats.max_gap_s * 1000:.0f} ms exceeds "
            f"--max-gap-ms {args.max_gap_ms:.0f} -- check WiFi.setSleep(false) "
            "and how far the laptop is from the antenna")

    if problems:
        print("FAIL")
        for problem in problems:
            print(f"  - {problem}")
        return 1

    print("PASS")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--cube", default=DEFAULT_CUBE_IP,
                        help=f"bridge address (default: {DEFAULT_CUBE_IP})")
    parser.add_argument("--announce", action="store_true",
                        help="send one probe packet and exit -- the minimum "
                             "needed to make the bridge start sending")
    parser.add_argument("-s", "--seconds", type=float, default=30.0,
                        help="measurement duration (default: 30)")
    parser.add_argument("--rate", type=float, default=10.0,
                        help="probe transmit rate, Hz (default: 10)")
    parser.add_argument("--max-loss-pct", type=float, default=1.0,
                        help="verdict threshold on frame loss (default: 1.0)")
    parser.add_argument("--max-gap-ms", type=float, default=250.0,
                        help="verdict threshold on the worst datagram gap "
                             "(default: 250)")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # 1 MB receive buffer.  WiFi delivers in bursts, and a burst that
    # overflows the socket buffer is loss this script would then blame on
    # the radio -- measuring the tool instead of the link.
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    sock.bind(("0.0.0.0", TELEMETRY_PORT))
    sock.settimeout(0.2)

    stats = LinkStats()

    if args.announce:
        with sock:
            sock.sendto(build_probe(0), (args.cube, CONTROL_PORT))
        print(f"announced to {args.cube}:{CONTROL_PORT} from udp/{TELEMETRY_PORT}")
        print("the bridge should now print a 'peer' line on its USB monitor")
        return 0

    period = 1.0 / args.rate if args.rate > 0 else None
    print(f"probing {args.cube}:{CONTROL_PORT} at {args.rate:.1f} Hz, "
          f"listening on udp/{TELEMETRY_PORT} for {args.seconds:.0f}s")
    print("Ctrl-C to stop early")

    started = time.monotonic()
    next_probe = started
    seq = 0

    try:
        with sock:
            while (time.monotonic() - started) < args.seconds:
                now = time.monotonic()
                if period is not None and now >= next_probe:
                    next_probe += period
                    sock.sendto(build_probe(seq), (args.cube, CONTROL_PORT))
                    seq += 1
                    stats.probes_sent += 1

                try:
                    payload, _addr = sock.recvfrom(65535)
                except socket.timeout:
                    continue

                decoded = parse_datagram(payload, stats)
                stats.note_datagram(len(payload), decoded, time.monotonic())
    except KeyboardInterrupt:
        print("\ninterrupted")

    return report(stats, args)


if __name__ == "__main__":
    sys.exit(main())
