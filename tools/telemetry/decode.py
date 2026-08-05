#!/usr/bin/env python3
"""Decode a binary telemetry capture into a pandas DataFrame / Parquet.

Runs in WSL against ``moteus-venv`` (needs pandas + pyarrow):

    ./moteus-venv/bin/python tools/telemetry/decode.py logs/t.bin --parquet

THE RESYNC RULE
---------------
On any failure -- bad magic, wrong version, bad CRC -- this advances by
exactly ONE byte and rescans.  It never assumes frames are aligned.

That is not defensive programming for its own sake: the eventual transport
is UDP over WiFi, where a truncated datagram leaves a partial frame in the
stream.  A decoder that skipped forward by a whole frame on error would
stay permanently misaligned after a single mid-frame truncation.  Advancing
one byte costs a little CPU on a corrupt file and nothing at all on a clean
one, because a valid frame is consumed whole.

Loss is COUNTED AND REPORTED, never silently dropped.  A 5% loss rate over
WiFi is a diagnosis; a decoder that hides it turns that into a mystery.
"""

import argparse
import struct
import sys
from pathlib import Path

# Must match include/core/TelemetryFrame.hpp exactly.  The C++ side pins
# this with static_assert(sizeof(TelemetryFrame) == 118); the assertion
# below is the other half of that contract.
#
# '<' is doing real work here: little-endian AND no alignment padding,
# matching __attribute__((packed)).  Without it Python inserts padding and
# every field after the first misaligned one is garbage.
FRAME_FORMAT = "<HBBII3h3h3f3f16fBBBBH"
FRAME_SIZE = struct.calcsize(FRAME_FORMAT)
assert FRAME_SIZE == 118, f"format string yields {FRAME_SIZE}, expected 118"

MAGIC = 0xA5C3
VERSION = 1

# Order must match the struct field order in the header.
FIELD_NAMES = [
    "magic", "length", "version", "seq", "t_us",
    "raw_accel_x", "raw_accel_y", "raw_accel_z",
    "raw_gyro_x", "raw_gyro_y", "raw_gyro_z",
    "accel_x", "accel_y", "accel_z",
    "gyro_x", "gyro_y", "gyro_z",
    "imu_temp_c",
    "theta", "theta_dot", "wheel_omega",
    "motor_position_rev", "motor_velocity_rev_s", "motor_torque_nm",
    "q_current_a", "bus_voltage_v", "motor_temp_c",
    "cmd_torque_nm",
    "term_theta", "term_theta_dot", "term_omega",
    "dt_s", "slack_s",
    "mode", "fault", "safety", "flags", "crc16",
]

FLAG_BITS = {
    "armed": 1 << 0,
    "torque_clamped": 1 << 1,
    "imu_valid": 1 << 2,
    "motor_valid": 1 << 3,
    "body_valid": 1 << 4,
    "dry_run": 1 << 5,
    "overflow": 1 << 6,
}

SAFETY_NAMES = {
    0: "OK", 1: "NOT_READY", 2: "TILT_EXCEEDED", 3: "WHEEL_SATURATED",
    4: "SENSOR_FAULT", 5: "MOTOR_FAULT", 6: "UNCONFIGURED",
}


def _crc16_table():
    """CRC-16/CCITT-FALSE table: poly 0x1021, MSB-first.

    An INDEPENDENT implementation of src/core/TelemetryFrame.cpp, not a
    port of it.  Both are checked against the published test vector
    (0x29B1 over "123456789"), so a bug would have to occur identically in
    two separately written implementations to slip through.
    """
    table = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
        table.append(crc)
    return table


_CRC_TABLE = _crc16_table()


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc = ((crc << 8) ^ _CRC_TABLE[((crc >> 8) ^ byte) & 0xFF]) & 0xFFFF
    return crc


assert crc16_ccitt(b"123456789") == 0x29B1, "CRC implementation is wrong"


class DecodeStats:
    """What the decoder saw.  Printed after every run."""

    def __init__(self):
        self.frames_ok = 0
        self.crc_errors = 0
        self.version_errors = 0
        self.resync_bytes = 0
        self.seq_gaps = 0
        self.frames_lost = 0
        self.overflow_frames = 0

    def report(self, total_bytes: int) -> str:
        expected = self.frames_ok + self.frames_lost
        loss_pct = (100.0 * self.frames_lost / expected) if expected else 0.0
        lines = [
            f"  bytes read      {total_bytes}",
            f"  frames decoded  {self.frames_ok}",
            f"  CRC failures    {self.crc_errors}",
            f"  version skew    {self.version_errors}",
            f"  resync bytes    {self.resync_bytes}",
            f"  seq gaps        {self.seq_gaps}",
            f"  frames lost     {self.frames_lost}  ({loss_pct:.2f}%)",
            f"  ring overflows  {self.overflow_frames}",
        ]
        return "\n".join(lines)


def decode_bytes(blob: bytes, stats: DecodeStats = None):
    """Yield decoded frame tuples, resyncing byte-by-byte on any error."""
    if stats is None:
        stats = DecodeStats()

    offset = 0
    limit = len(blob)
    last_seq = None

    while offset + FRAME_SIZE <= limit:
        # Cheap pre-filter: check the magic before doing a CRC.  On a clean
        # stream this is the only branch taken.
        if blob[offset] != (MAGIC & 0xFF) or blob[offset + 1] != (MAGIC >> 8):
            offset += 1
            stats.resync_bytes += 1
            continue

        chunk = blob[offset:offset + FRAME_SIZE]
        values = struct.unpack(FRAME_FORMAT, chunk)
        record = dict(zip(FIELD_NAMES, values))

        if record["length"] != FRAME_SIZE:
            offset += 1
            stats.resync_bytes += 1
            continue

        if record["version"] != VERSION:
            # A real frame from different firmware.  Refuse it loudly
            # rather than misparsing fields that may have moved.
            stats.version_errors += 1
            offset += 1
            stats.resync_bytes += 1
            continue

        if crc16_ccitt(chunk[:-2]) != record["crc16"]:
            stats.crc_errors += 1
            offset += 1
            stats.resync_bytes += 1
            continue

        # Valid frame.
        seq = record["seq"]
        if last_seq is not None:
            gap = (seq - last_seq) & 0xFFFFFFFF
            if gap > 1:
                stats.seq_gaps += 1
                stats.frames_lost += gap - 1
        last_seq = seq

        if record["flags"] & FLAG_BITS["overflow"]:
            stats.overflow_frames += 1

        stats.frames_ok += 1
        yield record
        offset += FRAME_SIZE

    return stats


def unwrap_micros(series):
    """Undo the 32-bit micros() rollover.

    t_us wraps every 71.6 minutes (2^32 us).  Any consumer that diffs the
    raw value sees one enormous negative dt per wrap; this adds 2^32 at
    each backward step so the result is monotonic.

    Handled here rather than by widening the frame to a 64-bit timestamp:
    that would cost 4 bytes on every record forever to avoid these six
    lines.
    """
    import numpy as np

    values = np.asarray(series, dtype=np.int64)
    if len(values) == 0:
        return values
    deltas = np.diff(values)
    # A backward step of any size is a wrap: the sequence is monotonic at
    # the source, so time never actually goes backwards.
    wraps = np.cumsum(np.where(deltas < 0, 1, 0))
    offsets = np.concatenate([[0], wraps]) * (1 << 32)
    return values + offsets


def to_dataframe(records, check_scale: bool = True):
    import pandas as pd

    df = pd.DataFrame(list(records))
    if df.empty:
        return df

    df["t_us_unwrapped"] = unwrap_micros(df["t_us"])
    df["t_s"] = (df["t_us_unwrapped"] - df["t_us_unwrapped"].iloc[0]) * 1e-6

    for name, bit in FLAG_BITS.items():
        df[f"flag_{name}"] = (df["flags"] & bit) != 0

    df["safety_name"] = df["safety"].map(SAFETY_NAMES).fillna("UNKNOWN")
    df["power_w"] = df["bus_voltage_v"] * df["q_current_a"]
    df["tracking_error_nm"] = df["cmd_torque_nm"] - df["motor_torque_nm"]

    # Drop the wire-protocol columns: they carry no physical information
    # and only invite someone to plot a CRC.
    df = df.drop(columns=["magic", "length", "version", "crc16"])

    if check_scale:
        _verify_scale(df)
    return df


def _verify_scale(df):
    """Assert the firmware's SI conversion matches the datasheet.

    This is the cross-check the raw columns exist for.  If the firmware's
    range register and its scale factor ever disagree, the difference shows
    up here on the next capture instead of as a plausible-but-wrong number
    in a plot.
    """
    import numpy as np

    import scale as scale_mod

    expected_ax = scale_mod.accel_counts_to_ms2(df["raw_accel_x"].to_numpy())
    worst_accel = float(np.nanmax(np.abs(expected_ax - df["accel_x"].to_numpy())))

    expected_gx = scale_mod.gyro_counts_to_rad_s(df["raw_gyro_x"].to_numpy())
    worst_gyro = float(np.nanmax(np.abs(expected_gx - df["gyro_x"].to_numpy())))

    # Tolerance is float32 round-off, not physics: the firmware stores the
    # SI value as float32 while this recomputes in float64.
    if worst_accel > 1e-3:
        print(f"  WARNING accel scale mismatch: worst {worst_accel:.6f} m/s^2")
        print("          firmware range register and scale.py disagree")
    if worst_gyro > 1e-4:
        print(f"  WARNING gyro scale mismatch: worst {worst_gyro:.6f} rad/s")
        print("          check GYR_RANGE -- the register order is INVERTED")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", type=Path, help="raw .bin capture")
    parser.add_argument("--parquet", action="store_true",
                        help="write <input>.parquet")
    parser.add_argument("--csv", action="store_true",
                        help="write <input>.csv (matches CsvLogger convention)")
    parser.add_argument("--out", type=Path, default=None,
                        help="explicit output path (extension picks format)")
    parser.add_argument("--no-check-scale", action="store_true",
                        help="skip the raw-vs-SI datasheet cross-check")
    parser.add_argument("--head", type=int, default=0,
                        help="print the first N decoded rows")
    args = parser.parse_args()

    if not args.input.exists():
        print(f"no such file: {args.input}", file=sys.stderr)
        return 1

    blob = args.input.read_bytes()
    stats = DecodeStats()
    records = list(decode_bytes(blob, stats))

    print(f"=== decode {args.input} ===")
    print(stats.report(len(blob)))

    if not records:
        print("\nNo valid frames.  Check that the capture came from "
              "telemetry_test and not a text sketch.")
        return 1

    df = to_dataframe(records, check_scale=not args.no_check_scale)

    duration = df["t_s"].iloc[-1] if len(df) > 1 else 0.0
    rate = (len(df) - 1) / duration if duration > 0 else 0.0
    print(f"  duration        {duration:.2f} s")
    print(f"  mean rate       {rate:.1f} Hz")

    if args.head:
        cols = ["t_s", "seq", "accel_x", "accel_y", "accel_z",
                "gyro_x", "gyro_y", "gyro_z", "imu_temp_c"]
        print()
        print(df[cols].head(args.head).to_string(index=False))

    outputs = []
    if args.out:
        outputs.append(args.out)
    if args.parquet:
        outputs.append(args.input.with_suffix(".parquet"))
    if args.csv:
        outputs.append(args.input.with_suffix(".csv"))

    for path in outputs:
        if path.suffix == ".parquet":
            df.to_parquet(path, index=False)
        else:
            df.to_csv(path, index=False)
        size_kb = path.stat().st_size / 1024.0
        print(f"  wrote {path}  ({size_kb:.1f} kB)")

    return 0


if __name__ == "__main__":
    sys.exit(main())
