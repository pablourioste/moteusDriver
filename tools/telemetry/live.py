#!/usr/bin/env python3
"""Replay a captured .bin through the live display.

    ./moteus-venv/bin/python tools/telemetry/live.py logs/t.bin --speed 4

The live view proper is ``capture.py --live``: it shares the one process
that owns the port, so the display can never contend with the capture for
the device.  This script exists for the other two cases:

  * checking the display renders correctly WITHOUT hardware attached, and
  * replaying an interesting run at slow speed to watch what happened just
    before something went wrong.

It reuses the exact LiveView class capture.py uses, so what you see here is
what you will see live -- a separate implementation would drift.
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from capture import FRAME_SIZE, LiveView   # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", type=Path, help="captured .bin")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="playback rate multiplier (default 1.0 = realtime)")
    parser.add_argument("--chunk", type=int, default=8,
                        help="frames fed per step (default 8)")
    args = parser.parse_args()

    if not args.input.exists():
        print(f"no such file: {args.input}", file=sys.stderr)
        return 1

    blob = args.input.read_bytes()
    view = LiveView(interval=0.05)
    if view.decoder is None:
        print("decoder unavailable", file=sys.stderr)
        return 1

    step = args.chunk * FRAME_SIZE
    # Assume the nominal 400 Hz sample rate for pacing.  Exact pacing is
    # not the point -- this is a display check, not a timing measurement,
    # and the real timing lives in the t_us column.
    delay = (args.chunk / 400.0) / max(args.speed, 1e-6)

    print(f"replaying {args.input} at {args.speed}x -- Ctrl-C to stop\n")
    try:
        for offset in range(0, len(blob), step):
            view.feed(blob[offset:offset + step])
            time.sleep(delay)
    except KeyboardInterrupt:
        pass

    print()
    print(f"replayed {view.frames} frames, {view.lost} lost")
    return 0


if __name__ == "__main__":
    sys.exit(main())
