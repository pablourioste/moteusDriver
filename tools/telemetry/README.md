# Binary telemetry

Fixed-size binary records instead of `printf`. One 118-byte frame per
control cycle, with a magic header, sequence number and CRC-16 so the
stream resynchronises after loss and reports exactly how much was lost.

USB today; the same frame goes over WiFi via the Xiao ESP32-C6 later with
**no change to anything in this directory**.

## Quick start

```bash
# 1. build + flash                                    WSL / Windows
pio run -e telemetry_test
#    flash .pio/build/telemetry_test/firmware.hex with Teensy Loader

# 2. capture                             Windows python, from WSL terminal
python.exe tools/telemetry/capture.py --port COM9 -s 60 --live

# 3. decode                                                          WSL
./moteus-venv/bin/python tools/telemetry/decode.py \
    logs/telemetry_<stamp>.bin --parquet

# 4. analyse                                                         WSL
#    open tools/telemetry/analyze.ipynb, kernel = moteus-venv
```

Or use the VS Code tasks (`Ctrl+Shift+P` -> "Tasks: Run Task"):
`telemetry: build firmware`, `telemetry: capture`, `telemetry: decode
latest`, `telemetry: replay live`.

## Why binary

The end transport is WiFi, which is a *packet* medium — datagrams drop,
reorder and arrive in bursts. A text stream has no record boundaries, so
losing 40 bytes mid-line leaves the parser permanently misaligned. A
fixed-size record with magic + CRC recovers on the very next record.

It is also the only thing that fits. At 200 Hz the frame is 23.6 kB/s;
the equivalent ASCII is ~2.5x larger. A 115200-baud UART tops out at
11.5 kB/s, so text does not fit the Xiao link at all. Teensy USB CDC
ignores `Serial.begin()` and runs at 480 Mbit/s, which is why USB works
today either way.

## Files

| File | Role |
|---|---|
| `capture.py` | serial **or** UDP -> raw `.bin`. Never parses. `--live` adds a display |
| `decode.py` | resync + CRC verify + unpack -> DataFrame -> Parquet/CSV |
| `scale.py` | BMI270 scale factors **from the datasheet**, independent of the C++ |
| `live.py` | replay a `.bin` through the live display (no hardware needed) |
| `analyze.ipynb` | the four analysis views |

The firmware side lives in `include/core/Telemetry*.hpp`,
`src/core/TelemetryFrame.cpp` and `firmware/telemetry_test/`.

## Dependencies

Split, because capture is dumb bytes and analysis is not:

```bash
python.exe -m pip install pyserial          # Windows: capture only
./moteus-venv/bin/pip install pandas pyarrow # WSL: decode + notebook
```

`numpy`, `scipy`, `matplotlib`, `pyserial` and `ipykernel` are already in
`moteus-venv`.

## Reading the decode summary

Printed on every decode. Read it before trusting a plot.

```
  frames decoded  23940
  CRC failures    0        non-zero over USB means a real bug
  resync bytes    0        bytes skipped hunting for the next frame
  seq gaps        0        how many discontinuities
  frames lost     0        how many frames those gaps cost
  ring overflows  0        producer-side loss: the drain fell behind
```

**`ring overflows` vs `frames lost` is the useful distinction.** Overflow
means the ring filled because the drain could not keep up — a firmware
problem, fixed with a bigger budget or ring. Loss without overflow means
frames left the ring and never arrived — a link problem, expected on WiFi
and a bug on USB.

## Gotchas

- **`telemetry_test` emits binary.** Opening it in PuTTY or a serial
  monitor shows garbage, and a monitor holding the port blocks the
  capture. `imu_calibrate` is still interactive text and is unaffected.
- **`t_us` wraps every 71.6 minutes.** `decode.py` unwraps it. Anything
  diffing the raw column sees one huge negative `dt` per wrap.
- **`q_current_a` is 0** until `MoteusDriverWrapper::FromQuery` is
  extended — it currently drops `q_current`, `d_current`, `power` and
  board temperature from `Query::Result`. The power panel is structurally
  correct and numerically empty until then.
- **Freeze the field list before recording data you care about.** Adding a
  field bumps `kTelemetryVersion`, and the decoder then refuses older
  captures rather than misparsing them.

## Going to WiFi

Only the transport changes:

1. Teensy `Serial1` TX -> Xiao RX, **common ground**. Both are 3.3 V, so
   no level shifter — but Teensy 4.1 pins are not 5 V tolerant, so meter
   it first.
2. Swap `UsbCdcSink` for a `Serial1Sink` at 2 Mbaud, ring in `DMAMEM` (or
   `arm_dcache_flush()` before arming DMA — the M7 D-cache will otherwise
   transmit stale bytes).
3. Xiao sketch: UART bytes -> UDP. It never parses a frame. Batch ~12
   frames per datagram (12 x 118 = 1416 B, inside the 1472 B MTU).
4. Host: `capture.py --udp 0.0.0.0:5005`. **`decode.py`, `live.py` and the
   notebook are untouched** — framing lives in the magic + CRC, so datagram
   boundaries are irrelevant.

Then the WSL/Windows split disappears from the telemetry path entirely:
UDP arrives in WSL directly, no COM port and no `python.exe`.
