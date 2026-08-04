# IMU Setup — BMI270 on Teensy 4.1

**Hands-on procedure.** Wire it, prove it works, calibrate it. Follow in order.

| Document | Answers |
|---|---|
| **this file** | **What do I physically do, step by step?** |
| [IMU_BLUEPRINT.md](IMU_BLUEPRINT.md) | Why is it built this way? What do the numbers mean? |
| [INTEGRATION.md](INTEGRATION.md) | Where does this fit in the whole build? (step S2) |

**Target:** BMI270 over **SPI at 10 MHz** on a **Teensy 4.1**. No Raspberry Pi, no
Linux I2C — the IMU runs on the microcontroller.

**Time:** ~30 min wiring and bring-up, plus a 90-minute unattended soak at the end.

---

## Before you start

| Need | Note |
|---|---|
| Teensy 4.1 + USB cable | A **data** cable — many charge-only cables look identical |
| BMI270 breakout, **3.3 V** | SPI pins exposed: SCK, SDI, SDO, CSB |
| Jumper wires, short | At 10 MHz, long leads cause flaky reads |
| Multimeter | For step 1. Not optional |
| Flat, level surface | For calibration |

PlatformIO is **not** assumed — Step 0 installs it.

```
  ⚠  Neither the Teensy nor the BMI270 is 5 V tolerant.  Both have a 3.6 V
     absolute maximum on every pin.  A 5 V wire destroys them.
```

---

# STEP 0 — Talk to the Teensy

**Do this with the IMU disconnected.** Before debugging a sensor you must be able to
build code, flash it, and read output back. Otherwise a failure at Step 2 is ambiguous:
dead IMU, or code that never ran?

## 0.1 — Where to build vs. where to flash (WSL2)

This repo lives in WSL2, and that matters for one specific reason:

> **Teensy flashing re-enumerates the USB device.** The board reboots into its HalfKay
> bootloader, which is a *different* USB device — VID:PID `16C0:0478` — from the running
> sketch (`16C0:0483`). A `usbipd` attachment is bound to one device, so it **drops on
> every upload**. You would re-attach dozens of times a session.

The clean split — **same `platformio.ini`, both sides**:

| Task | Where | Command |
|---|---|---|
| **Build** | WSL2 | `pio run -e board_check` |
| **Flash** | **Windows** | `pio run -e board_check -t upload` |
| **Serial monitor** | **Windows** | `pio device monitor` |

Building in WSL keeps the fast native filesystem and the same shell as your `cmake`
build. Flashing from Windows sidesteps usbipd entirely.

Windows sees the WSL repo directly — no second clone:

```
\\wsl$\Ubuntu\home\pablo_urioste\projects\moteusDriver
```

> **This matters more later.** Step S3 of [INTEGRATION.md](INTEGRATION.md) has you
> running `moteus_tool` over the fdcanusb *while* the Teensy listens on CAN. Under
> pure-WSL that is two simultaneous usbipd attachments, one dropping on every upload.
> Flashing from Windows removes that whole class of problem.

**Pure-WSL is possible** if you prefer one environment: `usbipd attach --wsl --busid X-Y`
before each upload, plus udev rules for non-root access (WSL has none installed by
default). It works — it is just friction on every cycle.

## 0.2 — Install PlatformIO

**Full instructions: [`../INSTALL.md`](../INSTALL.md) Part B.** Summary:

```bash
sudo apt remove platformio                        # ⚠ apt ships a broken 4.3.4
python3 -m venv ~/.platformio-venv
~/.platformio-venv/bin/pip install platformio
export PATH="$HOME/.platformio-venv/bin:$PATH"    # add to ~/.bashrc
```

Verify before continuing:

```bash
which pio          # must be ~/.platformio-venv/bin/pio
pio --version      # must be 6.x, NOT 4.3.4
```

> Ubuntu's `platformio` package is version 4.3.4 from 2020. It crashes on import
> against modern `click` (`'PlatformioCLI' object has no attribute 'resultcallback'`)
> and predates Teensy 4.1 support. If `which pio` says `/usr/bin/pio`, it is shadowing
> your venv — remove it and open a new shell.

**For flashing (Windows side):** VS Code → Extensions → "PlatformIO IDE".

## 0.3 — Verify the build

Everything you need is already in the repo. **Each hardware test is its own folder and
its own PlatformIO environment**, because a Teensy binary can have only one `setup()`:

```
firmware/
  board_check/     ← step 0.4 / 0.5   blink + serial       (no hardware)
  spi_loopback/    ← step 0.6         SPI peripheral       (one jumper)
  imu_spi_test/    ← step 2           BMI270 CHIP_ID       (IMU wired)
```

Pick the one you want with `-e`:

```bash
pio run -e board_check          # build just this test
pio run                         # build ALL of them -- catches bit-rot
```

### ✅ Check 0.3

- [ ] Ends with `[SUCCESS]`
- [ ] Reports Teensy 4.1 memory usage — expect **~8 MB flash free**, ~450 KB RAM1 free

That confirms compiler, framework and board config without any hardware attached. If
this fails, it is a toolchain problem, not a wiring problem.

## 0.4 — Blink: prove flashing works

`firmware/board_check/main.cpp` blinks **short on (200 ms), long off (800 ms)** and
prints over serial. The asymmetry is deliberate: a factory-fresh Teensy blinks *evenly*
at 1 Hz, so a symmetric pattern would not prove your code is the one running.

From **Windows**:

```bash
pio run -e board_check -t upload
```

### ✅ Check 0.4

- [ ] Upload completes without error
- [ ] LED blinks **short on, long off** — not the even factory blink

**If upload fails:** press the physical **PROGRAM button** on the Teensy and retry. That
forces the bootloader manually. If the Teensy Loader window never appears, the cable is
charge-only — swap it.

## 0.5 — Serial: prove you can read output

The same sketch already prints. From **Windows**:

```bash
pio device monitor
```

### ✅ Check 0.5

- [ ] Prints `Teensy alive`
- [ ] `F_CPU = 600000000` — confirms 600 MHz
- [ ] `sizeof(double) = 8` — confirms the double-precision FPU your control math needs
- [ ] `tick` increments once per second

The `while (!Serial && millis() < 3000)` guard is deliberate: it waits for a host to
attach but gives up after 3 s, so the board still runs standalone on battery.

**If the monitor shows nothing:** unplug/replug and retry — the port disappears during
upload and takes a moment to re-enumerate. Confirm the port with `pio device list`.

## 0.6 — SPI loopback (optional, 1 minute, worth it)

Proves the SPI peripheral works *before* you blame the IMU. **Jumper pin 11 to pin 12**
(MOSI→MISO) — no IMU involved.

```bash
pio run -e spi_loopback -t upload
pio device monitor
```

`firmware/spi_loopback/main.cpp` sends five distinctive patterns and checks each comes
back. (`0x00` and `0xFF` are avoided deliberately — they are indistinguishable from a
stuck-low or floating line.)

### ✅ Check 0.6

- [ ] With the jumper: `PASS -- 5/5. SPI peripheral works.`
- [ ] **Remove the jumper and re-run: it must FAIL.** A test that passes either way
      proves nothing

**Remove the jumper before continuing.**

---

## ✅ Step 0 complete

| Check | ✅ |
|---|---|
| PlatformIO builds in WSL, flashes from Windows | ☐ |
| LED blinks your pattern | ☐ |
| Serial prints, `F_CPU` = 600 MHz, `sizeof(double)` = 8 | ☐ |
| SPI loopback passes (optional) | ☐ |

Now a failure at Step 2 means **the IMU**, not the toolchain. That is the entire point
of this step.

---

# STEP 1 — Wire it

**Power off / USB unplugged while wiring.**

```
      BMI270                          Teensy 4.1
   ┌──────────────┐               ┌──────────────────┐
   │ VDD / VIN ───┼───────────────┤ 3.3V             │
   │ GND       ───┼───────────────┤ GND              │
   │ SCK / SCX ───┼───────────────┤ 13   (SCK)       │
   │ SDI / MOSI───┼───────────────┤ 11   (MOSI)      │
   │ SDO / MISO───┼───────────────┤ 12   (MISO)      │
   │ CSB       ───┼───────────────┤ 10   (CS)        │
   └──────────────┘               └──────────────────┘
```

| BMI270 | Teensy | Notes |
|---|---|---|
| VDD / VIN | **3.3V** | ⚠ **not** the 5V/VIN pin |
| GND | GND | |
| SCK / SCX | 13 | |
| SDI / MOSI | 11 | data *into* the IMU |
| SDO / MISO | 12 | data *out of* the IMU |
| CSB | 10 | must be driven, never left floating |

Naming varies by breakout. Some label SPI pins for I2C use — `SDA`→SDI, `SCL`→SCK,
`SDO`→MISO, `CS`→CSB. If in doubt, check the silkscreen against the BMI270 datasheet
pin table.

### ✅ Check 1 — before plugging in USB

With the multimeter, everything unpowered:

- [ ] BMI270 `VDD` traces to Teensy **3.3V**, not 5V
- [ ] GND continuity between both boards
- [ ] No shorts between adjacent pins

Then plug in USB and measure:

- [ ] **3.3 V** (±0.1) at the BMI270 VDD pin
- [ ] No pin on the BMI270 above 3.3 V

**If VDD reads 5 V, unplug immediately.** You have it on the wrong rail.

---

# STEP 2 — Prove the chip answers

The first real question: is the IMU alive and talking? We ask for its ID.

`CHIP_ID` (register 0x00) is read-only, always reads **0x24**, and needs no setup. It
exercises the whole SPI round trip while depending on nothing.

The test is already written: **`firmware/imu_spi_test/main.cpp`**. It reads `CHIP_ID`
and nothing else, and prints a diagnosis if the value is wrong.

```bash
pio run -e imu_spi_test               # WSL:     compile
pio run -e imu_spi_test -t upload     # Windows: flash
pio device monitor                    # Windows: read
```

Expected:

```
=== BMI270 SPI identity test ===
CHIP_ID = 0x24   (expect 0x24)
PASS -- BMI270 responding over SPI
```

### ✅ Check 2

- [ ] Serial prints `CHIP_ID = 0x24` and `PASS`

**It runs at 1 MHz deliberately** (`kSpiHz` in the file). The BMI270 tolerates 10 MHz,
but a slow clock first separates signal-integrity problems from logic bugs. Raise it only
once the identity read is reliable.

Two details in that file worth understanding before you write the real driver — both
are documented in the source:

- **The dummy byte.** SPI reads send `reg | 0x80`, then discard **one** byte, then read.
  The datasheet states the first byte on SDO is garbage. Forget it and every subsequent
  byte shifts by one — the most common BMI270 SPI bug, with no I2C equivalent.
- **The first read is thrown away.** The part powers up in I2C mode and switches to SPI
  on the first CS falling edge, so that read is the mode-select, not a measurement.

### If it fails

| Reading | Meaning | Fix |
|---|---|---|
| `0x00` | No response; MISO stuck low | Check SDO→pin 12, check power |
| `0xFF` | No response; MISO floating | Check CS→pin 10, check wiring |
| Changes each run | Marginal signal | Shorten leads, drop to 500 kHz |
| `0x1B`, `0x2B`, other | A **different** Bosch part (BMI160/BMI088) | Wrong module |
| Correct only sometimes | CS or clock timing | Verify `SPI_MODE0`, check CS is driven |

**Do not continue until this passes.** Everything downstream assumes the bus works.

---

# STEP 3 — Upload the config blob

**This is the step people miss.** The BMI270 ships with its accelerometer and gyroscope
**disabled**. They stay disabled until you upload Bosch's ~8 KB configuration blob. Until
then every data register reads zero — while `CHIP_ID` reads perfectly.

The blob is already in this repo: `third_party/bmi270_config.h`.

Sequence — order and timings matter:

```
1. Write PWR_CONF (0x7C) = 0x00        disable advanced power save
2. Wait 450 µs
3. Write INIT_CTRL (0x59) = 0x00       announce upload
4. For each chunk:
     write INIT_ADDR_0 (0x5B) = (half_word & 0x0F)
     write INIT_ADDR_1 (0x5C) = (half_word >> 4) & 0xFF
     burst-write INIT_DATA (0x5E) + payload   ← CS held LOW throughout
5. Write INIT_CTRL (0x59) = 0x01       upload complete
6. Poll INTERNAL_STATUS (0x21) until (value & 0x0F) == 0x01, up to ~20 ms
```

Two rules that will cost you an afternoon if broken:

> **SPI clock ≤2 MHz during the upload.** At 10 MHz `INTERNAL_STATUS` never reaches
> `0b0001` and it looks exactly like a wiring fault.

> **Do not raise CS mid-chunk.** The internal write pointer auto-increments only within
> one CS assertion. Register byte and payload go in a single transaction.

The full state machine already exists in `src/drivers/ImuDriver.cpp:91-178` — port it,
changing only the two bus functions. Chunk size can go from 32 to 256 bytes on SPI,
cutting the upload from 256 transactions to 32.

### ✅ Check 3

- [ ] `INTERNAL_STATUS & 0x0F == 0x01`

### If it fails

| Symptom | Cause |
|---|---|
| Never reaches `0x01` | SPI clock too fast → drop to 1 MHz |
| Never reaches `0x01` | CS released mid-chunk |
| Reads `0x00` forever | Blob never arrived — check the burst write |
| Was fine, now fails | Did you soft-reset without re-uploading? The blob does not survive a reset |

---

# STEP 4 — Read real data

Now the sensors produce numbers. Read **12 bytes in one burst from 0x0C** — accel XYZ
then gyro XYZ. One transaction keeps the pair coherent; two reads can straddle an
internal update.

Conversion (precompute at init, do not recompute per sample):

```
accel_scale = range_g   · 9.80665 / 32768      → m/s²
gyro_scale  = range_dps · π/180   / 32768      → rad/s
```

Defaults: accel ±2 g (`range_g = 2`), gyro ±500 dps (`range_dps = 500`), ODR 400 Hz.

Print accel XYZ, gyro XYZ, and the magnitude `|a| = √(ax²+ay²+az²)`.

### ✅ Check 4 — the physical sanity test

Rest the board flat and still:

- [ ] `|a|` reads **9.81 ±0.3 m/s²** — this is gravity, and it is the single best proof
      the scale factors are right
- [ ] Gravity sits on **one axis** (~9.81), the other two near 0
- [ ] Gyro reads **< 0.02 rad/s** on all axes
- [ ] Temperature within ~10 °C of ambient, and **not** exactly 0 (the `0x8000` invalid
      sentinel)

Now **turn the board over**:

- [ ] The dominant accel axis **flips sign** and still reads ~9.81

Now **rotate 90°**:

- [ ] Gravity moves cleanly to a different axis, with the expected sign

That last pair catches byte-order and axis-mapping errors that every earlier check
passes. Do not skip them.

**Write down which axis reads +9.81 in which orientation.** You need this for the
estimator's axis mapping later.

### If it fails

| Symptom | Cause |
|---|---|
| All zeros | Blob not uploaded (Step 3), or sensors not enabled in `PWR_CTRL` |
| `\|a\|` ≈ 9.81 but on the wrong axis | Normal — just record the mapping |
| `\|a\|` ≈ 19.6 or 4.9 | Wrong `range_g` in the scale factor |
| Values jump wildly | Reading 2 bytes separately instead of one 12-byte burst |
| Gyro reads ~1.0 rad/s at rest | Wrong `range_dps`, or bias not yet removed (that is Step 5) |

---

# STEP 5 — Calibrate the gyroscope

Every gyro has a zero-rate offset: it reports rotation while perfectly still. Left
uncorrected, that offset **integrates into unbounded tilt error** — the classic silent
balancer failure.

Fixing it is just an average, with one safeguard.

## Procedure

1. Put the rig on a **solid, level, still** surface. Not your hand, not a desk someone
   is typing on.
2. **Do not touch it** for the whole measurement.
3. Collect ~2 seconds of samples (≈800 at 400 Hz).
4. Compute per axis:

```
        1  N-1                          1  N-1
b_g  =  ─  Σ   ω[k]            σ²  =    ─  Σ  (ω[k] − b_g)²
        N k=0                           N k=0
```

5. **Reject if σ is high.** High variance means the rig moved, and the mean is then bias
   *plus* real rotation. This is the guard that stops you baking a bad number in.
6. Store `b_g` and subtract it from every future sample.

Two details already handled in `calibrateGyroBias()`
(`src/drivers/ImuDriver.cpp:304-371`) — port them as-is:

- **Clear any previous bias before measuring**, or you measure the residual of the last
  correction rather than the true offset.
- **Accumulate variance alongside the mean** — that is the motion detector.

### ✅ Check 5

- [ ] Calibration returns success (σ below threshold)
- [ ] After correction, gyro reads **< 0.002 rad/s** on all axes at rest
- [ ] Rotate the board by hand and stop — it should return to ~0, not settle at an offset

## Persisting it

There is no filesystem on the Teensy. Two options:

**A — EEPROM (recommended).** Write the three biases to emulated EEPROM, load at boot:

```cpp
#include <EEPROM.h>
struct GyroCal { double bx, by, bz; float temp_c; uint32_t magic; };
// magic = 0xB M I 2 7 0 sentinel, so an unwritten EEPROM is detectable
```

**B — Build flags.** Measure once, paste into `platformio.ini` as `-D GYRO_BIAS_X=...`.
Simple, but needs a rebuild whenever it changes.

**Warn loudly if no calibration is found.** Silently running with zero bias is exactly
the failure that produces slow, mysterious drift.

> **Record the temperature.** Gyro bias moves with temperature. A calibration taken cold
> and used warm is a known drift source — if the rig runs hot, re-measure warm.

---

# STEP 6 — Calibrate the accelerometer (recommended)

Not yet implemented anywhere in this repo, and it matters: the accelerometer is the
estimator's **only absolute attitude reference**. Gyro drift is corrected by it; nothing
corrects the accelerometer.

## Procedure — six positions

Rest each axis ±1 g against gravity, ~2 s per position, still:

| # | Orientation | Expected (m/s²) |
|---|---|---|
| 1 | +X up | `+9.81, 0, 0` |
| 2 | −X up | `−9.81, 0, 0` |
| 3 | +Y up | `0, +9.81, 0` |
| 4 | −Y up | `0, −9.81, 0` |
| 5 | +Z up (flat) | `0, 0, +9.81` |
| 6 | −Z up (upside down) | `0, 0, −9.81` |

Then per axis:

```
        a_max + a_min                    a_max − a_min
   o =  ─────────────            s  =    ─────────────
              2                              2 · 9.80665
```

`o` is the offset (m/s²), `s` the scale (≈1.0). Apply as
`a_corrected = (a_raw − o) / s`.

### ✅ Check 6

- [ ] After correction, `|‖a‖ − 9.81| < 0.05 m/s²` in **all six** orientations

> **Mounting misalignment is a separate problem — and a bigger one.** If the IMU is
> glued 0.5° off the cube's body frame, its idea of "vertical" is 0.5° off, and the
> balancer will hold that lean forever. That is a **rotation**, not an offset. Measure it
> at assembly and apply it as a fixed body-frame rotation *after* offset/scale
> correction. Do not try to absorb it into the offsets — it corrupts both.

---

# STEP 7 — Soak test

Boring, and it catches a real bug that is invisible for the first hour.

Teensy `micros()` is a **32-bit** counter that wraps every **~71 minutes**. Convert it
naively to seconds and `dt` goes hugely negative once an hour — after which the
complementary filter and the derivative term both explode.

The fix is a 64-bit accumulator that tracks wraps:

```cpp
uint64_t micros64() {
  static uint32_t last = 0;
  static uint64_t high = 0;
  const uint32_t now = micros();
  if (now < last) { high += 0x100000000ULL; }   // wrapped
  last = now;
  return high + now;
}
```

Run the read loop for **90+ minutes**, logging min/max `dt`.

### ✅ Check 7

- [ ] `dt` **never negative**, across the full 90 minutes
- [ ] `dt` stays near nominal (2.5 ms at 400 Hz) — no unexplained spikes
- [ ] No dropped/invalid reads

Only after this passes should the IMU feed a control loop.

---

# Final checklist

| Step | Check | ✅ |
|---|---|---|
| 0 | Teensy blinks, prints serial, `F_CPU`=600 MHz, `sizeof(double)`=8 | ☐ |
| 1 | 3.3 V at VDD, grounds common, no shorts | ☐ |
| 2 | `CHIP_ID` reads `0x24` | ☐ |
| 3 | `INTERNAL_STATUS & 0x0F == 0x01` | ☐ |
| 4 | `\|a\|` = 9.81 ±0.3, flips sign when inverted | ☐ |
| 5 | Gyro < 0.002 rad/s after bias correction | ☐ |
| 6 | `\|‖a‖ − 9.81\|` < 0.05 in all six positions | ☐ |
| 7 | 90-minute soak, `dt` never negative | ☐ |
| — | Raised SPI clock to 10 MHz and re-ran checks 2–4 | ☐ |
| — | Recorded axis mapping (which axis is +9.81, when) | ☐ |
| — | Calibration persisted to EEPROM / build flags | ☐ |

With all boxes ticked, the IMU is ready and
[INTEGRATION.md](INTEGRATION.md) step **S2 is complete**. Next is S3, CAN bring-up.

---

## Quick reference

| Register | Addr | Value | Meaning |
|---|---|---|---|
| `CHIP_ID` | 0x00 | `0x24` | identity |
| `DATA_8` (accel X LSB) | 0x0C | — | start of the 12-byte burst |
| `INTERNAL_STATUS` | 0x21 | `0x01` | blob accepted |
| `ACC_CONF` | 0x40 | `0xA0\|odr` | accel rate + filter |
| `ACC_RANGE` | 0x41 | `0x00` | ±2 g |
| `GYR_CONF` | 0x42 | `0xE0\|odr` | gyro rate + performance |
| `GYR_RANGE` | 0x43 | `0x02` | ±500 dps |
| `INIT_CTRL` | 0x59 | `0x00`/`0x01` | blob upload gate |
| `INIT_DATA` | 0x5E | — | blob payload |
| `PWR_CONF` | 0x7C | `0x00` | disable power save |
| `PWR_CTRL` | 0x7D | `0x0E` | enable acc+gyr+temp |
| `CMD` | 0x7E | `0xB6` | soft reset |

**SPI:** mode 0, MSB first, `reg|0x80` to read + **discard the first byte**, ≤2 MHz for
blob upload, 10 MHz after.
