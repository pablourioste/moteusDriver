# BMI270 Design Reference — Teensy 4.1 / SPI

**What the IMU subsystem is, and why.** Datasheet-grounded design notes, protocol
analysis, and calibration mathematics.

| Document | Answers |
|---|---|
| [IMU_SETUP.md](IMU_SETUP.md) | **How do I wire, test and calibrate it?** ← start here for hands-on work |
| **this file** | **Why is it built this way? What do the numbers mean?** |
| [INTEGRATION.md](INTEGRATION.md) | Where does the IMU fit in the whole build sequence? (step S2) |
| [ARCHITECTURE.md](ARCHITECTURE.md) | Where does the driver sit in the codebase? |

**Datasheet of record:** `datasheets/IMU_datasheet.pdf` — Bosch BMI270, rev 1.5,
March 2023, BST-BMI270-DS000-07.
**Target:** Teensy 4.1 (NXP i.MX RT1060), BMI270 over **SPI at 10 MHz**.

---

## 0. Target platform

The IMU runs on the **Teensy 4.1**, over **SPI**. There is no Raspberry Pi, no Linux
SBC, and no `/dev/i2c-*` in the target system.

| | Legacy (host) | **Target (Teensy)** |
|---|---|---|
| Processor | Linux x86-64 (WSL2 / SBC) | **Teensy 4.1, Cortex-M7 @600 MHz** |
| Bus | I2C at 400 kHz via `ioctl` | **SPI at 10 MHz via `SPI.h`** |
| Driver | `src/drivers/ImuDriver.cpp` | `src/embedded/Bmi270SpiDriver.cpp` |
| Timing | `clock_gettime` | `micros()` (64-bit accumulated) |
| Sleep | `nanosleep` | `delayMicroseconds` |
| Status | implemented, kept for host-side simulation | **to be written — INTEGRATION.md step S2** |

The existing Linux/I2C driver is **not deleted**. It stays for host-side testing and as
the reference implementation whose register logic ports across ~90% intact.

---

## 1. What ports, and what does not

The existing `src/drivers/ImuDriver.cpp` is a complete, working BMI270 driver. Only
four functions in it are Linux-specific.

| Concern | Lines | Ports? |
|---|---|---|
| `writeRegister` / `readRegisters` | 77-89 | ❌ **Rewrite for SPI** — the only real bus work |
| `SleepMicroseconds` | 46-51 | ❌ → `delayMicroseconds` |
| `MonotonicSeconds` | 53-58 | ❌ → `micros()`, 64-bit accumulated |
| Config blob upload state machine | 91-178 | ✅ unchanged (except burst size) |
| Init sequence, CHIP_ID, soft reset | 180-268 | ✅ unchanged |
| Scale factors (LSB → SI) | 262-265 | ✅ unchanged |
| Sample decode, 12-byte burst | 270-302 | ✅ unchanged |
| `calibrateGyroBias` + variance check | 304-371 | ✅ unchanged |

**The register map moves to a shared header** — `include/drivers/Bmi270Registers.hpp` —
so the I2C and SPI drivers cannot drift apart.

### The four SPI differences

1. **Read = `reg | 0x80`, then a dummy byte.** The datasheet is explicit (lines 11749,
   11815): *"the first byte received from the device via the SDO line corresponds to a
   dummy byte and the 2nd byte corresponds to the value read."* Discard byte 1. This is
   the most common BMI270 SPI bug and has **no I2C equivalent**.
2. **Write = `reg & 0x7F`**, then the value.
3. **Power-on mode latch.** The part boots in I2C mode; the first CS falling edge
   switches it to SPI, so the *first* read is invalid by design. The existing defensive
   triple-read at `ImuDriver.cpp:202-215` happens to be exactly right for SPI — keep it.
4. **Burst without releasing CS.** The internal write pointer auto-increments only
   within one CS assertion. Do not raise CS between the register byte and the payload.

### SPI parameters

| Setting | Value | Source |
|---|---|---|
| Mode | **0** (CPOL=0, CPHA=0) — mode 3 also works, auto-selected | datasheet 11613 |
| Bit order | MSB first | |
| Clock, blob upload | **≤2 MHz** | Bosch reference driver |
| Clock, normal operation | **10 MHz** | datasheet §1 (BMI270 max) |
| Data captured | on rising edge of SCK | datasheet 11831 |

**Uploading the blob at 10 MHz fails.** `INTERNAL_STATUS` never reaches `0b0001` and you
will hunt a wiring fault that does not exist. Drop to ≤2 MHz for the upload, then raise
the clock.

---

## 2. Datasheet constraints that drive the design

| # | Constraint | Locus | Consequence |
|---|---|---|---|
| D1 | `CHIP_ID` = **0x24** at reg 0x00 | §5.2.1 | The identity gate. Any other value = wrong part or dead bus |
| D2 | Sensors **stay disabled** until an ~8 KB config blob is uploaded; `INTERNAL_STATUS` (0x21) low nibble must read `0b0001` | §4.x | **Non-negotiable.** Without it every data register reads zero while CHIP_ID reads perfectly — the classic silent failure |
| D3 | Soft reset `CMD`(0x7E)=`0xB6`, then **2 ms** | §5.2.x | Implemented |
| D4 | Data burst: **12 bytes from 0x0C** (accel XYZ then gyro XYZ) | §5.2.x | One transaction keeps the pair coherent. Reading separately can straddle an internal update |
| D5 | First SPI read returns a **dummy byte** | 11749, 11815 | See §1 |
| D6 | SPI max **10 MHz**, 4-wire | §1 | Teensy LPSPI does 30 MHz, so the sensor is the limit |
| D7 | Gyro noise **<7 mdps/√Hz** in performance mode | §1 | `GYR_CONF=0xE0\|odr` already selects it |
| D8 | Gyro offset comp: **61 mdps** LSB, **±31 dps** range | ~4930 | Bounds any on-chip offset scheme |
| D9 | **CRT** — gyro sensitivity compensation, residual typ. **0.4%** | §1, §5144 | **Unused.** Real accuracy left on the table |
| D10 | **2 KB FIFO** (accel+gyro+timestamp) | §1 | **Unused.** Relevant to jitter |
| D11 | **Sensortime**, host↔sensor sync <40 µs | §1 | **Unused.** Currently host-timestamped at read time |
| D12 | 2 programmable INT pins | §6.2 T17 | **Unused** — the driver polls |
| D13 | Temperature: `0x8000` = invalid; `/512 + 23 °C` | §5.2.x | Implemented incl. the sentinel |
| D14 | ODR: gyro 25 Hz–6.4 kHz, accel 0.78–1600 Hz | §1 | `odr=0x0A` (400 Hz), above the 200 Hz loop |
| D15 | VDD 1.71–3.6 V, VDDIO 1.2–3.6 V | §1, §2 | 3.3 V. **No pin is 5 V tolerant** |

---

## 3. Why SPI

Both buses work. SPI has headroom; I2C does not.

| | 12-byte burst | % of a 5 ms cycle |
|---|---|---|
| I2C @ 400 kHz (legacy) | ~350 µs | 7.0% |
| I2C @ 1 MHz (Fm+ ceiling) | ~140 µs | 2.8% |
| **SPI @ 10 MHz** | **~15 µs** | **0.3%** |

| Criterion | I2C | SPI |
|---|---|---|
| Max clock | 1 MHz (Fm+) | **10 MHz** (BMI270 limit) |
| Pins | 2 | 4 + CS |
| Pull-ups | **required**, 2.2–4.7 kΩ | none |
| Quirk | watchdog needs SCL >100 kHz | first read is a dummy byte |
| Headroom for >1 kHz sampling | no | **yes** |

At 200 Hz the bus is not the bottleneck either way — **sample-timing jitter is**, and
neither bus fixes that (see §6). SPI is chosen for the headroom: it costs two extra pins
and buys a 20× margin, which is what makes a future move to 1 kHz sampling or FIFO burst
reads possible without re-wiring.

---

## 4. Electrical

```
  ⚠  The Teensy 4.1 is NOT 5 V tolerant.  GPIO abs-max is OVDD+0.3 V = 3.6 V.
     The BMI270 is NOT 5 V tolerant either — same 3.6 V ceiling.
     A 5 V signal on any pin of either part destroys it.
```

- **Supply:** 3.3 V. Many breakouts accept 5 V on `VIN` through their own regulator —
  that is the *breakout's* regulator, not the BMI270's. The chip still sees 3.3 V.
- **No pull-ups needed** on SPI — that is an I2C requirement only.
- **CS must be driven**, never floating. A floating CS latches the part into an
  indeterminate bus mode at power-on.
- **Keep leads short.** At 10 MHz, long dupont wiring is the usual cause of marginal
  SPI. If reads are flaky, drop to 1 MHz and see if they stabilise — that isolates
  signal integrity from logic bugs.

### Pin mapping — Teensy 4.1 ↔ BMI270

```
      BMI270 breakout                    Teensy 4.1
   ┌────────────────────┐            ┌──────────────────────┐
   │ VDD / VIN      ────┼────────────┤ 3.3V                 │
   │ GND            ────┼────────────┤ GND                  │
   │ SCK  / SCX     ────┼────────────┤ 13   SCK0            │
   │ SDI  / MOSI    ────┼────────────┤ 11   MOSI0           │
   │ SDO  / MISO    ────┼────────────┤ 12   MISO0           │
   │ CSB            ────┼────────────┤ 10   CS  (any GPIO)  │
   │ INT1           ────┼────────────┤ (optional, future)   │
   └────────────────────┘            └──────────────────────┘
```

| BMI270 pin | Function | Teensy | Required |
|---|---|---|---|
| VDD / VDDIO | supply | 3.3V | yes |
| GND | ground | GND | yes |
| SCK (SCX) | clock | 13 | yes |
| SDI | master out | 11 | yes |
| SDO | master in | 12 | yes |
| CSB | chip select, active low | 10 | yes — **must be driven** |
| INT1 | data-ready IRQ | any GPIO | no — future (§6) |

> **In SPI mode SDO is the data output**, not an address strap. The 0x68/0x69 address
> selection that mattered on I2C is irrelevant here.

---

## 5. Calibration mathematics

The *procedure* is in [IMU_SETUP.md](IMU_SETUP.md). This is the reasoning.

### 5.1 Gyroscope zero-rate bias — implemented

Held still, the mean of each axis is the bias:

```
        1  N-1                          1  N-1
b_g  =  ─  Σ   ω_raw[k]        σ²  =    ─  Σ  (ω_raw[k] − b_g)²
        N k=0                           N k=0
```

Applied per sample: `ω = ω_raw · s_g − b_g`.

Two details in `calibrateGyroBias()` worth preserving:

- **Prior bias is cleared before measuring** — otherwise it measures the residual of the
  previous correction, not the true offset.
- **Variance is accumulated alongside the mean** — this is the motion detector. High σ
  means the rig moved and the result must be rejected. The threshold is <0.02 rad/s.

### 5.2 Accelerometer calibration — missing

Unaddressed today, and it feeds `accelAngle()` — the estimator's **only absolute
attitude reference**. Per axis:

```
   a_corrected = (a_raw − o) / s          o = offset (m/s²), s = scale (≈1)
```

Six-position method, resting each axis ±1 g against gravity:

```
        a_max + a_min                    a_max − a_min
   o =  ─────────────            s  =    ─────────────
              2                              2 · 9.80665
```

> **Mounting misalignment matters more than offset.** An IMU mounted 0.5° off the body
> frame gives a 0.5° error in "vertical", which the balancer will faithfully hold —
> leaning permanently. That is a **rotation**, not a translation: capture it as a fixed
> body-frame rotation determined at assembly. Folding it into the accel offsets corrupts
> both.

### 5.3 Runtime application order

Not arbitrary — offset precedes scale, both precede rotation:

```
raw int16 ──► ×scale_factor ──► −offset ──► ×axis_correction ──► body-frame rotation
             (LSB → SI)        (bias)      (sensitivity)        (mounting)
```

### 5.4 Persistence

Bias currently lives in `Config` and dies with the process. On the Teensy there is no
filesystem, so persist to **EEPROM** (`EEPROM.h`, emulated in flash) or bake into
`build_flags` after measuring.

**Record the temperature it was measured at** — gyro bias moves with temperature, and a
calibration taken cold and used warm is a known drift source.

---

## 6. Known gaps, ranked

| # | Gap | Why it matters |
|---|---|---|
| **G1** | **Polling, not INT1/FIFO** (D10, D12) | Every sample carries 0–2.5 ms of unknown age, and `data.timestamp` is set *after* the transfer, so it misattributes the instant. An estimator differentiating that jitter sees phantom rate |
| **G2** | **Host-side timestamp** (D11) | Sensortime would tighten the `dt` the estimator integrates |
| **G3** | **No accel calibration** | §5.2 — feeds the only absolute attitude reference |
| **G4** | **Bias not persisted** | Every run re-calibrates or runs biased |
| **G5** | **CRT unused** (D9) | Gyro *scale* error uncorrected; shows up as drift that grows faster during fast rotation |
| **G6** | **No staleness detection** | A stuck-but-responding bus returning all zeros is indistinguishable from "perfectly level and still" |

G1 is the one worth doing after basic bring-up works. The others are refinements.

---

## 7. Pipeline

```
 ┌──────────────┐
 │  BMI270      │  16-bit signed, little-endian, ODR 400 Hz
 └──────┬───────┘
        │  [future: INT1 data-ready — G1]
        v
 ┌─────────────────────────────────────────────┐
 │ readRegisters(0x0C, buf, 12)                │  ONE burst, CS held low:
 │   SPI: send 0x0C|0x80, drop dummy, read 12  │  accel+gyro coherent
 └──────┬──────────────────────────────────────┘
        v
 ┌─────────────────────────────────────────────┐
 │ UNIT CONVERSION   (precomputed at init)     │
 │   accel_scale = range_g · 9.80665 / 32768   │  → m/s²
 │   gyro_scale  = range_dps · π/180 / 32768   │  → rad/s
 └──────┬──────────────────────────────────────┘
        v
 ┌─────────────────────────────────────────────┐
 │ OFFSET CORRECTION                           │
 │   gyro  −= bias      [implemented]          │
 │   accel −= offset    [G3 — missing]         │
 └──────┬──────────────────────────────────────┘
        v
 ┌─────────────────────────────────────────────┐   ImuData{ax..az, gx..gz,
 │ ImuData  ── crosses into core/ ─────────────│           temp, timestamp, valid}
 └──────┬──────────────────────────────────────┘
        v
   StateEstimator::update()  →  BalancingController::update()  →  torque
```

### Timing discipline

Never sleep a fixed interval and call it a control period — the work in the loop is not
free, so fixed sleeps accumulate drift. Compute an absolute deadline and sleep to it.
The host loop already does this (`cube_balancer.cpp:303-313`) and the structure carries
over; only the sleep call changes.

**`micros()` wraps every ~71 minutes** on Teensy (`uint32_t`). Naive conversion makes
`dt` go hugely negative once an hour and the filter explodes. Accumulate a 64-bit
microsecond counter.

### State machine

```
   UNINITIALIZED ──init()──► IDENTIFIED ──blob──► CONFIGURED
                     │ fail                 │ fail
                     v                      v
                   FAULT ◄───────────────── FAULT
                                              ▲
   CONFIGURED ──calibrate()──► READY ──run──► STREAMING ──bus error──┘
```

`FAULT` must be **terminal for the control loop** — on IMU loss the only safe action is
to stop the wheel, not to keep integrating stale data.

---

## Appendix — datasheet cross-reference

| Constant | Value | In code |
|---|---|---|
| `CHIP_ID` | `0x24` @ 0x00 | `kChipIdBmi270` |
| Soft reset | `CMD`=`0xB6` @ 0x7E, 2 ms | `kCmdSoftReset` |
| `INTERNAL_STATUS` ready | `0b0001` @ 0x21 | `kInternalStatusReady` |
| Data burst | 12 B @ 0x0C | `kRegAccXLsb` |
| Temperature | `/512 + 23 °C`; `0x8000` invalid | `ImuDriver.cpp:290-296` |
| SPI max clock | 10 MHz | — |
| SPI mode | 0 (or 3), auto-selected | — |
| SPI read | `reg\|0x80` + dummy byte | — |
| Supply | VDD 1.71–3.6 V | — |
| Gyro noise | <7 mdps/√Hz | `GYR_CONF=0xE0` |
| CRT residual | typ. 0.4% | unused (G5) |
| FIFO | 2 KB | unused (G1) |
| ODR | `0x0A` = 400 Hz | `Config::odr` |
