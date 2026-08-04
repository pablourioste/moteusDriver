# BMI270 Integration Blueprint

**Status:** technical blueprint only — no source files are modified by this document.
**Datasheet of record:** `docs/IMU_datasheet.pdf` — Bosch BMI270, rev 1.5, March 2023, BST-BMI270-DS000-07.
**Date:** 2026-08-04

---

## 0. Scope Correction — Read This First

The brief that produced this document specified a **Teensy 4.0/4.1 (ARM Cortex-M7)**
target, a PlatformIO/Arduino layout, and left the IMU model and protocol as
placeholders. Two of those premises do not match this repository, and building
to them would produce a blueprint for a machine that does not exist here.

| Brief assumed | Repository reality | Evidence |
|---|---|---|
| Teensy 4.x, Cortex-M7 @ 600 MHz | **Linux host, x86-64 (WSL2 now, SBC on the rig)** | `grep -riE "teensy\|arduino\|platformio\|cortex-m7\|micros\(\)\|digitalWrite"` over all sources returns **zero hits** |
| Arduino `Wire` / `SPI` library | **Linux i2c-dev `ioctl`** | `src/drivers/ImuDriver.cpp:11-13` — `<linux/i2c-dev.h>`, `<sys/ioctl.h>`, `<unistd.h>` |
| `src/main.cpp` with `micros()` loop | **`apps/*.cpp`, one per executable**, CMake targets | `CMakeLists.txt` targets `imu_test`, `cube_balancer`, `direct_motor_test` |
| IMU model `[INSERT]` | **Resolved: BMI270** | `docs/IMU_datasheet.pdf` p.1; `ImuDriver::name()` returns `"BMI270"` |
| Protocol `[INSERT]` | **Resolved: I2C** — and effectively forced, see §3 | `ioctl(fd_, I2C_SLAVE, ...)`, `ImuDriver.cpp:195` |
| "Refactor toward modular OOP" | **Already done** — `IImuSensor` interface, `core/` ↔ `drivers/` split enforced at link time | `include/core/interfaces/IImuSensor.hpp`; `cube_core` has no driver sources |

**What this document therefore is:** a BMI270-grounded gap analysis and roadmap for
*this* Linux/CAN reaction-wheel cube, targeting the work that is genuinely
outstanding. Sections 1–7 follow the requested structure, but the content is
anchored to the real code and the real datasheet.

**Where the Teensy framing still earns its keep:** the non-blocking-loop
discipline, the `WHO_AM_I`-first bring-up protocol, and the
read → convert → correct → fuse → log pipeline are all architecture-level
concerns that transfer intact. They are kept, retargeted to POSIX.

> If a Teensy is genuinely planned as a future sensor front-end (MCU samples the
> BMI270 at high rate, streams to the Linux host over serial/CAN), say so — that is
> a real and defensible architecture, and §4 notes the one seam where it would attach.
> Nothing in this repo currently anticipates it.

---

## 1. Gap Analysis: Current Repository vs. Target Architecture

### 1.1 Current State Audit

The IMU path is **implemented and structurally sound** — not a legacy mess awaiting
rescue. Auditing it honestly:

| Concern | Current handling | Verdict |
|---|---|---|
| Bus access | `writeRegister()` / `readRegisters()`, raw `::read`/`::write` on the i2c-dev fd | Correct. Register-write-then-read, no combined ioctl — `ImuDriver.cpp:85-89` documents why |
| Chip identification | `CHIP_ID` (0x00) read, compared against `0x24`, fails loudly with the observed byte | Correct; matches datasheet §5.2.1 (`RESET: 0x24`) |
| Config blob | Full 8 KB upload: `PWR_CONF=0` → 450 µs → `INIT_CTRL=0` → chunked `INIT_DATA` → `INIT_CTRL=1` → poll `INTERNAL_STATUS` | Correct and complete, incl. the nibble-split `INIT_ADDR_0/1` half-word addressing |
| Data read | **One 12-byte burst from 0x0C** (accel + gyro contiguous) | Correct — avoids straddling an internal update. This is the single most important thing to get right and it is right |
| Unit conversion | Scale factors precomputed at init; hot path is a multiply | Correct; `accel_scale_`, `gyro_scale_` set in `initialize()` |
| Bias correction | Subtracted inline in `read()`, from `Config::gyro_bias_*` | Correct placement |
| Calibration | `calibrateGyroBias()` — mean + variance over N samples, clears prior bias first | Sound; see §6 for the gaps |
| Interface abstraction | `IImuSensor` pure-virtual; `core/` cannot include `drivers/` | Already the target architecture |
| Timing loop | `SleepMicroseconds(2500)` open-loop in the calibration path | **Gap** — see §1.3 |

**The dominant risk in this module is not code quality.** It is that
`HAVE_BMI270_CONFIG_BLOB` may be undefined, in which case `initialize()` refuses to
run at all (`ImuDriver.cpp:92-101`). That refusal is *correct behaviour* — the
alternative is an IMU that returns zeros while the chip ID reads back perfectly.

### 1.2 Datasheet Insights Breakdown

Constraints extracted from `docs/IMU_datasheet.pdf` that bear directly on this design:

| # | Constraint | Datasheet locus | Consequence here |
|---|---|---|---|
| D1 | `CHIP_ID` = **0x24** at 0x00 | §5.2.1 | Already asserted. Any other value ⇒ wrong part or wrong address |
| D2 | I2C address **0x68** (SDO→GND) / **0x69** (SDO→VDDIO) | §6.2, Table 17 pin 1 | `kI2cAddrPrimary/Secondary` correct. **SDO must never float** |
| D3 | I2C supports 100 k / 400 k / **1 MHz (Fm+)** | §6.3 | Headroom exists; see §3 throughput math |
| D4 | `NV_CONF.i2c_wdt_sel=0` ⇒ SCL **must exceed 100 kHz** or the watchdog may trip | datasheet line ~12456 | Never clock the bus below 100 kHz. Rules out bit-banged slow I2C |
| D5 | VDD 1.71–3.6 V, **VDDIO independent** 1.2–3.6 V | §1 key features, §2 | 3.3 V rail. Logic pins abs-max `VDDIO+0.3 V, <4 V` — **5 V logic destroys the part** |
| D6 | Sensors stay **disabled** until config blob uploaded; `INTERNAL_STATUS` must read `0b0001` | §4.x init sequence | Implemented. Non-negotiable |
| D7 | After power-on the part latches bus type on first access; **dummy read** puts it in I2C mode | §6.2 / init notes | Implemented, `ImuDriver.cpp:206-208` |
| D8 | Soft reset `CMD=0xB6`, then **2 ms** settle | §5.2.x CMD | Implemented |
| D9 | Gyro ODR 25 Hz–6.4 kHz; accel 0.78 Hz–1.6 kHz | §1 | Config uses `odr=0x0A` (400 Hz) — above the 200 Hz loop. Sound |
| D10 | Gyro noise **< 7 mdps/√Hz** in performance mode | §1 | `GYR_CONF=0xE0\|odr` already selects performance + filter-perf |
| D11 | Gyro offset comp. LSB **61 mdps**, range **±31 dps**, range-independent | line ~4930 | Bounds any on-chip offset scheme (§6) |
| D12 | **CRT** — gyro sensitivity compensation, residual error typ. **0.4 %** | §1 | **Unused.** Real accuracy left on the table (§6.4) |
| D13 | **2 KB FIFO**, accel+gyro+timestamp+AUX | §1 | **Unused.** Relevant to jitter (§7.3) |
| D14 | **Sensortime** stamps, host↔sensor sync < 40 µs | §1 | **Unused.** Currently timestamping with host `CLOCK_MONOTONIC` at read time |
| D15 | 2 programmable INT pins (INT1/INT2, pins 4/9) | §6.2 Table 17 | **Unused** — driver polls. See §1.3/§7 |
| D16 | Temperature: 0x8000 = invalid; scale `/512 + 23 °C` | §5.2.x | Implemented incl. the invalid sentinel |

### 1.3 Required Structural Shifts

Ranked by real payoff. **None of these are refactors for their own sake.**

| # | Shift | Why | Effort |
|---|---|---|---|
| **S1** | **Data-ready timing: poll → INT1 or FIFO** (D13/D15) | `read()` is called on the host's schedule, not the sensor's. Every sample carries 0–2.5 ms of unknown age, and the host timestamp (`MonotonicSeconds()` *after* the transfer) attributes it to the wrong instant. A complementary/LQR loop differentiating that jitter sees it as phantom rate | M |
| **S2** | **Timestamp at the sensor, not the host** (D14) | `data.timestamp` is set after the I2C burst completes, so it includes bus latency. Sensortime, or at minimum stamping *before* the transfer, tightens the `dt` that the estimator integrates | S |
| **S3** | **Persist calibration across runs** | `Config::gyro_bias_*` lives in memory only. Every run re-calibrates or runs biased. A drifting gyro bias integrates into unbounded tilt error — the classic silent balancer failure | S |
| **S4** | **Accelerometer calibration entirely absent** | Only gyro bias is handled. Accel zero-g offset and axis misalignment feed straight into `accelAngle()`, which is the estimator's only absolute attitude reference. A 0.5° mounting tilt becomes a 0.5° permanent setpoint error the balancer fights forever | M |
| **S5** | **Adopt CRT** (D12) | Gyro scale error is unaddressed. 0.4 % residual vs. uncalibrated is the difference between a slow drift and a fast one | M |
| **S6** | **Health/liveness on the read path** | `read()` returns `valid=false` on bus failure, but a stuck-but-responding bus (all-zero data) is indistinguishable from "perfectly level and still". Add a staleness/all-zeros check | S |
| **S7** | Split register map out of `ImuDriver.hpp` into a config header | Cosmetic *here* — the constants are already `static constexpr` in one place. Listed for completeness against the requested §4; **low priority** | S |

**Explicitly not needed:** the OOP/interface refactor the brief asked for. `IImuSensor`,
the `core`/`drivers` split, and link-time layering enforcement already exist and are
better than the flat PlatformIO layout proposed.

---

## 2. Project Roadmap & Sequential Execution Strategy

```
 ┌──────────────────────────────────────────────────────────────────────┐
 │ PHASE 0 — GATE: is the config blob compiled in?                      │
 │   cmake -LA build | grep BMI270_CONFIG_HEADER   → must be ON         │
 │   Without it initialize() refuses. Everything below is blocked.      │
 └───────────────────────────────┬──────────────────────────────────────┘
                                 v
 ┌──────────────────────────────────────────────────────────────────────┐
 │ PHASE 1 — ELECTRICAL + WHO_AM_I         ** START HERE **             │
 │   3.3 V verified → i2cdetect shows 0x68 → CHIP_ID reads 0x24         │
 │   Deliverable: ./build/imu_test reaches "CHIP_ID ok"                 │
 └───────────────────────────────┬──────────────────────────────────────┘
                                 v
 ┌──────────────────────────────────────────────────────────────────────┐
 │ PHASE 2 — BLOB UPLOAD + LIVE DATA                                    │
 │   INTERNAL_STATUS == 0b0001 → burst 0x0C → gravity ≈ 9.81 on one axis│
 │   Gate: |‖a‖ − 9.81| < 0.3 m/s² at rest                              │
 └───────────────────────────────┬──────────────────────────────────────┘
                                 v
 ┌──────────────────────────────────────────────────────────────────────┐
 │ PHASE 3 — CALIBRATION  (S3, S4, S5)                                  │
 │   gyro bias → persist to file → accel 6-position → optional CRT      │
 │   Gate: bias-corrected gyro |ω| < 0.002 rad/s at rest, run-to-run    │
 └───────────────────────────────┬──────────────────────────────────────┘
                                 v
 ┌──────────────────────────────────────────────────────────────────────┐
 │ PHASE 4 — DETERMINISTIC TIMING  (S1, S2, S6)                         │
 │   INT1 data-ready or FIFO; timestamp before transfer; staleness check│
 │   Gate: dt jitter < 10 % of nominal over 10 000 samples              │
 └───────────────────────────────┬──────────────────────────────────────┘
                                 v
 ┌──────────────────────────────────────────────────────────────────────┐
 │ PHASE 5 — FUSION           ← StateEstimator::update(), accelAngle()  │
 │   USER-OWNED. Scaffolded with an 8-step derivation. Do not pre-empt. │
 └───────────────────────────────┬──────────────────────────────────────┘
                                 v
 ┌──────────────────────────────────────────────────────────────────────┐
 │ PHASE 6 — CONTROL          ← BalancingController::update()           │
 │   USER-OWNED. Blocked on physical-parameter measurement + LQR gains. │
 └──────────────────────────────────────────────────────────────────────┘
```

> **Phases 5 and 6 are deliberately left to the user.** Per `CLAUDE.md`, the
> estimator and controller are being written module by module to retain
> authorship of the control law. This blueprint stops at their boundary.

### Immediate first task — Phase 1

1. **Power off.** Confirm the BMI270 breakout's supply pin is on **3.3 V**, not 5 V (D5).
2. Confirm **SDO is strapped**, not floating — GND for 0x68, VDDIO for 0x69 (D2).
3. Power on, then: `i2cdetect -y 1` — expect `68` (or `69`).
4. `./build/imu_test` — the first hard gate is `CHIP_ID == 0x24` (D1).
5. Only once that passes, proceed to blob upload.

If step 3 shows nothing, the fault is electrical (§3.2/§5) — **do not** start editing
driver code.

---

## 3. Physical Connections & Protocol Analysis

### 3.1 Protocol Deep-Dive — SPI vs. I2C

The datasheet supports both (§6.2): I2C up to 1 MHz Fm+, SPI 4-wire/3-wire up to 10 MHz.

| Criterion | I2C (current) | SPI | Bearing on this rig |
|---|---|---|---|
| Max clock | 1 MHz (Fm+, D3) | 10 MHz | Both ample |
| Bytes per sample | 12 (burst 0x0C) + 2 temp | same | — |
| Wire time @ 400 kHz | ~350 µs incl. addressing/ACK | ~15 µs @ 10 MHz | I2C costs ~0.35 ms of a 5 ms budget |
| Wire time @ 1 MHz | ~140 µs | — | Comfortable |
| Pins | 2 (SDA/SCL), shared bus | 4 (SCK/SDI/SDO/CSB), + 1 CS per device | I2C wins on wiring |
| Pull-ups | **Required** externally | None | See §3.2 |
| Quirk | Watchdog needs SCL > 100 kHz (D4) | First read after power-on returns garbage — must dummy-read `CHIP_ID` (datasheet line ~11483) | Each has one trap |
| **Linux support** | **`/dev/i2c-*`, trivial** | `spidev`, also fine | — |

**Recommendation: stay on I2C.** At 400 kHz the transfer is ~7 % of a 200 Hz cycle;
at 1 MHz, ~3 %. The bottleneck in this system is **sample-timing jitter (S1), not bus
bandwidth**, and switching to SPI would not fix that — an interrupt-driven or
FIFO-backed read on I2C would. Reserve SPI for a future move to ≥1 kHz sampling.

> **If SCL is currently at 100 kHz**, raising it to 400 kHz is the single cheapest
> latency win available and carries no risk (D3/D4).

### 3.2 Electrical & Voltage Warnings

```
  ⚠  ABSOLUTE MAXIMUM (datasheet Table 5): logic pin ≤ VDDIO + 0.3 V, and < 4 V.
     With VDDIO = 3.3 V, any 5 V signal on SDA/SCL/SDO/CSB/INT DESTROYS the part.
     There is no 5 V tolerance on any BMI270 pin.
```

- **Supply (D5):** VDD 1.71–3.6 V, VDDIO 1.2–3.6 V, independent. Use 3.3 V for both.
  Most breakouts tie them together and add a regulator — if your breakout accepts 5 V
  on its `VIN`, that is the *breakout's* regulator, not the BMI270's.
- **Pull-ups:** I2C is open-drain and needs external pull-ups to **VDDIO (3.3 V)**.
  Use **2.2 kΩ–4.7 kΩ**; 4.7 kΩ is right for short leads at 400 kHz, drop to 2.2 kΩ
  for 1 MHz Fm+ or longer leads. Most breakouts already fit these — **check before
  adding more in parallel**, since doubled pull-ups can exceed the sink current spec.
- **Pull-ups must go to VDDIO**, never to 5 V. This is the most common way to kill a
  3.3 V IMU on an otherwise correct board.
- **SDO must be strapped** (D2). Floating SDO ⇒ a device that appears intermittently at
  0x68 or 0x69, presenting as "flaky wiring".
- **CSB:** in I2C mode tie **CSB → VDDIO**. Leaving it floating or low can latch the
  part into SPI mode at power-on (D7), after which the I2C address never responds.
- **Leads:** keep SDA/SCL short. I2C bus capacitance ≤ 400 pF; long dupont wiring at
  1 MHz is the usual cause of intermittent NACKs.

### 3.3 Pinout Mapping Matrix

**I2C — the wiring in use.** Host pin names given for a Raspberry Pi-class SBC, which
is what `/dev/i2c-1` implies.

```
        BMI270 breakout                        Linux SBC (e.g. RPi 40-pin)
     ┌────────────────────┐                  ┌────────────────────────────┐
     │ VDD / VIN      ────┼──────────────────┤ 3V3        (pin 1 or 17)   │
     │ GND            ────┼──────────────────┤ GND        (pin 6, 9, …)   │
     │ SDA            ────┼───────┬──────────┤ GPIO2 / SDA1   (pin 3)     │
     │ SCL            ────┼────┬──┼──────────┤ GPIO3 / SCL1   (pin 5)     │
     │ SDO            ────┼────┼──┼───┐      │                            │
     │ CSB            ────┼────┼──┼───┼──┐   │                            │
     │ INT1           ────┼────┼──┼───┼──┼───┤ GPIO4      (pin 7) [S1]    │
     └────────────────────┘    │  │   │  │   └────────────────────────────┘
                               │  │   │  │
                          Rp   │  │   │  └──→ 3V3   (CSB high = I2C mode)
                       2.2–4.7k│  │   └─────→ GND   (SDO low  = addr 0x68)
                               │  │                  3V3      (        0x69)
                          3V3 ─┴──┘   ← both pull-ups to 3V3, NEVER 5V
```

| BMI270 pin | Function (I2C) | Connect to | Required? | Note |
|---|---|---|---|---|
| VDD | Analog supply | 3V3 | Yes | 1.71–3.6 V (D5) |
| VDDIO | Interface supply | 3V3 | Yes | Often bonded to VDD on breakouts |
| GND | Ground | GND | Yes | Common ground with the SBC |
| SDA | Data | SDA1 | Yes | Pull-up 2.2–4.7 kΩ → 3V3 |
| SCL | Clock | SCL1 | Yes | Pull-up 2.2–4.7 kΩ → 3V3; **> 100 kHz** (D4) |
| SDO | Address select | GND **or** 3V3 | Yes | GND ⇒ 0x68. Must not float (D2) |
| CSB | Chip select | **3V3** | Yes | High selects I2C (D7) |
| INT1 | Data-ready IRQ | GPIO (any) | *Phase 4* | Enables S1; unused today (D15) |
| INT2 | Second IRQ | — | No | Leave open |

**SPI — reference only, not the current wiring.** Should the project ever move:

| BMI270 pin | SPI 4-wire | Note |
|---|---|---|
| SCK (SCx) | SPI clock | ≤ 10 MHz |
| SDI | MOSI | |
| SDO | MISO | No longer an address strap in SPI mode |
| CSB | Chip select | Active low; **drives mode selection at power-on** |

> **SPI trap (datasheet ~line 11483):** after power-up, perform a **single dummy read
> of `CHIP_ID`** — its value will be invalid — before any real access. The I2C path has
> the mirror-image requirement (D7), already implemented.

---

## 4. Modular Repository Target Architecture

The requested PlatformIO/Arduino layout does not apply (§0). The equivalent
separation of concerns **already exists** in the CMake tree. Mapping the brief's
intent onto reality:

| Brief's file | This repo's equivalent | State |
|---|---|---|
| `include/Config.h` | `ImuDriver::Config` + register constants in `ImuDriver.hpp`; bus defaults as CMake cache vars (`IMU_I2C_DEVICE`, `IMU_I2C_ADDRESS`) | **Exists.** Optional split → `include/drivers/Bmi270Registers.hpp` (S7, low priority) |
| `include/IMUDriver.h` / `.cpp` | `include/drivers/ImuDriver.hpp`, `src/drivers/ImuDriver.cpp` | **Exists, complete** |
| `include/IMUCalibrator.h` / `.cpp` | *(none — `calibrateGyroBias()` lives inside the driver)* | **Gap (S3/S4).** The one genuinely missing module |
| `include/OrientationFilter.h` / `.cpp` | `include/core/StateEstimator.hpp`, `src/core/StateEstimator.cpp` | **Scaffold — user-owned** |
| `src/main.cpp` | `apps/imu_test.cpp`, `apps/cube_balancer.cpp` | **Exists**, one entry point per executable |

### 4.1 Target tree (deltas marked)

```
include/
  core/                          PLATFORM-AGNOSTIC — must not include drivers/
    Types.hpp                    ImuData, MotorState, BodyState
    StateEstimator.hpp           ← scaffold, user-owned (Phase 5)
    BalancingController.hpp      ← scaffold, user-owned (Phase 6)
    interfaces/
      IImuSensor.hpp             initialize / read / calibrateGyroBias / name
      IMotorDriver.hpp
  drivers/                       HARDWARE
    ImuDriver.hpp                BMI270 over Linux I2C
 +  Bmi270Registers.hpp          [S7, optional] register map extracted verbatim
    MoteusDriverWrapper.hpp
    MoteusConfig.hpp
 +calibration/                   [S3/S4] NEW — the real structural gap
 +  ImuCalibration.hpp           struct: gyro bias, accel offset, accel scale
 +  ImuCalibrator.hpp            6-position accel routine + gyro bias
src/
  core/ drivers/ testing/        mirror include/
 +src/calibration/ImuCalibrator.cpp
apps/
  imu_test.cpp                   validation + calibration entry point
  cube_balancer.cpp              200 Hz loop
 +data/calibration/imu_cal.cfg   [S3] persisted offsets, loaded at startup
```

### 4.2 Layering contract

```
   apps/  ──────────────┬──────────────────────┐
                        v                      v
              cube_drivers (hardware)    cube_core (pure)
                 ImuDriver ──────────► IImuSensor ◄──── StateEstimator
                 MoteusDriverWrapper                    BalancingController
                        │
                        └──► moteus_lib (header-only), linux/i2c-dev
```

**Rule:** nothing in `core/` may include from `drivers/`. Enforced at **link time** —
`cube_core` is built without driver sources, so a violation is a build failure, not a
review comment. `ImuCalibration` (the plain data struct) belongs on the `core` side;
`ImuCalibrator` (which drives hardware) belongs on the `drivers` side.

> **Teensy seam, if that path is ever taken:** a Teensy front-end would be a *new
> implementation of `IImuSensor`* (e.g. `SerialImuBridge`) reading framed packets from
> `/dev/ttyACM*`. `core/` would not change by one line. That is the payoff of the
> existing interface split, and it is why the flat PlatformIO layout would be a
> regression here.

---

## 5. Diagnostic & Communication Testing Protocol

Escalating ladder — **each rung must pass before the next.** Most "driver bugs" at this
stage are wiring faults, and the ladder is built to prove that before any code changes.

### Rung 0 — Bus exists
```bash
ls /dev/i2c-*                 # nothing? enable I2C on the SBC
```
> On **WSL2 there is no I2C bus at all**; the driver says so explicitly
> (`ImuDriver.cpp:189-192`). The IMU path only runs on the target SBC.

### Rung 1 — Device acknowledges (pure electrical)
```bash
i2cdetect -y 1                # expect 68 (SDO low) or 69 (SDO high)
```
| Symptom | Diagnosis |
|---|---|
| Nothing at all | No power, no pull-ups, or SDA/SCL swapped |
| Address flickers 68↔69 | **SDO floating** (D2) |
| Every address 00–7F responds | SDA stuck low — shorted, or a device holding the bus |
| 68 appears, then vanishes | CSB floating → part latched into SPI mode (D7). Power-cycle with CSB → 3V3 |

### Rung 2 — `WHO_AM_I` / `CHIP_ID`
```bash
i2cget -y 1 0x68 0x00         # expect 0x24
```
**The logic, and why this register specifically:** `CHIP_ID` is at address 0x00, is
read-only, has a fixed reset value of `0x24` (D1), and requires no prior
configuration. It exercises the complete round trip — START, address+W, register
byte, repeated START, address+R, data byte, NACK, STOP — while depending on nothing.
A correct `0x24` proves addressing, both bus directions, pull-ups, and logic levels
in one transaction.

**Bus restart:** the driver writes the register pointer, then reads
(`ImuDriver.cpp:87-88`) — a stop-start rather than a repeated start. The comment
records that the BMI270 treats them identically at these rates. Keep it; combined
`I2C_RDWR` messages are only needed for parts that demand a true repeated start.

**Bit ranges to check when the value is wrong:**

| Read | Meaning |
|---|---|
| `0x24` | ✅ BMI270 confirmed |
| `0xFF` | Nothing driving the bus — pull-ups present but no device responding |
| `0x00` | Bus held low; SDA short |
| `0x1B`/`0x2B`/other | A *different* Bosch part (BMI160/BMI088 etc.) — wrong module |
| Unstable | Marginal pull-ups, clock too fast for the wiring, or a floating strap |

### Rung 3 — Post-reset identity
Soft-reset (`CMD=0xB6`), wait 2 ms (D8), re-read `CHIP_ID`. Still `0x24` ⇒ the part
survives a reset cycle and the dummy-read I2C latch (D7) is working.

### Rung 4 — Config blob accepted
`INTERNAL_STATUS` (0x21) low nibble must read `0b0001` (D6). The driver polls this for
up to 40 ms. Failure here with a good `CHIP_ID` means **the blob is wrong or truncated**,
not that the wiring is bad.

### Rung 5 — Physical plausibility
Rest the unit on a flat surface:
- `‖a‖` ≈ **9.81 m/s²**, concentrated on one axis (±0.3)
- `‖ω‖` ≈ **0** (< 0.02 rad/s pre-calibration)
- temperature within ~10 °C of ambient — and **not** the 0x8000 invalid sentinel (D16)

Rotate 90°: gravity must migrate cleanly to another axis with the expected sign.
This is what catches a byte-order or axis-mapping error that every prior rung passes.

### Rung 6 — Sustained integrity
```bash
./build/imu_test --stream
```
Watch for dropped reads (`valid=false`), all-zero samples (S6), and `dt` jitter.

---

## 6. Calibration Methodology & Mathematical Baseline

### 6.1 Gyroscope zero-rate bias — implemented

Currently in `ImuDriver::calibrateGyroBias()`. The device is held still; the mean of
each axis is the bias:

```
        1  N-1                          1  N-1
b_g  =  ─  Σ   ω_raw[k]        σ²  =    ─  Σ  (ω_raw[k] − b_g)²
        N k=0                           N k=0
```

Applied per-sample in `read()`: `ω = ω_raw · s_g − b_g`.

Two details already correct and worth preserving:
- **Prior bias is cleared before measuring** — otherwise it measures the residual of
  the previous correction, not the true offset.
- **Variance is accumulated alongside the mean** — this is the motion detector. High
  σ means the unit moved during calibration and the result must be rejected.

**Gate:** σ per axis below the datasheet noise floor scaled to the sample count
(D10: < 7 mdps/√Hz). If σ is large, the mean is contaminated — re-run.

### 6.2 Gap S3 — persistence

The bias lives in `Config` and dies with the process. Target:

```
data/calibration/imu_cal.cfg     # plain key value, same style as current_config.cfg
  gyro_bias_x   -0.001834
  gyro_bias_y    0.000417
  gyro_bias_z    0.002201
  accel_offset_x ...
  temperature_c  24.6           # bias is temperature-dependent — record the conditions
  timestamp      2026-08-04T09:12:00Z
```

Load at startup; **warn if absent** rather than silently running uncalibrated —
silent zero-bias is exactly the failure mode that produces slow, mysterious drift.
Recording the calibration temperature matters because gyro bias moves with it; a
calibration taken cold and used warm is a known drift source.

### 6.3 Gap S4 — accelerometer calibration

Unaddressed today, and it feeds the estimator's **only absolute attitude reference**.
Model, per axis:

```
        a_corrected = (a_raw − o) / s          o = offset (m/s²), s = scale (≈1)
```

**Six-position method.** Rest each axis ±1 g against gravity:

```
        a_max + a_min                    a_max − a_min
   o =  ─────────────            s  =    ─────────────
              2                              2 · 9.80665
```

| Position | Expected (m/s²) |
|---|---|
| +X up | `+9.81, 0, 0` |
| −X up | `−9.81, 0, 0` |
| +Y up | `0, +9.81, 0` |
| −Y up | `0, −9.81, 0` |
| +Z up | `0, 0, +9.81` |
| −Z up | `0, 0, −9.81` |

Average ≥ 2 s per position at rest. **Gate:** post-correction `|‖a‖ − 9.81| < 0.05 m/s²`
in all six orientations.

> **Mounting misalignment matters more than offset here.** A cube whose IMU is
> mounted 0.5° off the body frame has a 0.5° error in its notion of vertical, which the
> balancer will faithfully hold — leaning permanently. Capture this as a fixed
> body-frame rotation determined at assembly, and apply it after offset/scale
> correction. Do not attempt to absorb it into the accel offsets; it is a rotation,
> not a translation, and folding the two together corrupts both.

### 6.4 Gap S5 — CRT (optional, Phase 3 tail)

D12: on-chip gyro **sensitivity** compensation, residual typ. 0.4 %. §6.1 corrects
*offset* only — scale error remains and shows up as tilt that accumulates faster
during fast rotation than slow. Worth doing once §6.1–6.3 are solid and persisted;
not before.

### 6.5 Runtime application order

Order is not arbitrary — offset must precede scale, and both must precede rotation:

```
raw int16 ──► ×scale_factor ──► −offset ──► ×axis_correction ──► body-frame rotation
             (LSB → SI)        (bias)      (sensitivity)        (mounting)
```

---

## 7. Subprogram Modularization & Pipeline Flow

### 7.1 Data flow

```
 ┌──────────────┐
 │  BMI270      │  16-bit signed, little-endian, ODR 400 Hz
 └──────┬───────┘
        │  [Phase 4: INT1 data-ready asserts here — S1]
        v
 ┌─────────────────────────────────────────────┐
 │ ImuDriver::readRegisters(0x0C, buf, 12)     │  ONE burst: accel+gyro
 │   → ToInt16() ×6                            │  coherent sample pair
 └──────┬──────────────────────────────────────┘
        v
 ┌─────────────────────────────────────────────┐
 │ UNIT CONVERSION   (precomputed at init)     │
 │   accel_scale_ = range_g · 9.80665 / 32768  │  → m/s²
 │   gyro_scale_  = range_dps · π/180 / 32768  │  → rad/s
 └──────┬──────────────────────────────────────┘
        v
 ┌─────────────────────────────────────────────┐
 │ OFFSET CORRECTION                           │
 │   gyro  −= bias      [implemented]          │
 │   accel −= offset    [S4 — missing]         │
 └──────┬──────────────────────────────────────┘
        v
 ┌─────────────────────────────────────────────┐   ImuData{ax,ay,az,gx,gy,gz,
 │ ImuData  ── crosses the core/drivers seam ──│           temp,timestamp,valid}
 └──────┬──────────────────────────────────────┘
        v
 ┌─────────────────────────────────────────────┐
 │ StateEstimator::update()   ← Phase 5, USER  │  complementary filter
 │   accelAngle() ⊕ ∫gyro·dt                   │
 └──────┬──────────────────────────────────────┘
        v
 ┌─────────────────────────────────────────────┐
 │ BodyState → BalancingController::update()   │  ← Phase 6, USER
 │            → torque → MoteusDriverWrapper   │
 └──────┬──────────────────────────────────────┘
        v
 ┌─────────────────────────────────────────────┐
 │ TELEMETRY  — CsvLogger  [gap: takes a       │
 │   moteus::Query::Result, not BodyState]     │
 └─────────────────────────────────────────────┘
```

### 7.2 Non-blocking loop discipline

The Arduino `delay()` prohibition maps onto POSIX as: **never sleep for a fixed
interval and call it a control period.** The cost of the work in the loop is not zero,
so fixed sleeps accumulate drift. Compute an absolute deadline and sleep until it.

```cpp
// Target pattern — absolute deadlines, no accumulating drift.
struct timespec next;
clock_gettime(CLOCK_MONOTONIC, &next);
constexpr long kPeriodNs = 5'000'000;          // 200 Hz

while (running) {
  const ImuData imu = imu_.read();             // ~350 µs @ 400 kHz
  estimator_.update(imu, motor_state, dt);     // measured dt, not nominal
  motor_.sendTorque(controller_.update(...));

  next.tv_nsec += kPeriodNs;
  if (next.tv_nsec >= 1'000'000'000) { next.tv_nsec -= 1'000'000'000; next.tv_sec++; }
  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
}
```

Three rules:
- **`clock_nanosleep(TIMER_ABSTIME)`**, not `usleep(period)`. The latter drifts by
  exactly the loop's execution time, every cycle.
- **Pass measured `dt`** into the estimator, never the nominal period. If a cycle
  overruns, integrating with the nominal value silently under-integrates.
- **Detect overrun.** If the deadline has already passed, that cycle is late — count
  it and surface it. Silent overruns present as inexplicable control instability.

> **Anti-pattern already present:** `calibrateGyroBias()` uses
> `SleepMicroseconds(2500)` open-loop (`ImuDriver.cpp:333`). Harmless there — it only
> affects how many samples land in a fixed wall-clock window — but it must not be
> copied into the control loop.

### 7.3 State machine

```
   UNINITIALIZED ──initialize()──► IDENTIFIED ──blob──► CONFIGURED
                       │ fail                    │ fail
                       v                         v
                     FAULT ◄────────────────── FAULT
                                                 ▲
   CONFIGURED ──calibrate()──► READY ──run──► STREAMING ──bus error──┘
                                                 │
                                                 └── all-zeros / stale [S6] ──► FAULT
```

`FAULT` must be **terminal for the control loop** — on IMU loss the balancer's only
safe action is to stop the wheel, not to keep integrating stale data. This is the same
discipline as `SetStop()` on SIGINT: the failure path must actively disarm, never
coast.

### 7.4 FIFO alternative (D13)

The 2 KB FIFO lets the host read several samples per wake-up, each with a Sensortime
stamp (D14). This decouples loop rate from sample rate — the estimator gets evenly
spaced samples even when the host wakes unevenly. Heavier to implement than INT1
data-ready; worth it only if Phase 4 shows the jitter is dominated by host scheduling
rather than by polling.

---

## Appendix A — Verification Commands

```bash
# Phase 0 — blob gate
cmake -LA build | grep BMI270_CONFIG_HEADER          # must be ON

# Phase 1 — electrical + identity            (SBC only; no I2C on WSL2)
ls /dev/i2c-*
i2cdetect -y 1                                       # expect 68 or 69
i2cget -y 1 0x68 0x00                                # expect 0x24

# Phase 2 — blob + live data
i2cget -y 1 0x68 0x21                                # INTERNAL_STATUS, low nibble 0b0001
./build/imu_test

# Phase 3 — calibration
./build/imu_test                                     # runs gyro bias routine
./build/imu_test --stream                            # verify residual at rest

# Bus speed (Raspberry Pi) — 400 kHz; > 100 kHz mandatory per D4
#   /boot/firmware/config.txt:  dtparam=i2c_arm=on,i2c_arm_baudrate=400000
```

## Appendix B — Datasheet Cross-Reference

| Constant | Value | Source | In code |
|---|---|---|---|
| `CHIP_ID` | `0x24` @ 0x00 | §5.2.1 | `kChipIdBmi270` |
| Soft reset | `CMD=0xB6` @ 0x7E, 2 ms | §5.2.x | `kCmdSoftReset` |
| `INTERNAL_STATUS` ready | `0b0001` @ 0x21 | §4.x | `kInternalStatusReady` |
| Data burst | 12 B @ 0x0C | §5.2.x | `kRegAccXLsb` |
| Temperature | `/512 + 23 °C`; `0x8000` invalid | §5.2.x | `ImuDriver.cpp:290-296` |
| I2C address | `0x68` / `0x69` via SDO | §6.2 T17 | `kI2cAddrPrimary/Secondary` |
| I2C max clock | 1 MHz (Fm+) | §6.3 | — (bus config) |
| I2C min clock | > 100 kHz (watchdog) | ~12456 | — |
| VDD / VDDIO | 1.71–3.6 / 1.2–3.6 V | §1, §2 | — |
| Logic abs-max | VDDIO+0.3 V, < 4 V | T5 | — |
| Gyro noise | < 7 mdps/√Hz | §1 | `GYR_CONF=0xE0` |
| Gyro offset comp. | 61 mdps LSB, ±31 dps | ~4930 | unused (D11) |
| CRT residual | typ. 0.4 % | §1 | unused (D12) |
| FIFO | 2 KB | §1 | unused (D13) |
| Sensortime sync | < 40 µs | §1 | unused (D14) |
| ODR (accel/gyro) | 0.78–1600 / 25–6400 Hz | §1 | `odr=0x0A` (400 Hz) |
