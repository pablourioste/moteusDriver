# Integration Guide — Teensy 4.1 Port

How to build the system, in order, with a check after every step.

**Read [`ARCHITECTURE.md`](ARCHITECTURE.md) first** if you want to know *what* the
system looks like and *why*. This document is the *how* and the *in what order*.

---

# ★ QUICK GUIDE

Everything below this section is detail. This part is the map.

## What changes, in one picture

```
        TODAY                                  TARGET
   ┌──────────────┐                      ┌──────────────┐
   │    BMI270    │                      │    BMI270    │
   └──────┬───────┘                      └──────┬───────┘
          │ I2C                                 │ SPI  (20x faster)
   ┌──────▼───────┐                      ┌──────▼───────┐
   │  Linux host  │  ← jitter lives      │  Teensy 4.1  │  ← no OS, no jitter
   │  estimator   │     here             │  estimator   │
   │  + control   │                      │  + control   │
   └──────┬───────┘                      └──────┬───────┘
          │ USB                                 │ CAN3 (built-in CAN-FD)
   ┌──────▼───────┐                      ┌──────▼───────┐
   │   fdcanusb   │  ← this box goes     │ transceiver  │
   └──────┬───────┘     away             └──────┬───────┘
          │ CAN-FD                              │ CAN-FD
   ┌──────▼───────┐                      ┌──────▼───────┐
   │    moteus    │                      │    moteus    │
   └──────────────┘                      └──────────────┘

                                     Linux stays for: calibration tools,
                                     tuning, logging — but not the loop.
```

## The three big questions

**Do I need the Arduino IDE?** No. `ARDUINO` is just a compiler flag. You keep this
repo, VS Code, and C++. You add one command next to the one you already run:

```bash
cmake --build build      # host binaries (today, unchanged)
pio run -e board_check      # Teensy firmware (new, same source files)
```

**How much code has to be rewritten?** Less than it looks:

| Layer | Verdict |
|---|---|
| `core/` — estimator, controller, types | **Nothing changes.** Already hardware-free |
| BMI270 register logic | ~90% reused; only the 4 bus functions change |
| moteus frame encoding | **Reused as-is** from the vendor headers |
| moteus transport (talking on the wire) | Rewritten — ~150 lines replaces 2960 |
| Apps (`cube_balancer` etc.) | Loop body reused; the CLI shell around it goes |

**Why bother?** [`ARCHITECTURE.md`](ARCHITECTURE.md) §6 concedes that Linux gives "a few
hundred microseconds of jitter" and ships a `chrt -f 50` workaround. A bare-metal chip
removes the operating system from the control loop entirely.

## The work, as blocks

Three tracks. **Hardware and software are independent** until the very end — you can
start the software today while parts are still shipping.

```
 PART I — HARDWARE (bench, multimeter, no code)
 ┌────────────────────────────────────────────────────────┐
 │ H1  Buy the right parts    → 3.3V transceiver ONLY     │
 │ H2  Wire IMU + CAN         → pins 30/31 for CAN        │
 │ H3  MEASURE before connect → 60Ω bus, 3.3V everywhere  │
 │ H4  TAKE THE WHEEL OFF     → stays off until the end   │
 │ H5  Wire a physical e-stop → software stop isn't enough│
 └────────────────────────────────────────────────────────┘

 PART II — SOFTWARE (no motor power at any point)
 ┌────────────────────────────────────────────────────────┐
 │ S1  Build system           → does it compile both ways?│
 │ S2  IMU driver (SPI)       → do I get real gravity?    │
 │ S3  Listen to CAN          → can I hear the moteus?    │
 │ S4  Talk to CAN (read)     → can I ask its position?   │
 │ S5  Verify board config    → is it safely configured?  │
 │ S6  Assemble the loop      → does it run at 200 Hz?    │
 └────────────────────────────────────────────────────────┘

 PART III — INTEGRATION (motor powered for the first time)
 ┌────────────────────────────────────────────────────────┐
 │ N1  First motion, WHEEL OFF → does it push the right   │
 │                               way when I tilt it?      │
 │ N2  Wheel on, tethered      → does it balance?         │
 │ N3  Telemetry               → optional, logging        │
 └────────────────────────────────────────────────────────┘
```

## The check after each step, in plain words

| Step | You are asking | You know it worked when |
|---|---|---|
| **S1** | Does my build setup work? | Both `pio run` and `cmake --build` succeed, and `core/` was not edited |
| **S2** | Is the IMU alive and honest? | ID reads `0x24`, gravity reads 9.81, and it flips sign when you turn it over |
| **S3** | Is my CAN wiring right? | Teensy sees the messages while your laptop talks to the moteus |
| **S4** | Can I read the motor? | You spin the wheel **by hand** and the numbers move |
| **S5** | Is the motor safely set up? | Reads back the right current limit — **and refuses to run when it's wrong** |
| **S6** | Does the loop hold time? | 200 Hz, essentially no missed cycles |
| **N1** | Is the direction right? | Tilt it, motor pushes back — **with the wheel removed** |
| **N2** | Does it balance? | It balances, tethered, on a soft surface |

## The four things that will bite you

Ordered by how much damage they do. Each has a specific check that catches it.

1. **The silent watchdog.** The moteus command format defaults to *not sending* torque
   or the safety watchdog. The frame looks perfect, the board says OK, and nothing
   happens — or worse, nothing stops. → Caught by unplugging the Teensy in N1.
2. **A sign error drives the cube into its fall**, at full power, instead of catching
   it. → This is why **the wheel comes off** (H4) until N1 passes.
3. **A 5 V transceiver destroys the Teensy's CAN pin.** Permanently — it is inside the
   chip. → Caught by measuring with a meter *before* connecting (H3).
4. **The clock wraps after 71 minutes** and time appears to run backwards, which makes
   the filter explode. → Caught by the 90-minute soak test (S2f). Boring, real.

## Scope

`StateEstimator::update()`, `accelAngle()` and `BalancingController::update()` are
deliberately user-written scaffolds, and the physical parameters and LQR gains are
unmeasured. **This document does not implement them.** Parts I and II proceed entirely
without them; only Part III depends on them.

---
---

# Toolchain: do I need the Arduino IDE?

**No. You work in C++, in this repository, in VS Code.**

`ARDUINO` is a **preprocessor define**, not an application. Teensy's hardware support
libraries (`SPI.h`, `FlexCAN_T4`) ship as an Arduino-*framework* package, and the
compiler defines `ARDUINO` automatically when building against it. That is the entire
extent of the involvement.

**PlatformIO is a command-line C++ build tool** — a VS Code extension plus a CLI. You
keep this repo, these files, this editor:

```bash
cmake --build build      # host binaries, exactly as today — unchanged
pio run -e board_check      # Teensy firmware, same sources
pio run -t upload        # flash the board
pio device monitor       # serial console
```

Two build systems over **one set of sources**. Nothing is copied into an IDE, no `.ino`
sketch files, no GUI. `platformio.ini` sits at repo root doing the same job
`CMakeLists.txt` does for the host.

### WSL2: build in Linux, flash from Windows

This repo lives in WSL2, and Teensy flashing does not survive that cleanly:

> **Flashing re-enumerates the USB device.** The board reboots into its HalfKay
> bootloader — a *different* USB device (`16C0:0478`) from the running sketch
> (`16C0:0483`). A `usbipd` attachment binds to one device, so it **drops on every
> upload**.

Install PlatformIO on both sides; they share the one `platformio.ini`:

| Task | Where | Command |
|---|---|---|
| Build | WSL2 | `pio run -e board_check` |
| **Flash** | **Windows** | `pio run -e board_check -t upload` |
| **Serial** | **Windows** | `pio device monitor` |

Windows reads the repo in place at `\\wsl$\Ubuntu\<path>` — no second clone.

This pays off most at **S3**, where `moteus_tool` runs over the fdcanusb *while* the
Teensy listens on CAN. Under pure-WSL that is two simultaneous usbipd attachments, one
of which drops on every upload.

Pure-WSL remains possible (`usbipd attach --wsl --busid X-Y` before each upload, plus
udev rules WSL does not ship). It works; it is friction on every cycle.

Full walkthrough — install, blink, serial, SPI loopback — in
[IMU_SETUP.md](IMU_SETUP.md) **Step 0**.

The one thing `-DARDUINO` changes for us: `moteus_protocol.h:26-37` takes its `#else`
branch, defining `NaN` as `(0.0 / 0.0)` rather than
`std::numeric_limits<double>::quiet_NaN()`. That is the behaviour we want on the MCU —
see the trap note in S1.

*Alternative considered and rejected:* cmake + a Teensy toolchain file. The community
toolchain files are unmaintained and you would spend a week on linker scripts to gain
nothing. PlatformIO handles the ARM toolchain, linker script and upload protocol.

---

# Why these decisions

## Verified from the Teensy datasheet

`docs/2D_model/datasheets/Teensy_datasheet.pdf` — NXP i.MX RT1060 rev 4, the silicon
inside Teensy 4.0/4.1.

| Fact | Line | Consequence |
|---|---|---|
| "Full featured FPU with support of the VFPv5 architecture" | 111 | **Double-precision in hardware.** `core/` is `double` throughout and needs no `float` rewrite |
| "Two FlexCAN modules / One FlexCAN (with Flexible Data-Rate supported)" | 197-198 | Only **CAN3** does FD. moteus requires FD, so CAN1/CAN2 are unusable |
| LPSPI absolute max 30 MHz | 8555 | BMI270's 10 MHz is the binding limit, not the MCU |
| LPI2C Fm+ max 1 MHz (Hs-mode is slave-only) | 8999-9011 | Confirms SPI is the higher-headroom choice |
| GPIO abs-max OVDD+0.3 V | Table 7 | **Not 5 V tolerant.** A 5 V transceiver destroys the pin |

## Why SPI and not I2C

Both work. SPI has headroom; I2C does not.

| | 12-byte burst | % of a 5 ms cycle |
|---|---|---|
| I2C @ 400 kHz (today) | ~350 µs | 7.0% |
| I2C @ 1 MHz (Fm+ max) | ~140 µs | 2.8% |
| **SPI @ 10 MHz** | **~15 µs** | **0.3%** |

I2C costs 2 pins and needs 2.2–4.7 kΩ pull-ups; SPI costs 4 pins and needs none. If you
ever want >1 kHz sampling, SPI is the only option that gets there.

## What the vendor library gives us for free

- **`moteus_transport.h` + `moteus.h` are hard POSIX** — `<termios.h>`, `<glob.h>`,
  `<sys/socket.h>`, a `std::thread` event loop, `<iostream>`. Line 518 says outright:
  *"For now, we'll only do linux like systems."* No bare-metal path. **Deleted, not ported.**
- **But `moteus_protocol.h` + `moteus_multiplex.h` are portable** and explicitly support
  Arduino-framework builds (`#ifndef ARDUINO` at `moteus_protocol.h:26-37`). They give us
  `PositionMode::Command`, `Query::Parse`, `CanFdFrame` and all the register encoding —
  **the fiddly part is reusable as-is.**
- **`DiagnosticWrite`/`DiagnosticRead`/`DiagnosticResponse` live in `moteus_protocol.h`
  (1384-1449), not `moteus.h`, and contain zero `std::`.** This makes the `conf get`
  verification tunnel in S5 cheap.
- From `moteus.h` only the arbitration-ID formula (1320-1358) is needed.

**~150 lines of Teensy code replaces 2960 lines of vendor transport.**

## ⚠ The highest-severity trap, verified in source

`PositionMode::Format` defaults, read directly from `moteus_protocol.h`:

```cpp
struct Format {
  Resolution position = kFloat;
  Resolution velocity = kFloat;
  Resolution feedforward_torque = kIgnore;   // ← silently not sent
  Resolution kp_scale = kIgnore;             // ← silently not sent
  Resolution kd_scale = kIgnore;             // ← silently not sent
  Resolution maximum_torque = kIgnore;       // ← silently not sent
  Resolution watchdog_timeout = kIgnore;     // ← SAFETY WATCHDOG NOT SENT
  ...
};
```

The host code never hits this because `Controller::Options` supplies a
`default_position_format`. **Hand-rolled frames must set these five to `kFloat`
explicitly.** Otherwise the frame transmits cleanly, the board replies OK, the motor
does nothing, and the watchdog does not exist. Invisible at the CAN layer.

---
---

# PART I — HARDWARE

Bench work. Multimeter, no code. Each step is verifiable before any software exists.

**Standing rule: the moteus motor power supply stays OFF for H1–H5 and all of Part II.**
The board enumerates on CAN and answers queries on logic power alone. Motor power is
enabled only at N1, after the software has been proven.

## H1 — Bill of materials

| Item | Requirement | Why |
|---|---|---|
| Teensy 4.1 | — | CAN3 broken out on pins 30/31 |
| CAN-FD transceiver | **3.3 V logic** — TCAN334, or MCP2562FD with VIO on 3.3 V | RT1060 is not 5 V tolerant |
| BMI270 breakout | 3.3 V, SPI pins exposed (SCK/SDI/SDO/CSB) | |
| Termination resistor | 120 Ω, if the bus does not already have two | 5 Mbit data phase is unforgiving |
| USB cable | Teensy power + programming | Keeps the MCU off the motor rail |

**Verify the transceiver's *logic* supply rating, not just its bus rating.** A 5 V
MCP2551 will destroy CAN3 RX. This is unrecoverable — the pin is inside the SoC.

## H2 — Mounting and wiring

**IMU (SPI):**

```
BMI270            Teensy 4.1
  VDD/VIN   ────  3.3V
  GND       ────  GND
  SCK       ────  13  (SCK0)
  SDI/MOSI  ────  11  (MOSI0)
  SDO/MISO  ────  12  (MISO0)
  CSB       ────  10  (CS, hardware chip-select)
  INT1      ────  (optional, future data-ready IRQ)
```

Keep leads short — you are targeting 10 MHz. Long dupont wiring is the usual cause of
marginal SPI at that rate.

**CAN:**

```
Teensy 30 (CAN3 TX) ──── TXD ┐
Teensy 31 (CAN3 RX) ──── RXD ┤ 3.3V CAN-FD transceiver
Teensy 3.3V         ──── VCC ┤
Teensy GND          ──── GND ┘
                             └── CANH / CANL ──── moteus
```

**Pins 30/31 are not negotiable** — CAN3 is the only FD-capable module on this silicon.

**Grounds:** Teensy GND, transceiver GND and moteus GND must be common. Without a shared
ground the link appears to work and then corrupts once motor current flows — the classic
reaction-wheel debugging nightmare.

**Power:** Teensy from USB, *not* from the moteus regulator, through the whole plan. A
shared rail browns out the MCU mid-control-loop, which is the worst possible failure.

## H3 — Electrical verification (before connecting the Teensy)

Do these **in order**, with the Teensy physically disconnected:

1. **Transceiver logic level.** Power the transceiver, probe RXD with a meter. Must idle
   at **3.3 V**. If it reads 5 V, stop — wrong part. Only after this passes do you wire
   pin 31.
2. **Bus termination.** Everything unpowered, measure CANH↔CANL end to end. Must read
   **~60 Ω** (two 120 Ω in parallel).
   - 120 Ω → only one terminator, add the second
   - 40 Ω → three terminators present, remove one (the moteus has an onboard jumper)
   - Wrong termination often passes at 1 Mbit arbitration and fails only in the 5 Mbit
     data phase — an intermittent that will cost you days
3. **Ground continuity.** Confirm Teensy GND ↔ moteus GND with a continuity beep.
4. **IMU rail.** Confirm 3.3 V at the BMI270 VDD pin, and that no BMI270 pin sees more.

**Check H3:** ~60 Ω bus; every Teensy-facing signal measured ≤3.3 V; ground continuity
confirmed. Only now connect the Teensy.

## H4 — Wheel removal

**Physically remove the reaction wheel from the motor shaft.** It goes back on only at N2.

This is the single most important safety step. A sign error in the tilt estimate, the
axis mapping or the torque direction produces a controller that drives the cube *into*
its fall at full authority. With the wheel off, that is a spinning bare shaft. With the
wheel on, it is a rig launching itself off the bench.

## H5 — Physical e-stop

Wire a **normally-closed** button between a spare GPIO and GND. Opening it must be read
by software every cycle and trigger `stop()`.

The host has `SIGINT` → `motor.stop()`. There is no MCU equivalent — no OS, no signals.
A software-only stop on a bare-metal device is not a stop.

---
---

# PART II — SOFTWARE

What to program, in order. Each step names its hardware dependency and states what it
proves. **No step here enables motor power.**

## S1 — Build system

*Depends on: nothing. Start here today.*

Add `platformio.ini` at repo root targeting `board = teensy41`.

**`core/` sources are referenced in place, never copied.** A copy means the host and
Teensy control laws drift, and you end up debugging a difference between your simulation
and your hardware. Use `build_src_filter` to pull `src/core/*.cpp` plus a new
`src/embedded/*.cpp`, with `build_flags = -I include`, explicitly excluding
`src/drivers/*` and `src/testing/*` (both POSIX).

The host CMake build is **completely unchanged** in this step.

**The `NaN` macro trap.** Under `ARDUINO`, `moteus_protocol.h:35` does
`#define NaN (0.0 / 0.0)` — an unnamespaced macro that is a *runtime divide*, not a
compile-time constant. Confine vendor headers to exactly one `.cpp`, mirroring the
existing forward-declaration discipline in `MoteusDriverWrapper.hpp:18-25`. Assign it
once to a file-scope `static const double kNan` rather than re-dividing at 200 Hz.

**Check S1:**
- `pio run -e board_check` links `src/core/StateEstimator.cpp` and
  `src/core/BalancingController.cpp` **unmodified**
- `cmake --build build` still produces all four host binaries, zero `CMakeLists.txt` diff
- Flash a sketch printing `sizeof(ImuData)` and `sizeof(MotorState)` — must read **72**
  and **56**, matching the host. Proves ABI and that `double` is 8 bytes

**Fail if** you find yourself adding `#ifdef ARDUINO` under `include/core/` or
`src/core/`. Stop and reconsider — that is precisely the failure the architecture exists
to prevent.

## S2 — BMI270 SPI driver

*Depends on: H2 (IMU wired), H3 (rails verified).*

> **📋 Step-by-step procedure: [IMU_SETUP.md](IMU_SETUP.md).** It walks the wiring,
> the `CHIP_ID` test, the blob upload, the gravity check, both calibrations and the
> soak test, with a tick-box at every stage. The rest of this section is the summary
> and the porting notes.

**What ports unchanged.** In `src/drivers/ImuDriver.cpp` only four things are Linux:
`writeRegister` (77-80), `readRegisters` (82-89), `SleepMicroseconds` (46-51),
`MonotonicSeconds` (53-58). The blob upload state machine (91-178), init sequence
(180-268), scale factors (262-265), sample decode (270-302) and `calibrateGyroBias`
variance check (304-371) are **bus-agnostic and port as written**.

Write `Bmi270SpiDriver : IImuSensor` in `src/embedded/`. Factor the register constants
from `ImuDriver.hpp:30-51` into a shared `include/drivers/Bmi270Registers.hpp` so the two
drivers cannot drift.

**The four real SPI differences:**

1. **Read bit + dummy byte.** Send `reg | 0x80`, discard one garbage byte, then read.
   The BMI270 returns one invalid byte on every SPI read. Most common BMI270 SPI bug,
   and it has no I2C equivalent.
2. Writes send `reg & 0x7F` then the value.
3. **Power-on mode latch.** The part boots in I2C; the first CS falling edge switches it
   to SPI, so the first read is invalid *by design*. The existing defensive triple-read
   at `ImuDriver.cpp:202-215` happens to be exactly right for SPI — keep it.
4. **Blob burst.** Replace the per-chunk `std::vector` (124-151) with a fixed
   `uint8_t packet[N]`. **Do not raise CS between the register byte and the payload** —
   the internal write pointer auto-increments only within one CS assertion. Raise the
   chunk to 256 bytes: 8192 bytes drops from 256 transactions to 32.

**Clock ≤2 MHz during blob upload**, then 10 MHz for normal operation. Uploading at
10 MHz gives an `INTERNAL_STATUS` that never reaches `0b0001`, and sends you hunting a
wiring fault that does not exist.

The blob is already in the repo at `third_party/bmi270_config.h` — no acquisition step.

**`micros()` rollover.** Teensy `micros()` is `uint32_t` and wraps every ~71 minutes.
Naive conversion to seconds makes `dt` go hugely negative once an hour; the complementary
filter and the derivative term both explode. **Accumulate a 64-bit microsecond counter**
by tracking wraps against the previous value.

**Check S2 — in strict order, do not skip ahead:**

| # | Check | Pass criterion | If it fails |
|---|---|---|---|
| a | CHIP_ID | reads `0x24` | wiring, CS, or mode — stop here |
| b | Blob upload | `INTERNAL_STATUS & 0x0F == 0x01` | drop SPI to 1 MHz; check CS discipline |
| c | Gravity | `\|a\|` = 9.81 ±0.3 m/s², **sign flips when inverted** | scale factor or axis mapping |
| d | Gyro at rest | `calibrateGyroBias(2.0, &err)` returns true | reuses the <0.02 rad/s threshold at `ImuDriver.cpp:356` |
| e | Burst timing | 12-byte read **<20 µs** | >100 µs means you lost the reason for choosing SPI |
| f | **90-minute soak** | `dt` never negative | catches the `micros()` rollover — boring, do it anyway |

## S3 — CAN-FD receive only

*Depends on: H2 (CAN wired), H3 (termination + grounds). moteus on LOGIC POWER ONLY.*

Bring up FlexCAN_T4 on CAN3 in listen-only/passive mode. 1 Mbit arbitration / 5 Mbit data.

**Verify reception before ever transmitting.** A moteus at rest is quiet, so provoke it:
from the Linux host run `moteus_tool -t 1 --dump-config` or `tview` over the existing
fdcanusb, while the Teensy listens on the same bus.

This proves bit timing, termination and transceiver polarity while the Teensy is
physically incapable of commanding anything.

**Check S3:**
- Teensy prints arbitration IDs matching `dest | (reply_required ? 0x8000 : 0)`
  (formula from `moteus.h:1325-1329`)
- 60 seconds of well-formed frames with **zero CRC/form errors**
- FlexCAN_T4 TX/RX error counters stay at **0**
- Any error-frame storm → bit timing or termination. Return to H3

## S4 — `TeensyMoteusDriver`, query path only

*Depends on: S3 passing. `sendTorque()` is a deliberate no-op in this step.*

Write `TeensyMoteusDriver : IMotorDriver` in `src/embedded/`. Roughly 150 lines.

**Do not port `ThreadedEventLoop`.** It exists to paper over POSIX blocking I/O; on bare
metal a polled FlexCAN mailbox is simpler, faster and deterministic. The control loop is
already synchronous.

Design:
- **`makeArbitrationId()`** — reproduce `moteus.h:1325-1329`. Single controller, so
  `source` = 0 and `can_prefix` = 0.
- **`buildFrame()`** — stack-allocate `CanFdFrame`, wrap in
  `WriteCanData write_frame(result.data, &result.size)`, call `PositionMode::Make(...)`
  or `Query::Make(...)`. Reuses all vendor encoding.
- **`transact()`** — copy into `CANFD_message_t`, set `.brs = 1`, `.edl = 1`, and **pad
  `.len` up to a valid FD DLC** (0-8, 12, 16, 20, 24, 32, 48, 64) with zeros. Poll for
  the reply with a timeout.
- **Reply parse** — `Query::Parse()`, then lift `FromQuery`
  (`MoteusDriverWrapper.cpp:18-29`) verbatim into a shared header; it is already pure.

**Command shape** copied exactly from `MoteusDriverWrapper.cpp:207-223`: `position = NaN`,
`velocity = 0`, `kp_scale = kd_scale = 0`, plus `feedforward_torque`, `maximum_torque`,
`watchdog_timeout`.

**Set `Format` explicitly** — see the trap above. All five of `feedforward_torque`,
`kp_scale`, `kd_scale`, `maximum_torque`, `watchdog_timeout` → `kFloat`.

**Check S4:**
- `query()` returns `valid=true`, plausible `bus_voltage` (compare against a meter),
  `fault == 0`
- **Spin the wheel by hand** — `position` and `velocity` track. Proves encoder decode and
  sign convention with zero stored energy
- 200 Hz query loop sustained 10 minutes; log round-trip latency (budget: well under
  200 µs each at 5 Mbit — >1 ms means something is retrying)
- `stop()` yields `mode == 0`

## S5 — Config provisioning and verification

*Depends on: S4 passing.*

**Provision from the host, persist to flash, verify from the Teensy.**

Do **not** reimplement `applyConfig()` on the MCU. The load-bearing sleeps
(`MoteusDriverWrapper.cpp:117` 50 ms retry, `:127` 2 ms settle) and the `ERR` retry
encode real hardware failures documented in this repo; re-earning that knowledge on a new
platform is pure risk for no gain. Settings are pushed **once**, not per boot.

Flow: host runs the existing `applyConfig()` over fdcanusb with
`-DPERSIST_CONFIG_TO_FLASH=ON`, once. The moteus then boots correctly configured forever,
independent of the Teensy.

**But the Teensy must verify before arming.** A silently-wrong `servo.max_current_A` is
rig-destroying. This extends the existing `firstUnsetField()` refuse-to-run discipline
across the CAN link.

Cheap to implement, because `DiagnosticWrite`/`DiagnosticRead`/`DiagnosticResponse`
(`moteus_protocol.h:1384-1449`) are `std::`-free: send `conf get servo.max_current_A\n`,
poll, accumulate into a fixed `char[256]` until `\n`. ~60 lines.

**Read-only — `conf set` from the Teensy is out of scope.** A read cannot brick the board.

`DEFAULT_CONFIG_PATH` has no MCU equivalent and needs none — do not add SD or LittleFS.
The Teensy needs only the *expected values* of the safety-critical subset
(`servo.max_current_A`, `motor_position.rotor_to_output_ratio`, `servo.max_velocity`) as
`build_flags` mirroring the CMake cache vars.

**Check S5:**
- Host push with `conf write`; **power-cycle the moteus**; Teensy reads back the expected
  values
- **The negative test:** deliberately push a wrong `servo.max_current_A` and confirm the
  Teensy **refuses to arm**. An unverified verifier is worse than none

## S6 — Control loop scaffolding

*Depends on: S2, S4, S5 passing. Still no motor power.*

Assemble the loop body: BMI270 → `StateEstimator::update()` →
`BalancingController::update()` → `TeensyMoteusDriver::sendTorque()`, with `sendTorque()`
still stubbed or gated behind a `--dry-run` equivalent.

Port the **absolute-deadline scheduler** structurally from `cube_balancer.cpp:303-313` —
`next_deadline += period`, late-cycle counting, resync on gross overrun. Only the sleep
call changes. With no OS scheduler competing, late cycles should be ~0%; **measure it**,
since that number is the entire justification for this port.

Wire the **physical e-stop** (H5) into the loop: read the GPIO every cycle, call `stop()`
on open.

**Check S6:**
- Loop runs at 200 Hz with >99.9% of cycles meeting deadline over 10 minutes
- Estimator and controller report `armed=false` / `kUnconfigured` while gains are unset —
  proving the NaN-sentinel gate survived the port
- E-stop GPIO reads correctly and is polled every cycle

---
---

# PART III — INTEGRATION

Both tracks meet. **This is the first time motor power is enabled.**

**Blocked on:** your `StateEstimator::update()`, `accelAngle()`,
`BalancingController::update()`, the measured physical parameters, and the LQR gains.
Everything in Parts I and II proceeds without them.

## N1 — First motion, wheel OFF

*Requires: H4 (wheel physically removed), all of Part II, your control law written.*

Enable motor power for the first time. Tilt the cube by hand; confirm the motor tries to
spin in the **correct direction** with roughly the right magnitude.

**Sign convention is what this step tests** — see H4 for why the wheel must be off.

**Check N1:**

| Check | Pass criterion |
|---|---|
| Torque direction | correct for **both** tilt directions, wheel absent |
| **Watchdog** | unplug the Teensy mid-command → shaft stops within **150 ms** (`20 × period` = 100 ms). **If it keeps spinning, `format.watchdog_timeout` was left `kIgnore`** — the S4 trap |
| Tilt e-stop | exceeding `max_tilt_rad` latches and zeroes torque |
| Physical e-stop | zeroes torque within one cycle |
| Timing | >99.9% of cycles meet the 200 Hz deadline over 10 min |

## N2 — Wheel ON, tethered

Mount the wheel. Soft surface, cube tethered or in a travel-limiting jig, hand on the
power cut.

Start at **~30% of the LQR gains** with `max_torque_nm` clamped low; walk both up
together across runs. Tune `k_omega` **last**, per `BalancingController.hpp:127`.

**Check N2:** sustained balance, bounded wheel speed, no limit cycle, no persistent
saturation — log `ControlOutput::torque_clamped` directly.

## N3 — Telemetry (optional)

Fixed-width binary records over USB serial, every Nth cycle. **Never `printf` in the
control loop** — formatting doubles at 200 Hz destroys the timing budget. Reuse
`CsvLogger` host-side unchanged.

---
---

# Reference

## Interface changes: leave them alone

`std::string` appears only in `initialize()`, `calibrateGyroBias()` and `name()` — all
startup-only. The hot path (`read`/`sendTorque`/`query`/`stop`) is already POD-by-value.
Teensy newlib handles `std::string`; a few hundred bytes of boot-time heap on a 1 MB part
is irrelevant, and nothing allocates in a loop.

Changing the interfaces to `char*`/`size_t` would touch both host drivers and three apps
for zero measured benefit. **Do not.** Revisit only if S6 timing shows a real problem —
it will not.

## What stays on Linux permanently

`moteus_tool` and `tview` (firmware, calibration, live plotting); config provisioning via
`applyConfig()`; LQR gain computation (scipy Riccati,
[`../3d_scaling/README.md`](../3d_scaling/README.md) §4); `CsvLogger` analysis;
`MotorValidator` pre-flight checks including the deliberate watchdog provocation; and
host-side simulation of `core/` — the reason for the dual build, letting you
regression-test the control law on the desktop after every change.

## Risk register

| # | Risk | Caught by |
|---|---|---|
| 1 | **`Format` defaults → torque + watchdog silently never sent.** Zero symptoms at CAN layer | S4 explicit `kFloat`; **N1 watchdog disconnect test** |
| 2 | **Sign error → controller drives the fall at full authority** | **H4 wheel removal** + N1. Non-negotiable |
| 3 | **5 V transceiver destroys CAN3** (unrecoverable) | H1 part selection; **H3 meter check** |
| 4 | **`micros()` rollover at 71 min → negative `dt`** | S2 64-bit counter; **S2f 90-min soak** |
| 5 | Shared rail brownout resets Teensy mid-loop | H2 USB supply; watchdog catches the reset |
| 6 | Blob upload at 10 MHz fails `INTERNAL_STATUS` | S2 ≤2 MHz during upload; S2b |
| 7 | `core/` diverges between builds | S1 reference-in-place, never copy |
| 8 | FD DLC not padded → frame rejected | S4 pad in `transact()` |
| 9 | Missing common ground → corruption only under motor current | **H3 continuity check** |
| 10 | `NaN` macro leaks into `core/` | S1 vendor headers in one `.cpp` |
| 11 | Wrong termination → passes at 1 Mbit, fails at 5 Mbit | **H3 resistance**; S3 error counters |
| 12 | Config drift host↔Teensy | S5 read-back **with negative test** |

## Why this ordering

Every failure is diagnosed with the least possible stored energy present: electrical
faults with no power at all (H3), protocol faults on logic power only (S3–S5),
sign-convention faults with the wheel removed (N1), and control-law faults with the cube
tethered (N2).
