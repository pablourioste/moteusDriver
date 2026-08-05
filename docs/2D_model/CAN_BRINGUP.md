# CAN bring-up — the two tests

Runbook for INTEGRATION.md S3 and S4. Take the Teensy from "never talked
to the moteus" to "queries it 200 times a second", without the motor ever
being able to move.

Two tests, in order. **Test 1 must pass before Test 2 is flashed.**

| | Test 1 (S3) | Test 2 (S4) |
|---|---|---|
| env | `can_listen` | `moteus_driver_test` |
| Teensy transmits? | **no** — LISTEN_ONLY | yes |
| motor can move? | no | no — torque path disarmed |
| proves | bit timing, termination, polarity | frames, arbitration, reply parsing |

---

## Before anything: H3 meter checks

**Teensy disconnected. In this order.** Step 1 is destructive if skipped.

**1 — Transceiver logic level.** Power the transceiver on its own, probe
RXD against GND.

- Must idle at **3.3 V**.
- **5 V → STOP.** The RT1060 is not 5 V tolerant. Connecting pin 31
  destroys CAN3 RX permanently, inside the SoC. No repair, no workaround.
- Only once this passes do you wire pin 31.

**2 — Bus termination.** Everything unpowered, measure CANH↔CANL:

| Reading | Meaning |
|---|---|
| **~60 Ω** | correct — two 120 Ω in parallel |
| 120 Ω | only one terminator (acceptable under 0.5 m, see below) |
| 40 Ω | three terminators — remove one |

**moteus controllers have no termination resistors** (upstream
`docs/reference/pinouts.md`). So the two 120 Ω must come from elsewhere:

- one in the **fdcanusb** — software-configurable, **on by default**
- one you supply — a JST PH3 CAN terminator, or 120 Ω crimped into a
  PHR-3 housing plugged into the board's spare data connector

The moteus has two CAN connectors wired in parallel: daisy-chain in one,
terminator in the other.

**3 — Ground continuity.** Teensy GND ↔ moteus GND must beep.

## Wiring

```
Teensy 30 (CAN3 TX) ──── TXD ┐
Teensy 31 (CAN3 RX) ──── RXD ┤ 3.3 V CAN-FD transceiver
Teensy 3.3V         ──── VCC ┤   (TCAN334, or MCP2562FD with VIO on 3.3V)
Teensy GND          ──── GND ┘
                             └── CANH / CANL ──── moteus (JST PH-3)
```

**Pins 30/31 are not negotiable** — CAN3 is the only FD-capable module on
this silicon, and moteus requires FD. CAN1 and CAN2 cannot talk to it.

Connector is **JST PH-3**, 2 mm pitch. Mate PHR-3, terminals
SPH-002T-P0.5L.

**Teensy powered from USB-C, never VIN**, through both tests. A shared
rail sags when the motor draws current, and a Teensy that browns out
mid-control-loop is the worst failure mode available.

---

# TEST 1 — S3, listen only

The Teensy is in `LISTEN_ONLY`, a peripheral mode in which the CAN
controller never drives the bus: no acknowledgements, no error frames. It
is physically incapable of commanding the moteus, which is what makes
this safe to run with the controller powered.

## Commands

Build (WSL):

```bash
cd /home/pablo_urioste/projects/moteusDriver
~/.platformio-venv/bin/pio run -e can_listen
```

Flash (Windows, Teensy Loader — WSL cannot flash, the HalfKay bootloader
enumerates as a different USB device and drops any usbipd attachment):

```
\\wsl$\Ubuntu\home\pablo_urioste\projects\moteusDriver\.pio\build\can_listen\firmware.hex
```

Monitor: PuTTY, **115200**, session logging on.

Then provoke traffic — a moteus at rest is silent, so without this you
see nothing and it means nothing:

```bash
./moteus-venv/bin/moteus_tool -t 1 --dump-config
```

## Pass criteria

- [ ] frames appear, e.g. `id=0x8001 dst=1 src=0 reply` and `id=0x0100`
- [ ] after 60 s: `TX err=0  RX err=0`
- [ ] state reads `Idle` / `Error Active`
- [ ] no `*** BUS ERRORS ***`

**`ACK_ERR` is expected and harmless here.** In listen-only the
peripheral cannot acknowledge, by design. The sketch labels it as
ignorable so it does not look like a missed diagnosis.

**Frames arriving and zero counters are different claims.** Frames prove
the wiring is roughly right; zero counters prove the bit timing is right.
Only the second survives the 5 Mbit data phase — wrong termination
routinely passes at 1 Mbit arbitration and fails only in the data phase,
which reads as intermittent corruption and costs days.

**Do not continue with non-zero counters.** Return to H3 step 2.

---

# TEST 2 — S4, query path

The Teensy transmits here. The motor still cannot move: `sendTorque()`
returns a query unless `enableTorque(true)` is called, and nothing in
this sketch calls it. That is the software half of the discipline H4
applies physically.

## Before flashing

- [ ] Test 1 passed with **zero** error counters
- [ ] **reaction wheel physically off the shaft** (H4)
- [ ] **moteus on LOGIC POWER ONLY** — motor supply OFF

The board enumerates on CAN and answers queries on logic power alone.
Motor power is not needed until N1; the wheel goes back on at N2.

H4 matters because a sign error in the tilt estimate, the axis mapping or
the torque direction produces a controller that drives the cube *into*
its fall at full authority. With the wheel off that is a bare spinning
shaft.

## Commands

```bash
~/.platformio-venv/bin/pio run -e moteus_driver_test
```

Flash `.pio/build/moteus_driver_test/firmware.hex`, then PuTTY at 115200.

## Menu

| Key | What | Pass |
|---|---|---|
| `q` | single query | `valid`, bus voltage matches a meter, `fault 0` |
| `h` | **hand-spin** | position tracks, velocity signed by direction |
| `t` | 200 Hz, 10 s | `failed=0`, worst < 1 ms |
| `T` | 200 Hz, 600 s soak | same |
| `c` | config read-back (S5) | every setting `OK` |
| `s` | stop, clears faults | `mode 0` |

If `q` reports a non-zero fault, press `s` first — a board left in fault
ignores commands silently, which looks exactly like a wiring problem.

## The hand-spin test is the real check

Turn the motor shaft **by hand** and watch position and velocity track.
This proves encoder decode, sign convention and the whole parse path with
zero stored energy anywhere in the system.

**Write down which direction gives positive velocity.** The control law
needs that sign, and it is the cheapest possible moment to establish it.

## Timing

Budget is well under 200 µs per round trip at 5 Mbit. Anything over 1 ms
means something is retrying at the CAN layer — which S3's error counters
would not necessarily have shown, because a retry that eventually
succeeds is not an error.

---

## Troubleshooting

| Symptom | Cause |
|---|---|
| no frames at all (T1) | CANH/CANL swapped; transceiver unpowered; wrong pins (must be 30/31, not another CAN module) |
| frames but rising counters | termination — check ~60 Ω. Passes at 1 Mbit, fails at 5 Mbit |
| `initialising... FAIL` (T2) | link problem, not power — the board answers on logic power alone. Re-run Test 1 |
| queries valid, position frozen | encoder, not CAN. Check `aux2.spi.active` reads `True` in tview |
| worst RTT > 1 ms | retries at the CAN layer; suspect termination or grounding |

## What these tests do not cover

**Torque.** Both tests leave the motor incapable of moving, and not by
convention — `moteus_driver_test` is compiled without
`-D ENABLE_TORQUE_TEST`, so the instructions that command torque are
**not in the binary at all**. No keypress can move the motor.

First motion is N1, which is a *different environment*
(`moteus_driver_test_torque`) built from the same source with that flag
added. Arming the motor requires flashing different firmware, with the
checklist in front of you, rather than pressing a key on a build you
already had loaded.

Do not flash it until Test 2 passes and the wheel is off the shaft.

**Config provisioning.** `c` reads settings back and compares them; it
never writes. Provisioning stays on the host, where `applyConfig()`'s
load-bearing retry sleeps already encode real failures this hardware has
hit. Re-earning that on a new platform is risk for no gain — and a read
cannot brick the board.

To make `c` prove anything, run the negative test: push a deliberately
wrong value from the host (`moteus_tool -t 1 --console`, then
`conf set servo.max_current_A 3.0`) and confirm `c` reports a mismatch. A
check that has never failed is not known to work.
