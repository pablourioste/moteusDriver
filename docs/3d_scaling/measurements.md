# Rig measurements — the fill-in worksheet

**This is the only file you have to edit.** Take the rig to a scale and a knife
edge, write the numbers into the fenced block at the bottom, then run:

```bash
./moteus-venv/bin/python tools/lqr/solve_gains.py
```

It parses this file, solves the LQR problem, checks the result is actually
stable, and prints the `-D CTL_K_*` build flags to paste into
`[env:cube_balancer]` in `platformio.ini`. It refuses while any value is still
`nan`, and names the first one it finds.

Every value ships as the literal `nan` for the same reason the gains do: a
plausible-looking placeholder is exactly what makes an invented number look
authoritative. There are no defaults here to fall back on.

The procedures are in [README.md §3](README.md). This file is where the results
go, plus the raw readings behind them — a lone number with no working is one
nobody can check or re-derive six months later.

---

## Provenance

Fill this in at the same time as the numbers, not afterwards.

| | |
|---|---|
| Date measured | `____________` |
| Measured by | `____________` |
| Ambient temperature | `______` °C |
| Rig configuration | `____________________________________` |
| Scale used (and resolution) | `____________________________________` |

**Rig configuration** matters more than it looks. Record whether the wheel was
mounted, whether the battery/tether was attached, and where the Teensy and the
moteus were sitting. Every one of those is mass at a radius, and the whole
point of `l` is where the mass is. Re-measure if the layout changes.

---

## 1. Masses

Weigh the body **without** the wheel, then the wheel on its own. Three readings
each, lifting the item off the scale between them — a scale that has settled
reads repeatably even when it is wrong.

### `m_b` — body mass, wheel excluded

| Reading | kg |
|---|---|
| 1 | `______` |
| 2 | `______` |
| 3 | `______` |
| **mean → `m_b`** | `______` |

### `m_w` — wheel mass

| Reading | kg |
|---|---|
| 1 | `______` |
| 2 | `______` |
| 3 | `______` |
| **mean → `m_w`** | `______` |

---

## 2. Lengths

Both measured **from the pivot edge** — the edge the cube balances on, not the
centre of the cube and not the corner.

### `l` — pivot to body centre of mass

Balance the body (wheel off) across a knife edge, straightedge or drill bit,
parallel to the pivot edge. Where it balances is the CoM line; measure from the
pivot edge to there.

| Trial | m |
|---|---|
| 1 | `______` |
| 2 | `______` |
| **mean → `l`** | `______` |

Balance it twice, approaching from opposite sides. If the two disagree by more
than a few percent, there is friction in the knife edge — use something sharper.

### `l_w` — pivot to wheel axis

A direct measurement, no balancing needed: pivot edge to the centre of the motor
shaft.

**`l_w` = `______` m**

---

## 3. Body inertia `I_b`, by swing test

Hang the cube from its pivot edge so it swings freely as a physical pendulum,
and give it a **small** amplitude — under ~10°, or the small-angle assumption
behind the formula stops holding and the measured period comes out long.

Do this with the **wheel mounted but locked**, then subtract the wheel's
contribution. Time 20 full oscillations and divide; do it twice and average.

| Run | 20 oscillations (s) | → T = t/20 (s) |
|---|---|---|
| 1 | `______` | `______` |
| 2 | `______` | `______` |
| **mean T** | | `______` |

Then, for a physical pendulum:

```
I_pivot = (m_total · g · l_total · T²) / (4π²)      with g = 9.80665
I_b     = I_pivot − m_w · l_w²
```

where `m_total` and `l_total` are for the cube **as swung** (wheel included).
`solve_gains.py` does this arithmetic — record `T` in the block below and let
it compute, rather than doing it by hand here.

| | |
|---|---|
| **T_swing** (mean period, s) | `______` |

Only ~5% accuracy is needed. The LQR gains are not sensitive to this at the
third significant figure, and the tuning ramp on the rig absorbs the rest.

---

## 4. Wheel inertia `I_w`

Pick whichever applies and record the result.

- **Solid disc:** `I_w = ½ · m_w · r²`. Measure `r` and let the script compute
  it — fill in `wheel_radius_m` below and leave `I_w` as `nan`.
- **Spoked or rimmed wheel, or a CAD value:** fill in `I_w` directly and leave
  `wheel_radius_m` as `nan`. A CAD number is usually better than a bench
  measurement here.

Fill in exactly one of the two. The script refuses if both or neither are set,
because silently preferring one would hide a typo in the other.

| | |
|---|---|
| Wheel geometry (disc / spoked / rimmed / other) | `____________` |
| Where `I_w` came from, if given directly | `____________________` |

---

## 5. Not measured — computed or constant

Listed so nobody goes looking for them with a ruler. `solve_gains.py` prints
these; they are not inputs.

| Symbol | Value | Where from |
|---|---|---|
| `g` | 9.80665 m/s² | defined SI constant |
| `K_t` | 60 / (2π · kv) ≈ **0.0256** Nm/A | kv = 373.6, from `data/calibration/moteus-cal-AD4AN1QwUBYgOTNO-*.log` |
| torque ceiling | `K_t · servo.max_current_A` | the firmware reads `servo.max_current_A` back over CAN at boot and refuses to run if `CTL_MAX_TORQUE_NM` exceeds it |

---

## THE VALUES

Everything above is working; this is the answer. `solve_gains.py` reads **only**
this block — plain `name value` lines, the same shape as
`config/current_config.cfg.in`. Replace each `nan` with a number. Do not rename
the keys, and do not add a second copy of this block anywhere.

```measurements
# name              value      unit    where it came from
m_b                 nan        # kg    section 1, mean of 3
m_w                 nan        # kg    section 1, mean of 3
l                   nan        # m     section 2, knife edge
l_w                 nan        # m     section 2, direct
T_swing             nan        # s     section 3, mean period of 20 swings
I_w                 nan        # kg m^2  section 4 -- OR set wheel_radius_m
wheel_radius_m      nan        # m     section 4 -- OR set I_w, not both
```

---

## After the solve

Paste the printed flags into `[env:cube_balancer]` in `platformio.ini`, and
record what was pasted here so this file stays the whole story:

| | |
|---|---|
| Gains pasted on | `____________` |
| `k_theta` | `______` |
| `k_theta_dot` | `______` |
| `k_omega` | `______` |
| Q, R used (if not the defaults) | `____________________` |

### The sign — record what N1 showed

The solve fixes the gain **magnitudes**. It cannot fix their **sign**: what a
positive `feedforward_torque` physically does to the cube depends on the motor
phase order and `SOURCE0_SIGN`, which no model can see. Note also that the sign
the solve produces contradicts `BalancingController.hpp`'s per-field comment —
see [README.md §4](README.md), "The sign is not decidable here". Treat both as
unverified until the rig says otherwise.

| | |
|---|---|
| N1 run on | `____________` |
| Tilting **toward** \_\_\_\_\_\_ made `theta` go | `positive / negative` |
| The shaft pushed | `the correcting way / the wrong way` |
| `EST_INVERT_THETA` ended up | `true / false` |
| Final sign of `k_theta` | `positive / negative` |

If the shaft pushed the wrong way, the fix is `EST_INVERT_THETA` (or
`SOURCE0_SIGN`) — **never** negating the control law. Negating it leaves the
damping term pointing the wrong way and buries the real bug somewhere worse.

Then the rig, per [README.md §4 "Tuning procedure"](README.md): the firmware
boots at `gain_scale` 0.1 and steps through 0.3, 0.6, 1.0 with `+`, one run per
step. Tune `k_omega` **last**, after the cube balances at all.

**A solve is not a validation.** These gains come from a linearised model of a
rig measured with a kitchen scale. They are a starting point that is the right
order of magnitude — nothing more.
