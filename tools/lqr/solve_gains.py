#!/usr/bin/env python3
"""Turn the measured rig parameters into LQR gains, as build flags.

    ./moteus-venv/bin/python tools/lqr/solve_gains.py

Reads the ``measurements`` block in docs/3d_scaling/measurements.md, solves
the continuous-time Riccati equation for the linearised plant, checks the
result is actually stabilising, and prints the ``-D CTL_K_*`` flags to paste
into ``[env:cube_balancer]`` in platformio.ini.

WHY THIS IS A SCRIPT AND NOT A CODE BLOCK IN A DOC

docs/3d_scaling/README.md section 4 carries this solve as a fenced snippet.
That is fine for reading and wrong for using: every revision of a measurement
means pasting it into a REPL again, and the numbers it consumes then live in
two places -- the doc that recorded them and the terminal that used them.
This repo has already been bitten once by one number living in three files
(see the note above ``[env]``'s build_flags in platformio.ini, and the
2026-08-05 failure it records). So measurements.md is the single source of
truth and this parses it.

WHY IT REFUSES RATHER THAN WARNS

Two hard failures, both modelled on ``firstUnsetField()`` in core/:

  * Any value still ``nan`` -> stop and name it.  A NaN entering
    ``solve_continuous_are`` does not raise; it propagates, and the gain that
    comes out is a NaN that would disable every safety comparison downstream.
    Every comparison against NaN is false, so an unset gain fails silently in
    exactly the direction that matters.

  * A closed loop that is not stable -> stop and print the eigenvalues.
    README.md section 4 lists this as a "sanity check" in prose.  Prose does
    not stop anyone from pasting the gains anyway.

Neither is a case where printing a warning and continuing helps: the output
of this script goes straight onto a rig.
"""

import argparse
import math
import pathlib
import re
import sys

try:
    import numpy as np
    from scipy.linalg import solve_continuous_are
except ImportError:  # pragma: no cover - environment problem, not logic
    sys.exit(
        "numpy and scipy are required.\n"
        "This repo already has them in the bring-up venv:\n"
        "  ./moteus-venv/bin/python tools/lqr/solve_gains.py"
    )


REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
DEFAULT_MEASUREMENTS = REPO_ROOT / "docs" / "3d_scaling" / "measurements.md"

# Standard gravity.  A defined SI constant, matching kGravity in
# src/core/StateEstimator.cpp -- not a tuning value.
G = 9.80665

# K_t = 60 / (2*pi*kv), kv = 373.6 from this motor's calibration log
# (data/calibration/moteus-cal-AD4AN1QwUBYgOTNO-*.log).  Mirrors
# kTorqueConstantNmPerA in firmware/full_case/cube_balancer/main.cpp.
MOTOR_KV = 373.6
K_T = 60.0 / (2.0 * math.pi * MOTOR_KV)

# Keys the measurements block is allowed to contain.  An unknown key is an
# error rather than something to ignore: it is almost always a typo in a name
# that then silently keeps its nan.
KNOWN_KEYS = {
    "m_b",
    "m_w",
    "l",
    "l_w",
    "T_swing",
    "I_w",
    "wheel_radius_m",
}

# Required for the solve.  I_w / wheel_radius_m are handled separately --
# exactly one of the pair must be set.
REQUIRED_KEYS = ["m_b", "m_w", "l", "l_w", "T_swing"]


def parse_measurements(path):
    """Pull the fenced ``measurements`` block out of the markdown file.

    Plain ``name value`` lines with ``#`` comments, deliberately the same
    shape as config/current_config.cfg.in -- which ReadConfig() in
    apps/legacy_moteus_driver.cpp already parses.  One text format for
    hand-entered configuration in this repo, not two.
    """
    if not path.is_file():
        sys.exit(f"No such file: {path}")

    text = path.read_text(encoding="utf-8")
    blocks = re.findall(r"^```measurements\s*$(.*?)^```\s*$", text,
                        re.MULTILINE | re.DOTALL)
    if not blocks:
        sys.exit(
            f"{path}: no ```measurements block found.\n"
            "The block is what this script reads; the tables above it are for "
            "humans."
        )
    if len(blocks) > 1:
        # Two blocks means two answers, and picking one silently is how the
        # doc and the rig stop agreeing.
        sys.exit(
            f"{path}: found {len(blocks)} ```measurements blocks, expected 1.\n"
            "Delete the extras -- there can only be one source of truth."
        )

    values = {}
    for lineno, raw in enumerate(blocks[0].splitlines(), start=1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) != 2:
            sys.exit(
                f"{path}: measurements block line {lineno}: expected "
                f"'name value', got {raw.strip()!r}"
            )
        name, text_value = parts
        if name not in KNOWN_KEYS:
            sys.exit(
                f"{path}: unknown measurement {name!r}.\n"
                f"Known names: {', '.join(sorted(KNOWN_KEYS))}"
            )
        if name in values:
            sys.exit(f"{path}: {name!r} appears twice in the block.")
        try:
            values[name] = float(text_value)
        except ValueError:
            sys.exit(
                f"{path}: {name} = {text_value!r} is not a number "
                "(use 'nan' for a value not yet measured)"
            )
    return values


def first_unset(values):
    """Name the first required value still nan, mirroring firstUnsetField()."""
    for key in REQUIRED_KEYS:
        if key not in values or math.isnan(values[key]):
            return key
    return None


def resolve_wheel_inertia(values):
    """Exactly one of I_w or wheel_radius_m, never both, never neither."""
    have_iw = "I_w" in values and not math.isnan(values["I_w"])
    have_r = "wheel_radius_m" in values and not math.isnan(values["wheel_radius_m"])

    if have_iw and have_r:
        sys.exit(
            "Both I_w and wheel_radius_m are set.  Set exactly one.\n"
            "Preferring one silently would hide a typo in the other, and the "
            "two disagreeing is a real possibility -- a spoked wheel's inertia "
            "is nowhere near the solid-disc formula."
        )
    if have_iw:
        return values["I_w"], "given directly"
    if have_r:
        r = values["wheel_radius_m"]
        return 0.5 * values["m_w"] * r * r, f"solid disc, r = {r:.4f} m"
    sys.exit(
        "Neither I_w nor wheel_radius_m is set -- see section 4 of "
        "measurements.md.\n"
        "Set I_w directly for a spoked/rimmed wheel or a CAD value, or set "
        "wheel_radius_m to use the solid-disc formula."
    )


def body_inertia_from_swing(m_b, m_w, l, l_w, T):
    """I_b about the pivot edge, from the swing test in section 3.

    The cube is swung with the wheel MOUNTED (README.md section 3), so the
    period reflects the whole assembly; the wheel's own contribution is
    subtracted afterwards.  l_total is the CoM of the assembly, which is the
    mass-weighted combination of the body's CoM and the wheel axis -- not l,
    and using l here is the easy mistake.
    """
    m_total = m_b + m_w
    l_total = (m_b * l + m_w * l_w) / m_total
    i_pivot = (m_total * G * l_total * T * T) / (4.0 * math.pi ** 2)
    return i_pivot - m_w * l_w * l_w, l_total, i_pivot


def solve(values, q_diag, r_value):
    m_b = values["m_b"]
    m_w = values["m_w"]
    l = values["l"]
    l_w = values["l_w"]
    T = values["T_swing"]

    i_w, i_w_source = resolve_wheel_inertia(values)
    i_b, l_total, i_pivot = body_inertia_from_swing(m_b, m_w, l, l_w, T)

    if not (i_w > 0.0):
        sys.exit(f"I_w came out {i_w:.6g}, which is not physical.")
    if not (i_b > 0.0):
        sys.exit(
            f"I_b came out {i_b:.6g} kg m^2, which is not physical.\n"
            "The swing test subtracts the wheel's contribution "
            f"(m_w * l_w^2 = {m_w * l_w * l_w:.6g}) from the measured "
            f"I_pivot = {i_pivot:.6g}.  A negative result means the measured "
            "period is too short: check the amplitude was small (< ~10 deg) "
            "and that T is the period, not the time for 20 swings."
        )

    m_total = m_b + m_w
    i_total = i_b + m_w * l_w ** 2

    # README.md section 4.  State x = [theta, theta_dot, omega_w], u = tau.
    # The positive (2,1) entry is the open-loop instability -- the cube falls.
    a = np.array([
        [0.0, 1.0, 0.0],
        [m_total * G * l / i_total, 0.0, 0.0],
        [0.0, 0.0, 0.0],
    ])
    b = np.array([[0.0], [-1.0 / i_total], [1.0 / i_w + 1.0 / i_total]])

    q = np.diag(q_diag)
    r = np.array([[r_value]])

    p = solve_continuous_are(a, b, q, r)
    k = np.linalg.inv(r) @ b.T @ p

    derived = {
        "m_total": m_total,
        "l_total": l_total,
        "I_pivot": i_pivot,
        "I_b": i_b,
        "I_w": i_w,
        "I_w_source": i_w_source,
        "I_total": i_total,
    }
    return k, a, b, derived


def main():
    parser = argparse.ArgumentParser(
        description="Solve the 1D balancing LQR from measurements.md.")
    parser.add_argument(
        "--measurements", type=pathlib.Path, default=DEFAULT_MEASUREMENTS,
        help="path to measurements.md (default: docs/3d_scaling/measurements.md)")
    parser.add_argument(
        "--q", type=float, nargs=3, default=[100.0, 1.0, 0.01],
        metavar=("Q_THETA", "Q_RATE", "Q_WHEEL"),
        help="diagonal of Q.  Raising Q_THETA stiffens the angle response; "
             "raising Q_WHEEL makes the cube work harder to keep the wheel "
             "near zero.  Default: 100 1 0.01")
    parser.add_argument(
        "--r", type=float, default=1.0,
        help="control effort penalty.  Raising it makes the whole thing "
             "gentler.  Default: 1.0")
    args = parser.parse_args()

    values = parse_measurements(args.measurements)

    unset = first_unset(values)
    if unset is not None:
        print(f"REFUSING TO SOLVE: {unset} is still nan.", file=sys.stderr)
        print(file=sys.stderr)
        print(f"  Measure it and write it into {args.measurements}.",
              file=sys.stderr)
        print("  The procedures are in docs/3d_scaling/README.md section 3.",
              file=sys.stderr)
        print(file=sys.stderr)
        print("  A nan here does not raise inside the Riccati solve -- it",
              file=sys.stderr)
        print("  propagates, and a nan GAIN disables every safety comparison",
              file=sys.stderr)
        print("  downstream, because every comparison against nan is false.",
              file=sys.stderr)
        return 1

    k, a, b, d = solve(values, args.q, args.r)
    k_theta, k_theta_dot, k_omega = k[0, 0], k[0, 1], k[0, 2]

    print("=== measured ===")
    for key in REQUIRED_KEYS:
        print(f"  {key:<16} {values[key]:.6g}")
    print()
    print("=== derived ===")
    print(f"  m_total          {d['m_total']:.6g} kg")
    print(f"  l_total          {d['l_total']:.6g} m   (assembly CoM, wheel on)")
    print(f"  I_pivot          {d['I_pivot']:.6g} kg m^2  (from the swing test)")
    print(f"  I_b              {d['I_b']:.6g} kg m^2  (I_pivot - m_w*l_w^2)")
    print(f"  I_w              {d['I_w']:.6g} kg m^2  ({d['I_w_source']})")
    print(f"  I_total          {d['I_total']:.6g} kg m^2")
    print(f"  K_t              {K_T:.6g} Nm/A  (kv = {MOTOR_KV})")
    print()

    # --- the checks README.md section 4 lists as prose ---------------------
    eig = np.linalg.eigvals(a - b @ k)
    stable = bool(np.all(eig.real < 0.0))

    print("=== checks ===")
    print(f"  closed-loop eigenvalues: "
          f"{', '.join(f'{v.real:.4g}{v.imag:+.4g}j' for v in eig)}")
    print(f"  {'PASS' if stable else 'FAIL'}  all have negative real parts")

    # Magnitude checks only -- see the sign discussion below.  Advisory, not
    # fatal: they are shape expectations from README.md section 4, and an
    # unusual rig could legitimately violate them.
    advisories = []
    if not abs(k_theta) > 5.0 * abs(k_theta_dot):
        advisories.append(
            f"|k_theta| ({abs(k_theta):.6g}) is not comfortably larger than "
            f"|k_theta_dot| ({abs(k_theta_dot):.6g}).  Expect roughly an "
            "order of magnitude between them.")
    if abs(k_omega) > 0.01 * abs(k_theta):
        advisories.append(
            f"|k_omega| ({abs(k_omega):.6g}) is large relative to |k_theta| "
            f"({abs(k_theta):.6g}); section 4 expects roughly a thousandth.  "
            "Too large and the cube visibly leans away from vertical to "
            "bleed wheel speed.")
    for note in advisories:
        print(f"  NOTE  {note}")
    if not advisories:
        print("  PASS  gain magnitudes have the expected shape")
    print()

    # --- the sign, which this script CANNOT settle -------------------------
    #
    # The three signs above are the ones implied by README.md section 4's
    # B matrix, which has B[1] = -1/I_total.  That minus sign is Newton's
    # third law: the motor torque acts on the WHEEL, so the body sees its
    # reaction.  With BalancingController computing
    #
    #     tau = -(k_theta*theta + k_theta_dot*theta_dot + k_omega*omega)
    #
    # a stabilising k_theta for that plant comes out NEGATIVE, which is the
    # opposite of what BalancingController.hpp's per-field comment expects.
    # Both cannot be right, and this script deliberately does not pick:
    #
    # THE SIGN IS NOT DECIDABLE FROM THE MODEL.  What a positive
    # feedforward_torque physically does to the cube depends on the motor
    # phase order and SOURCE0_SIGN -- wiring and board configuration that no
    # amount of algebra here can see.  The model fixes the MAGNITUDES; N1
    # fixes the sign, by tilting the rig with the wheel off and watching
    # which way the shaft pushes.
    #
    # And when it is wrong, the fix is EST_INVERT_THETA (or SOURCE0_SIGN) --
    # NEVER negating the control law.  Flipping the law leaves the damping
    # term pointing the wrong way and hides the real bug somewhere worse.
    print("=== the sign ===")
    print("  These signs follow README.md section 4's B matrix, whose")
    print("  B[1] = -1/I_total is the wheel's reaction on the body.")
    print()
    print("  THE MODEL CANNOT SETTLE THE SIGN.  What a positive commanded")
    print("  torque physically does depends on the motor phase order and")
    print("  SOURCE0_SIGN.  N1 settles it: wheel OFF, tilt by hand, watch")
    print("  which way the shaft pushes.")
    print()
    print("  If it pushes the wrong way, change EST_INVERT_THETA -- never")
    print("  the sign of the control law.  Negating the law leaves the")
    print("  damping term pointing the wrong way and buries the real bug.")
    print()
    print("  Note BalancingController.hpp's k_theta comment says to expect a")
    print("  POSITIVE value, which is the opposite of what this plant gives.")
    print("  Treat both as unverified until N1, and record what N1 showed.")
    print()

    if not stable:
        print("REFUSING TO PRINT FLAGS: the closed loop is not stable.",
              file=sys.stderr)
        print(file=sys.stderr)
        print("  These gains would not balance the cube; they would drive it.",
              file=sys.stderr)
        print("  Check the measurements first -- a sign or a decimal place in",
              file=sys.stderr)
        print("  l, I_b or I_w is far more likely than an ill-posed problem.",
              file=sys.stderr)
        print("  Then try a larger --r, or a smaller --q.", file=sys.stderr)
        return 1

    print("=== paste into [env:cube_balancer] in platformio.ini ===")
    print(f"    -D CTL_K_THETA={k_theta:.6g}")
    print(f"    -D CTL_K_THETA_DOT={k_theta_dot:.6g}")
    print(f"    -D CTL_K_OMEGA={k_omega:.6g}")
    print()
    print("The remaining CTL_* flags are the safety envelope, not the solve:")
    print("  CTL_MAX_TILT_RAD       the angle past which the wheel cannot")
    print("                         recover the cube.  0.2618 (15 deg) is a")
    print("                         conservative start.")
    print(f"  CTL_MAX_TORQUE_NM      at or below K_t * servo.max_current_A.")
    print(f"                         At {K_T:.4g} Nm/A: 8 A -> "
          f"{K_T * 8:.3g} Nm, 15 A -> {K_T * 15:.3g} Nm.")
    print("                         The firmware reads the board's actual")
    print("                         setting at boot and refuses if this")
    print("                         exceeds it.")
    print("  CTL_MAX_WHEEL_OMEGA    well under no-load speed (~780 rad/s).")
    print("  CTL_WHEEL_TAPER_START  10-15% below CTL_MAX_WHEEL_OMEGA.")
    print("  CTL_THETA_DEADBAND     leave 0.0 unless chatter is demonstrated.")
    print("  CTL_MAX_INVALID_CYCLES 20 (50 ms at the 400 Hz loop rate).")
    print()
    print("Then record what you pasted in the table at the bottom of")
    print(f"{args.measurements}, and start the rig at gain_scale 0.1.")
    print()
    print("A solve is not a validation.  These come from a linearised model")
    print("of a rig measured with a kitchen scale -- the right order of")
    print("magnitude, and nothing more.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
