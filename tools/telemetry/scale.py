"""Datasheet scale factors for the BMI270.

Deliberately derived from the DATASHEET, not from the firmware.  The
firmware computes its scale as ``range * unit / 32768``; this module states
the sensitivity in the LSB-per-unit form the datasheet actually prints.
The two must agree, and ``decode.py --check-scale`` asserts that they do by
comparing the raw counts in each frame against the calibrated SI values the
firmware packed beside them.

That cross-check is the point of carrying both in the frame: if someone
changes the range register in firmware and forgets the scale factor, the
mismatch shows up in the very next capture instead of as a quietly wrong
number in a plot six weeks later.

Source: BMI270 datasheet (docs/2D_model/datasheets/IMU_datasheet.pdf),
"Output Signal Accelerometer" and "Output Signal Gyroscope" tables.
"""

# Accelerometer sensitivity, LSB per g, indexed by the ACC_RANGE register.
#   0 = +-2 g, 1 = +-4 g, 2 = +-8 g, 3 = +-16 g
ACCEL_LSB_PER_G = {
    0: 16384.0,
    1: 8192.0,
    2: 4096.0,
    3: 2048.0,
}

# Gyroscope sensitivity, LSB per dps, indexed by the GYR_RANGE register.
#
# NOTE THE INVERTED ORDER: register 0 is the WIDEST range (2000 dps), not
# the narrowest.  Reading this table backwards is a silent 4x or 16x scale
# error that looks entirely plausible in a plot.  The firmware carries the
# same warning at its kGyroRangeReg definition.
GYRO_LSB_PER_DPS = {
    0: 16.384,    # +-2000 dps
    1: 32.768,    # +-1000 dps
    2: 65.536,    # +-500 dps
    3: 131.072,   # +-250 dps
    4: 262.144,   # +-125 dps
}

# Standard gravity, m/s^2.  Matches kGravity in the firmware and in
# src/drivers/ImuDriver.cpp so the two derivations are comparable to the
# last digit.
GRAVITY = 9.80665

DEG_TO_RAD = 0.017453292519943295

# What this rig is actually configured for; see firmware/full_case/telemetry_test.
# Change these together with the firmware registers, never separately.
RIG_ACCEL_RANGE_REG = 0    # +-2 g
RIG_GYRO_RANGE_REG = 2     # +-500 dps

# BMI270 temperature register: value/512 + 23.0 degrees C, with 0x8000 as
# the invalid sentinel.  A reading of exactly 23.0 therefore deserves
# suspicion -- it is what a stuck-at-zero register produces.
TEMP_INVALID_RAW = -32768


def accel_counts_to_ms2(counts, range_reg=RIG_ACCEL_RANGE_REG):
    """Raw accelerometer counts -> m/s^2."""
    return counts * GRAVITY / ACCEL_LSB_PER_G[range_reg]


def gyro_counts_to_rad_s(counts, range_reg=RIG_GYRO_RANGE_REG):
    """Raw gyroscope counts -> rad/s."""
    return counts * DEG_TO_RAD / GYRO_LSB_PER_DPS[range_reg]


def accel_resolution_ms2(range_reg=RIG_ACCEL_RANGE_REG):
    """Smallest representable acceleration step, m/s^2.

    Useful when deciding whether a wobble in a plot is real or is simply
    the sensor's quantisation floor.  At +-2 g this is ~0.6 mm/s^2.
    """
    return GRAVITY / ACCEL_LSB_PER_G[range_reg]


def gyro_resolution_rad_s(range_reg=RIG_GYRO_RANGE_REG):
    """Smallest representable rotation rate step, rad/s.

    At +-500 dps this is ~0.27 mrad/s.  The datasheet's zero-rate offset
    (+-0.5 dps, i.e. ~8.7 mrad/s) is roughly 32x larger, which is why bias
    calibration matters far more than resolution here.
    """
    return DEG_TO_RAD / GYRO_LSB_PER_DPS[range_reg]
