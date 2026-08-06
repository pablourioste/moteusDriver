cmake_minimum_required(VERSION 3.10)
project(moteusDriver CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Emit compile_commands.json so editors/IntelliSense resolve includes
# the same way the real build does.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# The moteus C++ library is header-only; it just needs to be on the
# include path.  Headers live in the upstream checkout under moteus/.
add_library(moteus_lib INTERFACE)
target_include_directories(moteus_lib
  INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/moteus/lib/cpp/mjbots/moteus)

find_package(Threads REQUIRED)

# ---------------------------------------------------------------------------
# Controller port configuration
#
# These describe the aux2 port on the moteus board itself.  They are baked
# into current_config.cfg and pushed to the controller at startup with
# `conf set`.  Override any of them at configure time, e.g.
#
#   cmake -S . -B build -DAUX2_PIN0_MODE=13 -DAUX2_PIN1_MODE=13
#
# This board exposes aux2 only -- `moteus_tool --dump-config` reports no
# aux1.* keys whatsoever -- so there is nothing here for aux1.
#
# Every peripheral is given an explicit value, including the disabled ones.
# A peripheral left at a stale value can claim pins another one needs, which
# fails port validation: `aux2.uart.mode 5` held the UART pins while SPI
# wanted them, giving `error.uart_pin_error` and `spi.active False`.
#
# Pin mode values (aux2.pins.X.mode):
#   0 NC   1 SPI   2 SPI_CS   3 UART   4 SW_QUAD   5 HW_QUAD   6 HALL
#   7 INDEX   8 SINE   9 COSINE   13 I2C   14 DIN   15 DOUT
#   16 AIN   17 PWM_OUT
# Pull values (aux2.pins.X.pull):
#   0 none   1 pull-up   2 pull-down
# ---------------------------------------------------------------------------

set(MOTEUS_CONTROLLER_ID 1 CACHE STRING "CAN ID of the controller to drive")

# aux2 is wired to an off-axis MA600 breakout: pins 0-2 carry CLK/MISO/MOSI
# and pin 3 is chip select.  Pin 4 is unused.
set(AUX2_PIN0_MODE 1 CACHE STRING "aux2.pins.0.mode")
set(AUX2_PIN1_MODE 1 CACHE STRING "aux2.pins.1.mode")
set(AUX2_PIN2_MODE 1 CACHE STRING "aux2.pins.2.mode")
set(AUX2_PIN3_MODE 2 CACHE STRING "aux2.pins.3.mode")
set(AUX2_PIN4_MODE 0 CACHE STRING "aux2.pins.4.mode")
foreach(pin 0 1 2 3 4)
  set(AUX2_PIN${pin}_PULL 0 CACHE STRING "aux2.pins.${pin}.pull")
endforeach()

# SPI.  Mode 5 is the MA600; it will not hold the 12MHz default reliably
# over the breakout cable, so run the bus at 6MHz.
set(AUX2_SPI_MODE 5 CACHE STRING "aux2.spi.mode")
set(AUX2_SPI_RATE_HZ 6000000 CACHE STRING "aux2.spi.rate_hz")
set(AUX2_SPI_FILTER_US 64 CACHE STRING "aux2.spi.filter_us")
# bct/trim are the MA600 bias-current trim, set during provisioning.
set(AUX2_SPI_BCT 0 CACHE STRING "aux2.spi.bct")
set(AUX2_SPI_TRIM 0 CACHE STRING "aux2.spi.trim")

# UART.  Must stay 0 while SPI owns the pins -- this is the setting that
# caused the uart_pin_error.
set(AUX2_UART_MODE 0 CACHE STRING "aux2.uart.mode")
set(AUX2_UART_BAUD_RATE 921600 CACHE STRING "aux2.uart.baud_rate")
set(AUX2_UART_POLL_RATE_US 100 CACHE STRING "aux2.uart.poll_rate_us")
set(AUX2_UART_CUI_AMT21_ADDRESS 84 CACHE STRING "aux2.uart.cui_amt21_address")
set(AUX2_RS422 0 CACHE STRING "aux2.rs422")

# I2C.  This firmware has no aux2.i2c.i2c_hz or .pullup key; rate is set
# per device.  All three device slots are disabled (type 0) while SPI runs.
set(AUX2_I2C_STARTUP_DELAY_MS 30 CACHE STRING "aux2.i2c_startup_delay_ms")
foreach(dev 0 1 2)
  set(AUX2_I2C_DEV${dev}_TYPE 0 CACHE STRING "aux2.i2c.devices.${dev}.type")
  set(AUX2_I2C_DEV${dev}_ADDRESS 64 CACHE STRING "aux2.i2c.devices.${dev}.address")
  set(AUX2_I2C_DEV${dev}_POLL_RATE_US 1000
      CACHE STRING "aux2.i2c.devices.${dev}.poll_rate_us")
endforeach()

# Remaining peripherals, all disabled but set explicitly so none of them
# can silently claim a pin that SPI needs.
set(AUX2_QUADRATURE_ENABLED 0 CACHE STRING "aux2.quadrature.enabled")
set(AUX2_QUADRATURE_CPR 16384 CACHE STRING "aux2.quadrature.cpr")
set(AUX2_HALL_ENABLED 0 CACHE STRING "aux2.hall.enabled")
set(AUX2_HALL_POLARITY 0 CACHE STRING "aux2.hall.polarity")
set(AUX2_INDEX_ENABLED 0 CACHE STRING "aux2.index.enabled")
set(AUX2_INDEX_INVERT 0 CACHE STRING "aux2.index.invert")
set(AUX2_SINE_COSINE_ENABLED 0 CACHE STRING "aux2.sine_cosine.enabled")
set(AUX2_SINE_COSINE_COMMON 1700 CACHE STRING "aux2.sine_cosine.common")
set(AUX2_PWM_INPUT_ENABLED 0 CACHE STRING "aux2.pwm_input.enabled")
set(AUX2_PWM_PERIOD_US 1000 CACHE STRING "aux2.pwm_period_us")
set(AUX2_BISSC_ENABLED 0 CACHE STRING "aux2.bissc.enabled")
set(AUX2_BISSC_RATE_HZ 1000000 CACHE STRING "aux2.bissc.rate_hz")
set(AUX2_BISSC_DATA_BITS 20 CACHE STRING "aux2.bissc.data_bits")
set(AUX2_BISSC_CRC_BITS 6 CACHE STRING "aux2.bissc.crc_bits")
set(AUX2_BISSC_POLL_RATE_US 100 CACHE STRING "aux2.bissc.poll_rate_us")

# ---------------------------------------------------------------------------
# Encoder source
#
# The pin settings above only wire up the bus; these tell the firmware to
# actually read position from it.  Source 0 is the MA600 on aux2.
#
#   type:       0 disabled  1 SPI  2 UART  3 quadrature  4 hall  5 index
#               6 sine/cosine  7 I2C
#   reference:  0 relative to the rotor   1 relative to the output
# ---------------------------------------------------------------------------
set(SOURCE0_AUX_NUMBER 2 CACHE STRING "motor_position.sources.0.aux_number")
set(SOURCE0_TYPE 1 CACHE STRING "motor_position.sources.0.type")
set(SOURCE0_CPR 65536 CACHE STRING "motor_position.sources.0.cpr")
set(SOURCE0_OFFSET 0 CACHE STRING "motor_position.sources.0.offset")
set(SOURCE0_SIGN 1 CACHE STRING "motor_position.sources.0.sign")
set(SOURCE0_REFERENCE 0 CACHE STRING "motor_position.sources.0.reference")
# 0 disables the filter, which would leave this source with no velocity
# reading; the MA600 does not measure velocity natively.
set(SOURCE0_PLL_FILTER_HZ 400 CACHE STRING "motor_position.sources.0.pll_filter_hz")

# Which source drives commutation and which reports output position.  Both
# index into the source list above.
set(COMMUTATION_SOURCE 0 CACHE STRING "motor_position.commutation_source")
set(OUTPUT_SOURCE 0 CACHE STRING "motor_position.output.source")

# ---------------------------------------------------------------------------
# Gearbox and safety limits
#
# These are the first-time-setup parameters from the upstream configuration
# guide.  They are unrelated to the encoder wiring above, but a bare board
# ships without them, and the defaults below are deliberately conservative:
# they are meant to be raised toward the real machine, not lowered.
# ---------------------------------------------------------------------------

# Output turns per rotor turn.  1.0 is direct drive; a 4x reducer is 0.25.
# Anything > 1.0 faults unless rotor_to_output_override is also set.
set(ROTOR_TO_OUTPUT_RATIO 1.0 CACHE STRING
    "motor_position.rotor_to_output_ratio (reducer < 1.0)")

# Travel limits in output rotations.  nan disables them.
#
# nan is correct for THIS rig: the load is a reaction wheel, which must spin
# freely through unbounded revolutions.  The former +/-1.0 default stopped the
# wheel dead at exactly one turn -- `moteus_driver --velocity 2.0` cruised to
# position 1.0, braked hard, and reported fault 103 (kLimitPositionBounds).
# Worse, it then coasted to 1.004, i.e. OUTSIDE the bound, so the very next
# position command failed up front with fault 39 (kStartOutsideLimit) and the
# board had to be un-faulted with `d stop` before it would move at all.
#
# Safe here specifically because every mode this project commands is torque or
# velocity -- position = NaN, so there is no position target for servopos to
# bound.  The limits that matter are BalancingController's tilt e-stop and
# torque clamp, plus servo.max_current_A below.  Restore finite values if a
# bounded-travel joint is ever driven from this build.
set(SERVOPOS_POSITION_MIN nan CACHE STRING "servopos.position_min (nan = none)")
set(SERVOPOS_POSITION_MAX nan CACHE STRING "servopos.position_max (nan = none)")

# Torque ceiling.  There is no safe generic default, so this starts low and
# must be raised deliberately.  Use the LOWER of the motor's rated phase
# current and the controller's own rating -- exceeding the factory value
# risks hardware damage.
set(SERVO_MAX_CURRENT_A 15.0 CACHE STRING "servo.max_current_A (set from your motor)")

# Backstop speed at which output power is derated.  NOT a trajectory limit --
# raising it does not make anything spin faster, it only moves the point where
# the board starts cutting power.  The trajectory limit is
# SERVO_DEFAULT_VELOCITY_LIMIT below.
#
# 100 rev/s is deliberately above what this rig can physically reach, so the
# derating never interferes: calibration measured kv = 344.6 RPM/V, which on a
# 20 V bus gives a no-load ceiling of 344.6 * 20 = 6892 RPM = 114.9 rev/s.
# Under load, back-EMF and the 8 A current limit put the real ceiling well
# below that.  The backstop sits just under the no-load figure on purpose.
set(SERVO_MAX_VELOCITY 100.0 CACHE STRING "servo.max_velocity (output rev/s)")

# Trajectory limits for reaching a target.  With both unset -- the bare-board
# default -- velocity changes instantaneously and can grow without bound, so
# the first position command slams to target.  These make it ramp.
set(SERVO_DEFAULT_VELOCITY_LIMIT 2.0 CACHE STRING
    "servo.default_velocity_limit (output rev/s, nan = unset)")
set(SERVO_DEFAULT_ACCEL_LIMIT 10.0 CACHE STRING
    "servo.default_accel_limit (output rev/s^2, nan = unset)")

# A reducing gearbox is < 1.0; larger values fault on the board unless
# rotor_to_output_override is set.  Catch the typo here instead.
if(ROTOR_TO_OUTPUT_RATIO GREATER 1.0)
  message(FATAL_ERROR
    "ROTOR_TO_OUTPUT_RATIO=${ROTOR_TO_OUTPUT_RATIO} is greater than 1.0, "
    "which faults the controller unless motor_position.rotor_to_output_override "
    "is set.  A reducing gearbox is entered as a value below 1.0 -- a 4x "
    "reduction is 0.25, not 4.")
endif()

option(APPLY_CONFIG_ON_STARTUP "Push the port configuration to the controller" ON)
option(PERSIST_CONFIG_TO_FLASH "Follow the push with `conf write`" OFF)

# UART and SPI cannot both hold the aux2 pins.  Catch the combination here
# rather than letting the board come up with a dead port.
if(NOT AUX2_UART_MODE EQUAL 0 AND NOT AUX2_SPI_MODE EQUAL 1)
  message(FATAL_ERROR
    "AUX2_UART_MODE=${AUX2_UART_MODE} conflicts with AUX2_SPI_MODE="
    "${AUX2_SPI_MODE}: the port cannot run UART and SPI at once.  Set "
    "AUX2_UART_MODE=0 to use SPI, or AUX2_SPI_MODE=1 to use UART.")
endif()

# The .cfg lands next to the binary so `./build/moteus_driver` finds it with
# the default relative path.
set(CURRENT_CONFIG_CFG ${CMAKE_CURRENT_BINARY_DIR}/current_config.cfg)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/config/current_config.cfg.in
  ${CURRENT_CONFIG_CFG}
  @ONLY)

# Runtime behaviour of the driver's control loop.  The velocity is only the
# default -- `--velocity <rev/s>` overrides it per run without a rebuild.
set(TARGET_VELOCITY_REV_S 1.0 CACHE STRING
    "default commanded speed in output rev/s (1.0 = 60 RPM)")
set(MAX_TORQUE_NM 0.5 CACHE STRING
    "torque ceiling applied to each velocity command, Nm")
# Must comfortably exceed the loop period: at 100 Hz a 0.1 s watchdog leaves
# 10 cycles of margin, so ordinary scheduling jitter cannot trip it.
set(COMMAND_WATCHDOG_S 0.1 CACHE STRING
    "per-command watchdog in seconds; board stops if the host goes quiet")
set(LOOP_PERIOD_US 10000 CACHE STRING
    "control loop period in microseconds (10000 = 100 Hz)")

# ---------------------------------------------------------------------------
# Balancing cube layer
#
# The tree splits in two:
#
#   include/core, src/core        platform-agnostic control code.  Must not
#                                 include a driver or vendor header, so the
#                                 control law stays testable without hardware.
#   include/drivers, src/drivers  hardware implementations of the core's
#                                 interfaces (moteus over CAN, BMI270 over I2C).
#
# Applications depend on both; nothing in core/ depends on drivers/.
# ---------------------------------------------------------------------------

# IMU I2C defaults.  Overridable per run with --i2c, so these only set what
# the executables print as their default.
set(IMU_I2C_DEVICE "/dev/i2c-1" CACHE STRING
    "I2C device node the BMI270 is on")
set(IMU_I2C_ADDRESS 0x68 CACHE STRING
    "BMI270 I2C address (0x68 with SDO low, 0x69 with SDO high)")

# IMU calibration, measured per physical part -- never guessed.
#
# Produced by firmware/imu/imu_calibrate: menu [g] for the gyro biases, and
# [1]..[6] then [f] for the six-position accel fit.  That sketch prints them
# as PlatformIO build flags for the Teensy build; these cache variables are
# the same numbers for the host build, so both targets correct the same
# sensor identically.  Keep them in step -- a divergence means the host
# simulation and the rig disagree about which way gravity points.
#
# Values below are the measured calibration of THIS sensor, from
# data/imu_calibration/imu-cal-20260805T125013.log (bench, 23.5 C).
# They are not identity: replacing this board means re-running
# imu_calibrate and updating both here and platformio.ini.
set(IMU_GYRO_BIAS_X 0.002348 CACHE STRING "Gyro X zero-rate offset, rad/s")
set(IMU_GYRO_BIAS_Y -0.001140 CACHE STRING "Gyro Y zero-rate offset, rad/s")
set(IMU_GYRO_BIAS_Z -0.000762 CACHE STRING "Gyro Z zero-rate offset, rad/s")

# The Y offset is ~2% of g.  That is a real sensor zero error, not a
# tilted capture: tilt can only ever REDUCE a magnitude (it scales by
# cos theta), yet -Y read 9.9152 and +Z read 9.8945, both ABOVE g.
# The pairs are shifted, not shrunk.  See the log's README.
set(IMU_ACCEL_OFFSET_X -0.092282 CACHE STRING "Accel X offset, m/s^2")
set(IMU_ACCEL_OFFSET_Y -0.196510 CACHE STRING "Accel Y offset, m/s^2")
set(IMU_ACCEL_OFFSET_Z 0.050664 CACHE STRING "Accel Z offset, m/s^2")

# Gain corrections, near 1.0.  A value far from 1.0 is a transcription
# error or a fit taken at the wrong range, and ImuDriver::initialize()
# refuses to start on one rather than dividing by it.
set(IMU_ACCEL_SCALE_X 0.991085 CACHE STRING "Accel X gain correction (~1.0)")
set(IMU_ACCEL_SCALE_Y 0.991035 CACHE STRING "Accel Y gain correction (~1.0)")
set(IMU_ACCEL_SCALE_Z 1.003787 CACHE STRING "Accel Z gain correction (~1.0)")

# The BMI270 keeps its accelerometer and gyroscope disabled until Bosch's
# ~8 KB config blob is uploaded, so the IMU driver cannot produce data without
# it.  third_party/bmi270_config.h holds that blob, extracted from the
# Apache-2.0 BMI270-Sensor-API; docs/3d_scaling/README.md records how.
#
# Defaults ON when the file is present, so a fresh clone that is missing it
# still configures and builds rather than failing at the include.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/third_party/bmi270_config.h)
  set(BMI270_BLOB_DEFAULT ON)
else()
  set(BMI270_BLOB_DEFAULT OFF)
endif()
option(BMI270_CONFIG_HEADER
       "third_party/bmi270_config.h is present and holds bmi270_config_file[]"
       ${BMI270_BLOB_DEFAULT})

# Core control library.  No hardware dependencies at all.
#
# TelemetryFrame.cpp is deliberately here rather than under drivers/: the
# wire format is compiled by BOTH build systems (this one for the host,
# PlatformIO for the Teensy) and the static_assert on sizeof is what proves
# the two toolchains agree about the layout.
add_library(cube_core STATIC
  src/core/StateEstimator.cpp
  src/core/BalancingController.cpp
  src/core/TelemetryFrame.cpp)
target_compile_options(cube_core PRIVATE -Wall -Wextra)
target_include_directories(cube_core
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Hardware drivers.  Links moteus_lib for the CAN side; the IMU side needs
# nothing beyond libc's ioctl.
add_library(cube_drivers STATIC
  src/drivers/MoteusDriverWrapper.cpp
  src/drivers/ImuDriver.cpp)
target_compile_options(cube_drivers PRIVATE -Wall -Wextra)
target_include_directories(cube_drivers
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(cube_drivers PUBLIC cube_core moteus_lib Threads::Threads)
target_compile_definitions(cube_drivers PRIVATE
  DEFAULT_CONFIG_PATH="${CURRENT_CONFIG_CFG}"
  DEFAULT_CONTROLLER_ID=${MOTEUS_CONTROLLER_ID}
  MAX_TORQUE_NM=${MAX_TORQUE_NM}
  COMMAND_WATCHDOG_S=${COMMAND_WATCHDOG_S}
  ROTOR_TO_OUTPUT_RATIO_DEF=${ROTOR_TO_OUTPUT_RATIO}
  APPLY_CONFIG_ON_STARTUP=$<BOOL:${APPLY_CONFIG_ON_STARTUP}>
  PERSIST_CONFIG_TO_FLASH=$<BOOL:${PERSIST_CONFIG_TO_FLASH}>
  # IMU calibration.  Defined on the library rather than on each app so
  # every binary that reads the IMU corrects it the same way -- an app that
  # forgot one of these would silently read an uncalibrated sensor.
  DEFAULT_IMU_GYRO_BIAS_X=${IMU_GYRO_BIAS_X}
  DEFAULT_IMU_GYRO_BIAS_Y=${IMU_GYRO_BIAS_Y}
  DEFAULT_IMU_GYRO_BIAS_Z=${IMU_GYRO_BIAS_Z}
  DEFAULT_IMU_ACCEL_OFFSET_X=${IMU_ACCEL_OFFSET_X}
  DEFAULT_IMU_ACCEL_OFFSET_Y=${IMU_ACCEL_OFFSET_Y}
  DEFAULT_IMU_ACCEL_OFFSET_Z=${IMU_ACCEL_OFFSET_Z}
  DEFAULT_IMU_ACCEL_SCALE_X=${IMU_ACCEL_SCALE_X}
  DEFAULT_IMU_ACCEL_SCALE_Y=${IMU_ACCEL_SCALE_Y}
  DEFAULT_IMU_ACCEL_SCALE_Z=${IMU_ACCEL_SCALE_Z})

if(BMI270_CONFIG_HEADER)
  target_compile_definitions(cube_drivers PRIVATE HAVE_BMI270_CONFIG_BLOB)
  target_include_directories(cube_drivers
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party)
  message(STATUS "BMI270 config blob enabled")
else()
  message(STATUS
    "BMI270 config blob NOT enabled - imu_test and cube_balancer will build "
    "but the IMU will refuse to initialize.  See docs/3d_scaling/README.md.")
endif()

# ---------------------------------------------------------------------------
# Target 1: direct_motor_test -- hardware validation harness
#
# A separate executable from the balancer on purpose: it applies torque and
# deliberately provokes a watchdog timeout, so it must never be something the
# normal control path can wander into.  Run it before trusting the motor in a
# control loop.
#
#   ./build/direct_motor_test              interactive menu
#   ./build/direct_motor_test --run-all    non-interactive, exit code is the
#                                          verdict (0 = all passed)
#   ./build/direct_motor_test --apply-config build/current_config.cfg
#
# It writes logs/motor_test_<timestamp>.csv relative to the working
# directory, so run it from the project root.
# ---------------------------------------------------------------------------
add_executable(direct_motor_test
  apps/direct_motor_test.cpp
  src/testing/MotorValidator.cpp
  src/testing/CsvLogger.cpp)
target_compile_options(direct_motor_test PRIVATE -Wall -Wextra)
target_include_directories(direct_motor_test
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_definitions(direct_motor_test PRIVATE
  DEFAULT_CONTROLLER_ID=${MOTEUS_CONTROLLER_ID})
target_link_libraries(direct_motor_test PRIVATE cube_drivers)

# ---------------------------------------------------------------------------
# Target 2: imu_test -- IMU validation and calibration, no motor involved
#
#   ./build/imu_test                  full check then live tilt stream
#   ./build/imu_test --stream         skip the checks
#   ./build/imu_test --device /dev/i2c-3 --address 69
#
# Run this before cube_balancer: it establishes the gyro bias and confirms
# the tilt sign convention, and a sign error there makes the controller
# drive the cube over rather than catch it.
# ---------------------------------------------------------------------------
add_executable(imu_test apps/imu_test.cpp)
target_compile_options(imu_test PRIVATE -Wall -Wextra)
target_include_directories(imu_test
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_definitions(imu_test PRIVATE
  DEFAULT_IMU_I2C_DEVICE="${IMU_I2C_DEVICE}"
  DEFAULT_IMU_I2C_ADDRESS=${IMU_I2C_ADDRESS})
target_link_libraries(imu_test PRIVATE cube_drivers)

# ---------------------------------------------------------------------------
# Target 3: cube_balancer -- the 200 Hz balancing control loop
#
#   ./build/cube_balancer --dry-run   run the loop, never command torque
#   ./build/cube_balancer             live
#
# The gains in BalancingController are placeholders and will NOT balance the
# cube; they are set low deliberately.  Compute real ones by the procedure in
# docs/3d_scaling/README.md before expecting it to work.
# ---------------------------------------------------------------------------
add_executable(cube_balancer apps/cube_balancer.cpp)
target_compile_options(cube_balancer PRIVATE -Wall -Wextra)
target_include_directories(cube_balancer
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_definitions(cube_balancer PRIVATE
  DEFAULT_IMU_I2C_DEVICE="${IMU_I2C_DEVICE}"
  DEFAULT_IMU_I2C_ADDRESS=${IMU_I2C_ADDRESS})
target_link_libraries(cube_balancer PRIVATE cube_drivers)

# ---------------------------------------------------------------------------
# Legacy single-file driver.  Kept building because it is the reference the
# HAL was extracted from and is still the quickest way to spin the motor at a
# constant speed:  ./build/moteus_driver --velocity 2.0
# ---------------------------------------------------------------------------
add_executable(moteus_driver apps/legacy_moteus_driver.cpp)
target_compile_options(moteus_driver PRIVATE -Wall -Wextra)
target_compile_definitions(moteus_driver PRIVATE
  DEFAULT_CONFIG_PATH="${CURRENT_CONFIG_CFG}"
  DEFAULT_CONTROLLER_ID=${MOTEUS_CONTROLLER_ID}
  TARGET_VELOCITY_REV_S=${TARGET_VELOCITY_REV_S}
  MAX_TORQUE_NM=${MAX_TORQUE_NM}
  COMMAND_WATCHDOG_S=${COMMAND_WATCHDOG_S}
  LOOP_PERIOD_US=${LOOP_PERIOD_US}
  # Rejects a --velocity beyond the derating backstop pushed to the board,
  # rather than letting the board quietly cut power mid-run.
  SERVO_MAX_VELOCITY_GUARD=${SERVO_MAX_VELOCITY}
  APPLY_CONFIG_ON_STARTUP=$<BOOL:${APPLY_CONFIG_ON_STARTUP}>
  PERSIST_CONFIG_TO_FLASH=$<BOOL:${PERSIST_CONFIG_TO_FLASH}>)
target_link_libraries(moteus_driver PRIVATE moteus_lib Threads::Threads)

# Upstream library unit tests.  These need Boost.Test; if it is not
# installed the tests are simply skipped rather than failing the build.
#   sudo apt install libboost-test-dev
enable_testing()

# First-party telemetry tests.  No Boost, no gtest -- plain asserts and an
# exit code, so these always run even where the optional Boost.Test
# dependency below is unavailable.  The telemetry layer is precisely the
# thing that must not go unverified: a bug here silently corrupts every
# log downstream.  Needs no hardware.
add_executable(test_telemetry_ring tests/test_telemetry_ring.cpp)
target_compile_options(test_telemetry_ring PRIVATE -Wall -Wextra)
target_link_libraries(test_telemetry_ring PRIVATE cube_core)
add_test(NAME test_telemetry_ring COMMAND test_telemetry_ring)

find_package(Boost QUIET COMPONENTS unit_test_framework)
if(Boost_FOUND AND TARGET Boost::unit_test_framework)
  set(MOTEUS_TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/moteus/lib/cpp/mjbots/moteus/test)
  add_executable(moteus_test
    ${MOTEUS_TEST_DIR}/test_main.cc
    ${MOTEUS_TEST_DIR}/moteus_test.cc
    ${MOTEUS_TEST_DIR}/moteus_protocol_test.cc)
  # The tests include via "mjbots/moteus/moteus.h", so they need the
  # lib/cpp root on the include path rather than the deeper directory
  # moteus_lib provides.
  target_include_directories(moteus_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/moteus/lib/cpp)
  # test_main.cc defines BOOST_TEST_MODULE, so link the shared runtime.
  target_compile_definitions(moteus_test PRIVATE BOOST_TEST_DYN_LINK)
  # moteus_test.cc uses std::optional without including <optional>; it
  # only compiles upstream because their toolchain pulls it in
  # transitively.  Inject it rather than patching the vendored source.
  target_compile_options(moteus_test PRIVATE -include optional)
  # The upstream tests are not warning-clean; don't inherit -Wall -Wextra
  # noise from a dependency we don't maintain.
  target_compile_options(moteus_test PRIVATE -Wno-unused-result)
  target_link_libraries(moteus_test
    PRIVATE moteus_lib Threads::Threads Boost::unit_test_framework)
  add_test(NAME moteus_test COMMAND moteus_test)
  message(STATUS "Boost.Test found - unit tests enabled")
else()
  message(STATUS
    "Boost.Test not found - skipping unit tests "
    "(install with: sudo apt install libboost-test-dev)")
endif()
