// TeensyMoteusDriver query-path validation -- INTEGRATION.md S4.
//
// Proves the Teensy can address the moteus, build a valid CAN-FD frame,
// and parse the reply -- WITHOUT the motor being able to move.
// sendTorque() is disarmed in this sketch and never armed; the driver
// requires an explicit enableTorque(true) that appears nowhere here.
//
//   pio run -e moteus_driver_test     (WSL) build, flash with Teensy Loader
//
// ---------------------------------------------------------------------
// BEFORE FLASHING
// ---------------------------------------------------------------------
//
//   [ ] S3 (can_listen) passed: frames seen, error counters at 0
//   [ ] H4 done -- REACTION WHEEL PHYSICALLY OFF THE SHAFT
//   [ ] moteus on LOGIC POWER ONLY -- motor supply OFF
//
// The board enumerates on CAN and answers queries on logic power alone.
// Motor power is not needed until N1, and the wheel does not go back on
// until N2.
//
// Wiring is S3's, unchanged: pins 30 (TX) / 31 (RX) through a 3.3 V
// CAN-FD transceiver.  CAN3 is the only FD-capable module on this
// silicon.

#include <Arduino.h>

#include "embedded/TeensyMoteusDriver.hpp"

namespace {

cube::TeensyMoteusDriver driver;

void printState(const cube::MotorState& s) {
  if (!s.valid) {
    Serial.println("  INVALID -- no reply from the controller");
    return;
  }
  Serial.printf("  pos %+9.4f rev   vel %+9.4f rev/s   torque %+7.4f Nm\r\n",
                s.position_rev, s.velocity_rev_s, s.torque_nm);
  Serial.printf("  bus %6.2f V   motor %5.1f C   mode %d   fault %d\r\n",
                s.bus_voltage, s.motor_temperature_c, s.mode, s.fault);
}

// The best check in S4: spin the wheel BY HAND and watch position and
// velocity track.  This proves encoder decode, sign convention and the
// whole parse path with zero stored energy anywhere in the system.
void handSpin() {
  Serial.println();
  Serial.println("--- Hand-spin test ---");
  Serial.println("  Turn the motor shaft BY HAND.  Any key stops.");
  Serial.println("  Expect: position tracks smoothly, velocity signed by");
  Serial.println("  direction.  Note WHICH WAY is positive -- the control");
  Serial.println("  law needs that sign, and getting it wrong drives the");
  Serial.println("  cube INTO its fall at full authority.");
  Serial.println();

  while (!Serial.available()) {
    const cube::MotorState s = driver.query();
    if (s.valid) {
      Serial.printf("  pos %+9.4f   vel %+9.4f   rtt %4lu us\r\n",
                    s.position_rev, s.velocity_rev_s,
                    (unsigned long)driver.lastRoundTripUs());
    } else {
      Serial.println("  query failed");
    }
    delay(100);
  }
  while (Serial.available()) { Serial.read(); }
}

// Sustained-rate check.  The budget is well under 200 us per round trip
// at 5 Mbit; anything over 1 ms means something is retrying at the CAN
// layer, which S3's error counters would not necessarily have shown.
void timingTest(uint32_t seconds) {
  Serial.println();
  Serial.printf("--- %lu s at 200 Hz ---\r\n", (unsigned long)seconds);

  uint32_t worst = 0;
  uint64_t total = 0;
  uint32_t ok = 0;
  uint32_t failed = 0;
  const uint32_t started = millis();

  while (millis() - started < seconds * 1000) {
    const uint32_t cycle_start = micros();
    const cube::MotorState s = driver.query();

    if (s.valid) {
      const uint32_t rtt = driver.lastRoundTripUs();
      if (rtt > worst) { worst = rtt; }
      total += rtt;
      ok++;
    } else {
      failed++;
    }

    // Pace to 200 Hz without drifting: sleep the remainder of the 5 ms
    // budget rather than a fixed delay after variable work.
    const uint32_t elapsed = micros() - cycle_start;
    if (elapsed < 5000) { delayMicroseconds(5000 - elapsed); }
  }

  Serial.printf("  ok=%lu  failed=%lu  timeouts=%lu\r\n",
                (unsigned long)ok, (unsigned long)failed,
                (unsigned long)driver.timeoutCount());
  if (ok > 0) {
    Serial.printf("  round trip: mean %lu us   worst %lu us\r\n",
                  (unsigned long)(total / ok), (unsigned long)worst);
  }

  if (failed > 0) {
    Serial.println("  *** FAIL -- dropped transactions. ***");
  } else if (worst > 1000) {
    Serial.println("  *** worst > 1 ms -- something is retrying. ***");
  } else {
    Serial.println("  PASS");
  }
}

// --- S5: configuration verification ---------------------------------
//
// One expected value, mirrored from the CMake cache variable that
// provisioned it.  Build flags rather than a file: there is no
// filesystem here, and adding SD or LittleFS to hold nine numbers would
// be a lot of machinery for something the compiler can bake in.
//
// The tolerance is fractional because these come back as text and a
// float round trip is not exact.  1e-6 is far tighter than any real
// misconfiguration and far looser than the parse noise.
struct ExpectedSetting {
  const char* name;
  double expected;
};

// The safety-critical subset -- the settings where being silently wrong
// damages the rig rather than merely misbehaving:
//
//   servo.max_current_A     the real torque ceiling.  Too high and the
//                           motor or board can be destroyed.
//   rotor_to_output_ratio   every PID gain is in output units, so a
//                           wrong ratio silently rescales all of them.
//   servo.max_velocity      the power-derating backstop.
//
// Values come from CMakeLists.txt, which is what actually pushed them.
constexpr ExpectedSetting kExpectedConfig[] = {
    {"servo.max_current_A", EXPECT_SERVO_MAX_CURRENT_A},
    {"motor_position.rotor_to_output_ratio", EXPECT_ROTOR_TO_OUTPUT_RATIO},
    {"servo.max_velocity", EXPECT_SERVO_MAX_VELOCITY},
};

// Set once verifyConfig() has run and every setting matched.  The
// balancer will refuse to arm on a false -- this is the same
// refuse-to-run discipline firstUnsetField() applies to the gains,
// extended across the CAN link to the board's own settings.
bool g_config_verified = false;

void verifyConfig() {
  Serial.println();
  Serial.println("--- Config read-back (S5) ---");
  Serial.println("Reads the board's own settings and compares them against");
  Serial.println("what this binary was told to expect.  READ ONLY -- nothing");
  Serial.println("is written; provisioning is the host's job.");
  Serial.println();

  bool all_ok = true;

  for (const auto& setting : kExpectedConfig) {
    double actual = 0.0;
    std::string error;

    Serial.printf("  %-38s ", setting.name);

    if (!driver.readConfigDouble(setting.name, &actual, &error)) {
      Serial.printf("READ FAILED -- %s\r\n", error.c_str());
      all_ok = false;
      continue;
    }

    const double tolerance =
        fabs(setting.expected) * 1e-6 + 1e-9;   // + absolute, for zero
    const bool match = fabs(actual - setting.expected) <= tolerance;

    Serial.printf("%12.4f  expected %12.4f  %s\r\n", actual, setting.expected,
                  match ? "OK" : "*** MISMATCH ***");
    if (!match) { all_ok = false; }
  }

  g_config_verified = all_ok;

  Serial.println();
  if (all_ok) {
    Serial.println("  PASS -- the board is configured as expected.");
    Serial.println();
    Serial.println("  Now run the NEGATIVE test, or this proves nothing:");
    Serial.println("    1. push a deliberately wrong value from the host,");
    Serial.println("       e.g. moteus_tool -t 1 --console then");
    Serial.println("       `conf set servo.max_current_A 3.0`");
    Serial.println("    2. re-run [c] and confirm it reports MISMATCH");
    Serial.println("    3. put the correct value back");
    Serial.println("  An unverified verifier is worse than none.");
  } else {
    Serial.println("  FAIL -- do NOT arm the balancer against this board.");
    Serial.println();
    Serial.println("  Expected values come from CMakeLists.txt.  If the board");
    Serial.println("  is right and the expectation is wrong, rebuild; if the");
    Serial.println("  board is wrong, re-provision from the host with");
    Serial.println("  -DPERSIST_CONFIG_TO_FLASH=ON and power-cycle it.");
  }
}

// =====================================================================
// TORQUE TESTING -- COMPILED IN ONLY WITH -D ENABLE_TORQUE_TEST
// =====================================================================
//
// Everything between here and the matching #endif is absent from the
// default `-e moteus_driver_test` binary.  Not hidden behind a runtime
// prompt -- ABSENT.  The instruction that commands torque does not exist
// in that image, so no sequence of keystrokes, no serial glitch and no
// stuck input can produce motion from it.
//
// Moving the motor therefore requires flashing a different firmware
// (`-e moteus_driver_test_torque`), which is a deliberate physical act
// with the wheel-off checklist in front of you, rather than a keypress.
//
// This is where INTEGRATION.md's N1 checks are actually performed: the
// direction test and, more importantly, the watchdog test.
#if defined(ENABLE_TORQUE_TEST)

// Ceiling for this sketch, well under the driver's own max_torque_nm.
// Enough to turn a bare shaft against its detent so direction is
// unmistakable, far too little to be dangerous.
constexpr double kTestTorqueNm = 0.05;

// Hard stop for any commanded torque.  The board's own watchdog is the
// real backstop; this is the one on this side of the link.
constexpr uint32_t kTorqueDurationMs = 2000;

void torqueStep(double torque_nm) {
  Serial.println();
  Serial.printf("--- Torque step: %+.3f Nm for %lu ms ---\r\n", torque_nm,
                (unsigned long)kTorqueDurationMs);
  Serial.println("  WHEEL MUST BE OFF THE SHAFT.");

  if (!g_config_verified) {
    Serial.println("  REFUSED -- run [c] first and get a PASS.");
    Serial.println("  Commanding torque against an unverified board is how a");
    Serial.println("  wrong servo.max_current_A destroys hardware.");
    return;
  }

  driver.enableTorque(true);

  const uint32_t start = millis();
  uint32_t cycles = 0;
  cube::MotorState last;

  // Command continuously: the board's watchdog_timeout is armed on every
  // frame, so a gap longer than it stops the motor by itself.  That is
  // the behaviour N1's unplug test exercises.
  while (millis() - start < kTorqueDurationMs) {
    last = driver.sendTorque(torque_nm);
    cycles++;
    delay(5);                       // 200 Hz, the real control rate
  }

  // Disarm before reporting, so an exception path or a long print cannot
  // leave the motor commanded.
  driver.sendTorque(0.0);
  driver.stop();
  driver.enableTorque(false);

  Serial.printf("  %lu commands sent\r\n", (unsigned long)cycles);
  printState(last);
  Serial.println("  torque path disarmed again");
}

// N1's most important check, and the one that catches Risk #1.
//
// PositionMode::Format defaults watchdog_timeout to kIgnore.  If the
// driver failed to override that, the frame still transmits cleanly and
// the board still replies OK -- but no watchdog exists, and a dead host
// leaves the motor spinning indefinitely.  It is invisible at the CAN
// layer; the ONLY way to detect it is to stop talking and watch.

void watchdogTest() {
  Serial.println();
  Serial.println("--- Watchdog test (N1) ---");
  Serial.println();
  Serial.println("  This is the check that catches a missing watchdog, which");
  Serial.println("  is invisible at the CAN layer.  Procedure:");
  Serial.println();
  Serial.println("    1. this sketch commands a small torque for 10 s");
  Serial.println("    2. WHILE IT RUNS, unplug the Teensy's USB cable");
  Serial.println("    3. the shaft must stop within ~150 ms");
  Serial.println();
  Serial.println("  If it keeps turning, watchdog_timeout was left kIgnore.");
  Serial.println("  Stop the whole bring-up there -- nothing after this step");
  Serial.println("  is safe until it passes.");
  Serial.println();

  if (!g_config_verified) {
    Serial.println("  REFUSED -- run [c] first and get a PASS.");
    return;
  }

  Serial.println("  starting in 3 s, wheel OFF...");
  delay(3000);

  driver.enableTorque(true);
  const uint32_t start = millis();
  while (millis() - start < 10000) {
    driver.sendTorque(kTestTorqueNm);
    delay(5);
  }
  driver.sendTorque(0.0);
  driver.stop();
  driver.enableTorque(false);

  Serial.println("  10 s elapsed without a disconnect -- torque removed.");
  Serial.println("  Re-run and actually unplug it; this check only counts");
  Serial.println("  when the host really goes away.");
}

#endif  // ENABLE_TORQUE_TEST

void printMenu() {
  Serial.println();
#if defined(ENABLE_TORQUE_TEST)
  Serial.println("=== TeensyMoteusDriver -- TORQUE ENABLED (N1) ===");
#else
  Serial.println("=== TeensyMoteusDriver -- S4/S5, query path only ===");
#endif
  Serial.println("  q  single query");
  Serial.println("  h  hand-spin test      <- the real S4 check");
  Serial.println("  c  verify config       <- the S5 check");
  Serial.println("  t  200 Hz timing, 10 s");
  Serial.println("  T  200 Hz timing, 600 s (the full 10 minute soak)");
  Serial.println("  s  stop (clears faults)");
#if defined(ENABLE_TORQUE_TEST)
  Serial.println("  --- WHEEL OFF -- these move the motor ---");
  Serial.println("  +  torque step, positive");
  Serial.println("  -  torque step, negative");
  Serial.println("  w  watchdog test       <- the N1 check");
#endif
  Serial.println("  ?  this menu");
}

}  // namespace

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Serial.println();
#if defined(ENABLE_TORQUE_TEST)
  Serial.println("*********************************************************");
  Serial.println("***  TORQUE TEST BUILD -- THIS FIRMWARE MOVES THE      ***");
  Serial.println("***  MOTOR.  THE REACTION WHEEL MUST BE OFF THE SHAFT. ***");
  Serial.println("*********************************************************");
  Serial.println();
  Serial.println("Confirm before continuing:");
  Serial.println("  [ ] S4 and S5 passed on the query-only build");
  Serial.println("  [ ] REACTION WHEEL PHYSICALLY REMOVED (H4)");
  Serial.println("  [ ] motor power ON, and you can reach the cut");
  Serial.println("  [ ] nothing in the shaft's arc");
  Serial.printf("  [ ] torque ceiling this build: %.3f Nm\r\n", kTestTorqueNm);
#else
  Serial.println("=== TeensyMoteusDriver -- S4/S5 ===");
  Serial.println();
  Serial.println("Confirm before continuing:");
  Serial.println("  [ ] S3 (can_listen) passed with zero error counters");
  Serial.println("  [ ] REACTION WHEEL IS OFF THE SHAFT (H4)");
  Serial.println("  [ ] moteus on LOGIC POWER ONLY");
#endif
  Serial.println();

  Serial.println("initialising...");
  std::string error;
  if (!driver.initialize(&error)) {
    Serial.print("FAIL -- ");
    Serial.println(error.c_str());
    Serial.println();
    Serial.println("The board answers queries on logic power alone, so this");
    Serial.println("is a link problem, not a power one.  Re-run can_listen.");
    while (true) { delay(1000); }
  }

  Serial.print("OK -- ");
  Serial.println(driver.name().c_str());
  Serial.printf("torque path armed: %s\r\n",
                driver.torqueEnabled() ? "YES" : "no (S4 expects this)");

  const cube::MotorState s = driver.query();
  printState(s);

  if (s.valid && s.fault != 0) {
    Serial.printf("  NOTE fault=%d latched -- press [s] to clear\r\n", s.fault);
  }

  printMenu();
}

void loop() {
  if (!Serial.available()) { return; }

  const char c = Serial.read();
  while (Serial.available()) { Serial.read(); }
  if (c == '\n' || c == '\r') { return; }

  switch (c) {
    case 'q': {
      Serial.println();
      printState(driver.query());
      break;
    }
    case 'h': handSpin(); break;
    case 'c': verifyConfig(); break;
#if defined(ENABLE_TORQUE_TEST)
    case '+': torqueStep(+kTestTorqueNm); break;
    case '-': torqueStep(-kTestTorqueNm); break;
    case 'w': watchdogTest(); break;
#endif
    case 't': timingTest(10); break;
    case 'T': timingTest(600); break;
    case 's':
      Serial.println();
      driver.stop();
      Serial.println("  stop sent");
      printState(driver.query());
      break;
    case '?': printMenu(); break;
    default:
      Serial.print("unknown: ");
      Serial.println(c);
      break;
  }
}
