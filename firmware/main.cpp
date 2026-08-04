// Teensy 4.1 bring-up sketch -- IMU_SETUP.md step 0.4 / 0.5.
//
// Proves the toolchain end to end before any sensor is wired: build,
// flash, run, and read serial back.  Until this passes, a failure at the
// CHIP_ID step is ambiguous -- dead IMU, or code that never ran?
//
// Replace this file as you work through IMU_SETUP.md.

#include <Arduino.h>

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  // Wait for a host to attach, but give up after 3 s so the board still
  // runs standalone on battery with nothing listening.
  while (!Serial && millis() < 3000) {}

  Serial.println("Teensy alive");

  // 600000000 confirms the clock.  sizeof(double) == 8 confirms the
  // VFPv5 double-precision FPU that core/'s control math relies on --
  // if this printed 4, every double would be software-emulated.
  Serial.printf("F_CPU          = %lu Hz\n", (unsigned long)F_CPU);
  Serial.printf("sizeof(double) = %u\n", (unsigned)sizeof(double));
}

void loop() {
  static uint32_t n = 0;

  // Asymmetric blink: short on, long off.  A factory-fresh Teensy blinks
  // evenly at 1 Hz, so a symmetric pattern would not prove YOUR code is
  // running.
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
  delay(800);

  Serial.printf("tick %lu  millis=%lu\n",
                (unsigned long)n++, (unsigned long)millis());
}
