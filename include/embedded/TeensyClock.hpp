// Monotonic microsecond clock for the Teensy, safe past 71 minutes.
//
// THE PROBLEM
//
// Arduino's micros() returns uint32_t, which wraps every 2^32 us -- about
// 71.6 minutes.  Code that does
//
//     const uint32_t dt = micros() - last;      // fine, unsigned wraps
//     const double seconds = dt * 1e-6;
//
// happens to survive the wrap because unsigned subtraction wraps too.  But
// the moment a timestamp is widened to double, or compared, or stored as a
// signed type, the wrap becomes a 71-minute jump BACKWARDS.
//
// In this project that lands in two places, both bad:
//
//   * dt goes negative.  StateEstimator's complementary filter computes
//     alpha = tau / (tau + dt).  With dt negative and near -tau, alpha
//     explodes; the filter output diverges and the balancer commands
//     whatever falls out of it.  The estimator rejects dt <= 0, so the
//     immediate effect is a stalled estimate rather than a wild one -- but
//     the loop stops estimating until the clock catches up, which is 71
//     minutes.
//
//   * ImuData::timestamp goes backwards, so any log built from it is
//     unusable exactly once per wrap.
//
// This is INTEGRATION.md risk #4, and it is not hypothetical: a balancing
// rig left running on the bench crosses 71 minutes routinely.  It is caught
// by the 90-minute soak test in check S2f, which exists solely to run past
// one wrap.
//
// THE FIX
//
// Track the previous 32-bit reading and accumulate a 64-bit total.  Every
// time the raw value goes down instead of up, a wrap happened; add 2^32.
// At 64 bits the next wrap is in roughly 584,000 years.
//
// USAGE CONSTRAINT: micros64() must be called at least once per wrap
// period to detect it.  A 200 Hz control loop calls it 858,000 times per
// wrap, so this is only a caveat for code that sleeps for over an hour.
#pragma once

#include <Arduino.h>

#include <cstdint>

#include "core/Types.hpp"

namespace cube {

class TeensyClock {
 public:
  // Microseconds since first use, monotonic and wrap-free.
  //
  // Not interrupt-safe: state_ is read-modify-written without a critical
  // section.  The control loop calls this from the main context only.  If
  // an ISR ever needs time, give it its own instance rather than sharing
  // this one.
  std::uint64_t micros64() {
    const std::uint32_t now = ::micros();
    // Unsigned comparison is the whole detector: the counter only ever
    // counts up, so a value below its predecessor means it wrapped.
    if (now < last_raw_) { high_ += (std::uint64_t{1} << 32); }
    last_raw_ = now;
    return high_ + now;
  }

  // Same instant in seconds, the unit core/ works in.
  //
  // double has 53 bits of mantissa, so microsecond resolution survives to
  // 2^53 us -- about 285 years.  The 64-bit accumulator is what matters;
  // this conversion is not the limiting factor.
  Seconds seconds() { return static_cast<Seconds>(micros64()) * 1e-6; }

 private:
  std::uint64_t high_ = 0;
  std::uint32_t last_raw_ = 0;
};

}  // namespace cube
