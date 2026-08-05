// moteus over Teensy 4.1 CAN-FD -- the IMotorDriver the control loop uses.
//
// INTEGRATION.md S4.  The bare-metal counterpart of
// drivers/MoteusDriverWrapper, which talks to the same controller from
// Linux through an fdcanusb.  Same interface, same command shape, same
// FromQuery mapping; only the transport differs.
//
// WHAT THIS DELIBERATELY DOES NOT DO
//
// No ThreadedEventLoop.  Upstream's async machinery exists to paper over
// POSIX blocking I/O; on bare metal a polled FlexCAN mailbox is simpler,
// faster and deterministic, and the control loop is already synchronous.
//
// The vendor ENCODING is reused in full, though: moteus_protocol.h
// compiles unmodified for Cortex-M7 (it pulls in nothing heavier than
// <cmath>, <vector> and <limits>), so PositionMode::Make and
// Query::Parse do the register packing here exactly as they do on the
// host.  Hand-rolling the wire format would be a second implementation
// to keep in step with upstream, and the register encoding is the part
// least worth reimplementing.
//
// ---------------------------------------------------------------------
// S4 SCOPE: sendTorque() IS A NO-OP.
// ---------------------------------------------------------------------
//
// This step proves the query path only -- arbitration, bit timing, frame
// building, reply parsing -- while the Teensy is incapable of moving the
// motor.  `enableTorque()` must be called explicitly to arm it, and
// nothing in S4 calls it.  That is the software half of the same
// discipline H4 applies physically by taking the wheel off.
//
// Wiring and the H3 meter checks: see firmware/drivers/can_listen.
// Pins 30/31 are not negotiable -- CAN3 is the only FD-capable module.

#pragma once

#include <cstdint>
#include <string>

#include "core/Types.hpp"
#include "core/interfaces/IMotorDriver.hpp"

namespace cube {

class TeensyMoteusDriver : public IMotorDriver {
 public:
  struct Config {
    // CAN ID of the controller.  Must match MOTEUS_CONTROLLER_ID in
    // CMakeLists.txt -- the host and the Teensy address the same board.
    int controller_id = 1;

    // Torque ceiling this driver enforces, Nm, independent of the
    // board's own servo.max_current_A.  Note the real ceiling is the
    // LOWER of the two: at K_t = 0.0256 Nm/A, 15 A is only ~0.38 Nm, so
    // a larger value here saturates before it is reached.
    double max_torque_nm = 0.5;

    // Board stops itself if it hears nothing for this long.  This is the
    // only thing that stops the wheel if the Teensy hangs or resets --
    // there is no OS to notice, and no SIGINT on bare metal.
    double watchdog_timeout_s = 0.1;

    // How long to wait for a reply before declaring the transaction
    // failed.  At 5 Mbit a round trip is well under 200 us; 2 ms is
    // generous enough to absorb a retry without stalling a 200 Hz loop.
    uint32_t reply_timeout_us = 2000;
  };

  TeensyMoteusDriver();
  explicit TeensyMoteusDriver(const Config& config);

  // Brings up CAN3 at 1 Mbit arbitration / 5 Mbit data and clears any
  // latched fault with a stop.  Does NOT push controller configuration:
  // that is provisioned from the host and persisted to flash (S5),
  // because applyConfig's load-bearing retry sleeps belong on a machine
  // that can afford them.
  bool initialize(std::string* error) override;

  // S4: returns query() without commanding, unless enableTorque(true)
  // has been called.  See the scope note above.
  MotorState sendTorque(double torque_nm) override;

  MotorState query() override;

  // Safe to call repeatedly, and when initialize() failed.
  void stop() override;

  double maxTorqueNm() const override { return config_.max_torque_nm; }
  std::string name() const override { return "moteus/CAN-FD"; }

  // Arms the torque path.  Separate from initialize() so that moving the
  // motor is always an explicit act in the source, greppable in one
  // search, rather than a side effect of constructing a driver.
  void enableTorque(bool enabled) { torque_enabled_ = enabled; }
  bool torqueEnabled() const { return torque_enabled_; }

  // Round-trip time of the last successful transaction, microseconds.
  // S4's check budgets well under 200 us; anything over 1 ms means
  // something is retrying at the CAN layer.
  uint32_t lastRoundTripUs() const { return last_round_trip_us_; }

  // Transactions that got no reply within reply_timeout_us.  Should be 0.
  uint32_t timeoutCount() const { return timeout_count_; }

  // --- S5: configuration read-back ------------------------------------
  //
  // Reads one setting over the diagnostic channel, e.g.
  //
  //   double current = 0.0;
  //   driver.readConfigDouble("servo.max_current_A", &current, &err);
  //
  // READ ONLY, deliberately.  `conf set` from the Teensy is out of scope:
  // provisioning happens once from the host, where applyConfig()'s
  // load-bearing retry sleeps already encode real hardware failures this
  // repo has hit.  Re-earning that knowledge on a new platform is risk
  // for no gain -- and a read cannot brick the board.
  //
  // The point of reading it back is that a silently-wrong
  // servo.max_current_A is rig-destroying.  This extends the same
  // refuse-to-run discipline firstUnsetField() applies to the gains
  // across the CAN link to the board's own settings.
  //
  // Startup-only: the exchange takes several round trips and allocates a
  // fixed buffer on the stack.  Never call it from the control loop.
  bool readConfigDouble(const char* name, double* value, std::string* error);

 private:
  // Sends `text` on diagnostic channel 1 and accumulates the reply into
  // `out` until a newline or `out_size - 1` bytes.  Always NUL-terminates.
  bool diagnosticCommand(const char* text, char* out, size_t out_size,
                         std::string* error);

  // One DiagnosticRead round trip.  Appends to `out`; returns the bytes
  // added, or -1 on transport failure.  A reply of 0 bytes is normal and
  // just means the board has nothing more to say yet.
  int diagnosticPoll(char* out, size_t out_size, size_t used);

  // Reproduces moteus.h:1325-1329.  Single controller, so source = 0 and
  // can_prefix = 0.
  uint32_t makeArbitrationId(bool reply_required) const;

  // Sends `payload` and waits for the reply.  Returns false on timeout.
  bool transact(const uint8_t* payload, uint8_t size, MotorState* out);

  Config config_;
  bool initialized_ = false;
  bool torque_enabled_ = false;
  uint32_t last_round_trip_us_ = 0;
  uint32_t timeout_count_ = 0;
};

}  // namespace cube
