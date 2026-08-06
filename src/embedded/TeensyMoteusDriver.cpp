#include "embedded/TeensyMoteusDriver.hpp"

#include <Arduino.h>
#include <FlexCAN_T4.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

#include "moteus_protocol.h"

namespace cube {
namespace {

namespace mm = mjbots::moteus;

// FlexCAN_T4FD, not FlexCAN_T4: the plain template is CAN 2.0 only and
// its setBaudRate() is a no-op stub, so it would silently configure
// nothing.  Same reasoning as can_listen.
FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

constexpr uint32_t kArbitrationBaud = 1000000;
constexpr uint32_t kDataBaud = 5000000;

// CAN-FD carries only these payload lengths.  A frame whose length is
// not one of them cannot be transmitted, so short payloads are zero
// padded UP to the next valid size.  The moteus firmware ignores
// trailing zero bytes (they decode as padding in the register
// multiplex), which is what makes the padding safe.
constexpr uint8_t kValidDlc[] = {0,  1,  2,  3,  4,  5,  6,  7,  8,
                                 12, 16, 20, 24, 32, 48, 64};

// How many DiagnosticRead round trips to make before giving up on a
// reply.  The board answers a `conf get` in one or two, but it may return
// nothing for the first few while it composes output.  50 at ~0.5 ms is a
// 25 ms ceiling -- long enough to be certain, short enough that a dead
// board fails startup rather than hanging it.
constexpr int kDiagnosticPollAttempts = 50;

uint8_t roundUpDlc(uint8_t size) {
  for (uint8_t candidate : kValidDlc) {
    if (candidate >= size) { return candidate; }
  }
  return 64;
}

// Build an outgoing frame.  ALL THREE SEND SITES MUST GO THROUGH HERE --
// they were open-coded once and drifted, with the consequences below.
//
// [1] THE EXTENDED-ID FLAG.  CANFD_message_t defaults flags.extended to
//     false (FlexCAN_T4.h:80), and writeTxMailbox() then encodes the id as
//     FLEXCAN_MB_ID_IDSTD(x) = (x & 0x7FF) << 18 -- an 11-bit mask.  A
//     query id of 0x8001 becomes 0x001: destination 1 survives, and the
//     0x8000 REPLY-REQUIRED BIT IS SILENTLY STRIPPED.
//
//     The board then receives a valid, correctly addressed command and
//     acts on it, but never answers.  transact() times out and blames the
//     bus.  In the torque build that asymmetry is the dangerous one: the
//     driver concludes it has no link WHILE THE MOTOR IS EXECUTING THE
//     COMMAND IT JUST SENT.
//
//     The >= 0x7ff threshold is copied from the vendor client rather than
//     invented (moteus_transport.h:1250) so the two cannot disagree.
//
// [2] THE PADDING BYTE IS 0x50, NOT ZERO.  moteus pads short frames with
//     Multiplex::kNop (moteus_transport.h:1256, moteus_multiplex.h:76),
//     and its parser skips that byte explicitly.  0x00 is kWriteInt8 with
//     a variable-length count, so zero padding decodes as a run of
//     zero-length register writes -- benign in the reference parser, but
//     it is a deviation from the vendor client for no reason at all.
void buildFrame(CANFD_message_t* frame, uint32_t id, const uint8_t* payload,
                uint8_t size) {
  frame->id = id;
  frame->flags.extended = (id >= 0x7ffu);   // see [1]
  frame->brs = 1;     // bit rate switch -- the 5 Mbit data phase
  frame->edl = 1;     // extended data length -- this is an FD frame
  frame->len = roundUpDlc(size);

  for (uint8_t i = 0; i < frame->len; i++) {
    frame->buf[i] = (i < size) ? payload[i] : mm::Multiplex::kNop;  // see [2]
  }
}

// Copy the fields the control code uses out of a moteus query reply.
// Lifted verbatim from MoteusDriverWrapper.cpp:18-29 -- it is already
// pure, and the host and the Teensy must agree about what a reply means.
MotorState FromQuery(const mm::Query::Result& v) {
  MotorState state;
  state.position_rev = v.position;
  state.velocity_rev_s = v.velocity;
  state.torque_nm = v.torque;
  state.bus_voltage = v.voltage;
  state.motor_temperature_c = v.motor_temperature;
  state.mode = static_cast<int>(v.mode);
  state.fault = static_cast<int>(v.fault);
  state.valid = true;
  return state;
}

// ** THE HIGHEST-SEVERITY TRAP IN THIS FILE.  INTEGRATION.md:242. **
//
// PositionMode::Format defaults five fields to kIgnore, which means
// "do not put this in the frame at all":
//
//   feedforward_torque, kp_scale, kd_scale, maximum_torque,
//   watchdog_timeout
//
// The host code never hits this because Controller::Options supplies a
// default_position_format.  Hand-rolled frames get the raw defaults, so
// leaving them alone produces a frame that transmits cleanly, is
// acknowledged by the board, commands NO TORQUE -- and, worst of all,
// ARMS NO WATCHDOG.  None of that is visible at the CAN layer: the bus
// looks perfectly healthy while the safety net silently does not exist.
mm::PositionMode::Format torqueFormat() {
  mm::PositionMode::Format format;
  format.position = mm::kFloat;
  format.velocity = mm::kFloat;
  format.feedforward_torque = mm::kFloat;
  format.kp_scale = mm::kFloat;
  format.kd_scale = mm::kFloat;
  format.maximum_torque = mm::kFloat;
  format.watchdog_timeout = mm::kFloat;
  return format;
}

}  // namespace

TeensyMoteusDriver::TeensyMoteusDriver() = default;

TeensyMoteusDriver::TeensyMoteusDriver(const Config& config)
    : config_(config) {}

// moteus.h:1325-1329.  source = 0 and can_prefix = 0 for a single
// controller, so this reduces to the destination plus the reply bit.
uint32_t TeensyMoteusDriver::makeArbitrationId(bool reply_required) const {
  return static_cast<uint32_t>(config_.controller_id) |
         (reply_required ? 0x8000u : 0u);
}

bool TeensyMoteusDriver::initialize(std::string* error) {
  can3.begin();

  // Exact rates via CANFD_timings_t rather than a FLEXCAN_FDRATES
  // preset: the presets stop at CAN_1M_8M and none is 1M/5M, so a preset
  // would put the data phase at the wrong rate.  Values match
  // can_listen, which is what S3 proved against the real bus.
  //
  // CLK_40MHz, not 24: the data-phase bit must divide into a whole number
  // of time quanta between 5 and 48 (FlexCAN_T4FDTimings.tpp:311), and
  // 24 MHz / 5 Mbit = 4.8 Tq has no solution.  20 and 30 MHz fail as well.
  // 40 MHz gives 8 Tq at a 75% sample point.  See the longer note in
  // firmware/drivers/can_listen/main.cpp -- CLK_24MHz cost a bench session
  // that looked exactly like a broken transceiver.
  CANFD_timings_t timings;
  timings.clock = CLK_40MHz;
  timings.baudrate = kArbitrationBaud;
  timings.baudrateFD = kDataBaud;
  timings.propdelay = 190;
  timings.bus_length = 1;
  timings.sample = 75;

  // NOT listen-only: this one transmits.  The return value is checked
  // because setBaudRate() signals an unsolvable timing by returning 0 and
  // writing no register at all -- CAN3 then runs on begin()'s CTRL1, which
  // has every segment field zero and can never synchronise.  Diagnosed as
  // a wiring problem for as long as it went unchecked.
  if (!can3.setBaudRate(timings)) {
    if (error != nullptr) {
      *error =
          "setBaudRate() rejected the CAN timings -- CAN3 is NOT configured. "
          "This is a firmware fault, not wiring: no valid bit timing exists "
          "for this clock/rate pair";
    }
    return false;
  }

  // setRegions() ends with disableFIFO(), which is what arms the
  // mailboxes (0-6 RX, 7-13 TX at 64-byte regions).  Nothing else needed.
  //
  // NEVER call enableFIFO() here: the 1062 has no FD FIFO, so
  // FlexCAN_T4FD's enableFIFO(true) body is empty -- but the code before
  // the branch still zeroes every mailbox CS word to RX_INACTIVE.  It
  // disarms the receiver and reports nothing.  Replies would simply never
  // arrive, and query() would time out on a bus that is working.  See the
  // long note in firmware/drivers/can_listen/main.cpp.
  can3.setRegions(64);

  initialized_ = true;

  // Clear any latched fault before the first command, exactly as the
  // host driver does.  A board left in fault ignores torque commands
  // silently, which looks identical to a wiring problem.
  stop();

  // Prove the link before claiming success.  Without this, initialize()
  // returns true on a disconnected bus and the failure surfaces later,
  // somewhere less obvious.
  const MotorState state = query();
  if (!state.valid) {
    initialized_ = false;
    if (error != nullptr) {
      *error =
          "no reply from controller -- check CAN wiring, termination (~60 "
          "ohm), and that the moteus has logic power";
    }
    return false;
  }

  return true;
}

bool TeensyMoteusDriver::transact(const uint8_t* payload, uint8_t size,
                                  MotorState* out) {
  CANFD_message_t frame;
  buildFrame(&frame, makeArbitrationId(true), payload, size);

  const uint32_t start = micros();
  if (!can3.write(frame)) { return false; }

  // Poll for the reply.  A polled mailbox rather than an ISR: the
  // control loop is synchronous, and a deterministic spin here is easier
  // to reason about than an interrupt racing the loop.
  CANFD_message_t reply;
  while (micros() - start < config_.reply_timeout_us) {
    if (!can3.read(reply)) { continue; }

    // Ignore anything that is not this controller answering us.  On a
    // single-controller bus this should never fire, but a stale frame
    // parsed as a reply would produce plausible-looking garbage.
    const uint32_t expected_source =
        static_cast<uint32_t>(config_.controller_id) << 8;
    if ((reply.id & 0x7F00u) != expected_source) { continue; }

    last_round_trip_us_ = micros() - start;

    mm::Query::Result parsed;
    mm::CanData reply_data;
    for (uint8_t i = 0; i < reply.len && i < sizeof(reply_data.data); i++) {
      reply_data.data[i] = reply.buf[i];
    }
    reply_data.size = reply.len;

    parsed = mm::Query::Parse(reply_data.data, reply_data.size);
    *out = FromQuery(parsed);
    return true;
  }

  timeout_count_++;
  return false;
}

MotorState TeensyMoteusDriver::query() {
  MotorState state;                 // valid = false until proven otherwise
  if (!initialized_) { return state; }

  mm::CanData data;
  mm::WriteCanData writer(&data);
  mm::Query::Format format;
  mm::Query::Make(&writer, format);

  transact(data.data, data.size, &state);
  return state;
}

MotorState TeensyMoteusDriver::sendTorque(double torque_nm) {
  if (!initialized_) { return MotorState(); }

  // S4: the torque path is disarmed unless explicitly enabled, so this
  // degrades to a query.  Reporting real state keeps the control loop
  // exercisable end to end while the motor cannot move.
  if (!torque_enabled_) { return query(); }

  // NaN would otherwise be encoded and sent as a command.
  if (!std::isfinite(torque_nm)) { torque_nm = 0.0; }

  const double limit = config_.max_torque_nm;
  if (torque_nm > limit) { torque_nm = limit; }
  if (torque_nm < -limit) { torque_nm = -limit; }

  mm::PositionMode::Command command;

  // Pure torque.  position = NaN means "no position target"; zeroing both
  // gain scales removes the board's own position and velocity feedback,
  // leaving feedforward_torque as the only term.  Deliberate: the
  // balancing law is the regulator, and a second one underneath would
  // fight it.  Identical to MoteusDriverWrapper.cpp:207-223.
  command.position = std::numeric_limits<double>::quiet_NaN();
  command.velocity = 0.0;
  command.kp_scale = 0.0;
  command.kd_scale = 0.0;
  command.feedforward_torque = torque_nm;
  command.maximum_torque = limit;
  command.watchdog_timeout = config_.watchdog_timeout_s;

  mm::CanData data;
  mm::WriteCanData writer(&data);
  const auto format = torqueFormat();       // see the trap note above
  mm::PositionMode::Make(&writer, command, format);

  MotorState state;
  transact(data.data, data.size, &state);
  return state;
}

void TeensyMoteusDriver::stop() {
  if (!initialized_) { return; }

  mm::CanData data;
  mm::WriteCanData writer(&data);
  mm::StopMode::Format format;
  mm::StopMode::Make(&writer, mm::StopMode::Command(), format);

  MotorState ignored;
  transact(data.data, data.size, &ignored);
}

// ---------------------------------------------------------------------
// S5 -- configuration read-back
//
// The diagnostic channel is a byte stream tunnelled over CAN, not a
// register read: you write "conf get <name>\n" and then poll for output
// until a newline arrives.  DiagnosticWrite / DiagnosticRead /
// DiagnosticResponse (moteus_protocol.h:1384-1449) are std::-free, which
// is why this works unchanged on the MCU.
//
// transact() is not reused here because it parses every reply as a
// Query::Result.  A diagnostic reply is text, and forcing it through the
// query parser would produce garbage rather than an error.
// ---------------------------------------------------------------------

int TeensyMoteusDriver::diagnosticPoll(char* out, size_t out_size,
                                       size_t used) {
  mm::CanData data;
  mm::WriteCanData writer(&data);
  mm::DiagnosticRead::Command read_command;
  read_command.channel = 1;
  mm::DiagnosticRead::Make(&writer, read_command, mm::DiagnosticRead::Format());

  CANFD_message_t frame;
  buildFrame(&frame, makeArbitrationId(true), data.data, data.size);

  const uint32_t start = micros();
  if (!can3.write(frame)) { return -1; }

  CANFD_message_t reply;
  while (micros() - start < config_.reply_timeout_us) {
    if (!can3.read(reply)) { continue; }

    const uint32_t expected_source =
        static_cast<uint32_t>(config_.controller_id) << 8;
    if ((reply.id & 0x7F00u) != expected_source) { continue; }

    const auto parsed = mm::DiagnosticResponse::Parse(reply.buf, reply.len);
    if (parsed.channel != 1 || parsed.size <= 0) { return 0; }

    int copied = 0;
    for (int i = 0; i < parsed.size && (used + copied) < (out_size - 1); i++) {
      out[used + copied] = static_cast<char>(parsed.data[i]);
      copied++;
    }
    return copied;
  }

  timeout_count_++;
  return -1;
}

bool TeensyMoteusDriver::diagnosticCommand(const char* text, char* out,
                                           size_t out_size,
                                           std::string* error) {
  if (out_size == 0) { return false; }
  out[0] = '\0';

  if (!initialized_) {
    if (error != nullptr) { *error = "driver not initialized"; }
    return false;
  }

  // --- send ---
  {
    mm::CanData data;
    mm::WriteCanData writer(&data);
    mm::DiagnosticWrite::Command write_command;
    write_command.channel = 1;
    write_command.data = text;
    write_command.size = static_cast<int8_t>(strlen(text));
    mm::DiagnosticWrite::Make(&writer, write_command,
                              mm::DiagnosticWrite::Format());

    // No reply to the write itself, so the id is 0x0001 and this frame is
    // correctly a standard one -- buildFrame() decides that from the id.
    CANFD_message_t frame;
    buildFrame(&frame, makeArbitrationId(false), data.data, data.size);
    if (!can3.write(frame)) {
      if (error != nullptr) { *error = "CAN write failed"; }
      return false;
    }
  }

  // --- poll for the answer ---
  //
  // Bounded by attempts rather than by "until newline": a board that
  // never answers must not hang startup forever.  Empty replies are
  // normal while the board is still composing output, so only a
  // transport failure aborts.
  size_t used = 0;
  for (int attempt = 0; attempt < kDiagnosticPollAttempts; attempt++) {
    const int added = diagnosticPoll(out, out_size, used);
    if (added < 0) {
      if (error != nullptr) { *error = "no reply on the diagnostic channel"; }
      return false;
    }
    used += added;
    out[used] = '\0';

    if (memchr(out, '\n', used) != nullptr) { return true; }
    if (used >= out_size - 1) { return true; }

    if (added == 0) { delayMicroseconds(500); }
  }

  if (error != nullptr) {
    *error = "diagnostic reply never terminated with a newline";
  }
  return used > 0;
}

bool TeensyMoteusDriver::readConfigDouble(const char* name, double* value,
                                          std::string* error) {
  if (value == nullptr) { return false; }

  char command[96];
  snprintf(command, sizeof(command), "conf get %s\n", name);

  char reply[256];
  if (!diagnosticCommand(command, reply, sizeof(reply), error)) {
    return false;
  }

  // The board answers with the bare value and a newline, e.g. "15.0\n".
  // An error instead starts with "ERR" -- reporting that verbatim is more
  // useful than a parse failure, since it usually means the setting name
  // is wrong for this firmware version.
  if (strncmp(reply, "ERR", 3) == 0) {
    if (error != nullptr) {
      *error = std::string("controller rejected '") + name + "': " + reply;
    }
    return false;
  }

  char* end = nullptr;
  const double parsed = strtod(reply, &end);
  if (end == reply) {
    if (error != nullptr) {
      *error = std::string("could not parse reply for '") + name + "': " + reply;
    }
    return false;
  }

  *value = parsed;
  return true;
}

}  // namespace cube
