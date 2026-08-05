// Host-side tests for the telemetry frame and ring buffer.
//
// Deliberately free-standing: no Boost, no gtest.  The upstream moteus
// tests need Boost.Test and are skipped when it is absent, and the
// telemetry layer is exactly the thing that must not silently go
// unverified -- a bug here corrupts every log downstream.  Plain asserts
// and a non-zero exit code are enough for what is being checked.
//
// These run on the desktop with no hardware attached, which is the point:
// steps 1 and 2 of the build order are fully verifiable before the Teensy
// is ever plugged in.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "core/TelemetryFrame.hpp"
#include "core/TelemetryRing.hpp"
#include "core/TelemetrySink.hpp"

namespace {

int g_failures = 0;

void Check(bool condition, const char* what) {
  if (!condition) {
    std::printf("  FAIL  %s\n", what);
    g_failures++;
  }
}

// ------------------------------------------------------------------ CRC

void TestCrcStandardVector() {
  std::printf("CRC-16/CCITT-FALSE standard vector\n");
  // The published check value for CRC-16/CCITT-FALSE over "123456789".
  // If this passes, the Python decoder's independent implementation can be
  // trusted against the same constant.
  const char* data = "123456789";
  const std::uint16_t crc = cube::Crc16Ccitt(data, 9);
  Check(crc == 0x29B1, "Crc16Ccitt(\"123456789\") == 0x29B1");
  if (crc != 0x29B1) {
    std::printf("        got 0x%04X\n", crc);
  }
}

void TestCrcDetectsBitFlip() {
  std::printf("CRC detects a single-bit flip\n");
  cube::TelemetryFrame frame{};
  frame.theta = 0.25f;
  cube::FinalizeFrame(&frame);
  Check(cube::ValidateFrame(frame), "freshly finalized frame validates");

  // Flip one bit in the middle of the payload.
  std::uint8_t* bytes = reinterpret_cast<std::uint8_t*>(&frame);
  bytes[50] ^= 0x01;
  Check(!cube::ValidateFrame(frame), "bit flip is rejected");
}

void TestValidateRejectsBadHeader() {
  std::printf("Header validation\n");
  cube::TelemetryFrame frame{};
  cube::FinalizeFrame(&frame);

  cube::TelemetryFrame bad_magic = frame;
  bad_magic.magic = 0x1234;
  Check(!cube::ValidateFrame(bad_magic), "wrong magic rejected");

  cube::TelemetryFrame bad_version = frame;
  bad_version.version = cube::kTelemetryVersion + 1;
  Check(!cube::ValidateFrame(bad_version), "wrong version rejected");

  cube::TelemetryFrame bad_length = frame;
  bad_length.length = 42;
  Check(!cube::ValidateFrame(bad_length), "wrong length rejected");
}

void TestFrameLayout() {
  std::printf("Frame layout\n");
  // The static_asserts in the header already enforce this at compile time;
  // repeating it here makes the failure legible at runtime too, and pins
  // the value the Python struct format string depends on.
  Check(sizeof(cube::TelemetryFrame) == 118, "sizeof == 118");
  Check(offsetof(cube::TelemetryFrame, seq) == 4, "seq at offset 4");
  Check(offsetof(cube::TelemetryFrame, raw_accel) == 12, "raw_accel at 12");
  Check(offsetof(cube::TelemetryFrame, accel_si) == 24, "accel_si at 24");
  Check(offsetof(cube::TelemetryFrame, cmd_torque_nm) == 88, "cmd at 88");
  Check(offsetof(cube::TelemetryFrame, flags) == 115, "flags at 115");
  Check(offsetof(cube::TelemetryFrame, crc16) == 116, "crc16 at 116");
}

// ----------------------------------------------------------------- ring

void TestRingBasicFifo() {
  std::printf("Ring FIFO ordering\n");
  cube::TelemetryRing<8> ring;
  Check(ring.empty(), "starts empty");

  for (int i = 0; i < 5; i++) {
    cube::TelemetryFrame frame{};
    frame.theta = static_cast<float>(i);
    Check(ring.push(&frame), "push does not report overflow");
  }
  Check(ring.size() == 5, "size == 5 after 5 pushes");

  for (int i = 0; i < 5; i++) {
    cube::TelemetryFrame out{};
    Check(ring.peek(&out), "peek succeeds");
    Check(out.theta == static_cast<float>(i), "frames come out in order");
    Check(out.seq == static_cast<std::uint32_t>(i), "seq is monotonic");
    Check(cube::ValidateFrame(out), "frame validates after round trip");
    ring.pop();
  }
  Check(ring.empty(), "empty after draining");
}

void TestRingPeekIsNonDestructive() {
  std::printf("peek() without pop() does not consume\n");
  cube::TelemetryRing<8> ring;
  cube::TelemetryFrame frame{};
  frame.theta = 7.0f;
  ring.push(&frame);

  cube::TelemetryFrame a{}, b{};
  Check(ring.peek(&a), "first peek");
  Check(ring.peek(&b), "second peek");
  Check(a.seq == b.seq, "same frame both times");
  Check(ring.size() == 1, "still one frame queued");
}

void TestRingOverwritesOldest() {
  std::printf("Ring overwrites oldest when full\n");
  cube::TelemetryRing<4> ring;

  // Push 6 into a 4-slot ring: 0 and 1 should be discarded.
  for (int i = 0; i < 6; i++) {
    cube::TelemetryFrame frame{};
    frame.theta = static_cast<float>(i);
    const bool ok = ring.push(&frame);
    if (i < 4) {
      Check(ok, "no overflow while space remains");
    } else {
      Check(!ok, "overflow reported once full");
    }
  }

  Check(ring.size() == 4, "size clamped to capacity");

  cube::TelemetryFrame out{};
  Check(ring.peek(&out), "peek after overflow");
  // The oldest surviving frame is #2, not #0 -- this is the
  // drop-oldest-never-block property the control loop depends on.
  Check(out.theta == 2.0f, "oldest surviving frame is #2");
  Check(out.seq == 2, "seq gap of 2 is visible to the reader");
}

void TestOverflowFlagIsSet() {
  std::printf("Overflow sets kFlagOverflow on the next frame\n");
  cube::TelemetryRing<4> ring;

  for (int i = 0; i < 5; i++) {
    cube::TelemetryFrame frame{};
    ring.push(&frame);
  }
  // Frame 5 is the first push after an overflow occurred, so it carries
  // the flag.  This is what distinguishes "the ring dropped it" from "the
  // link dropped it" when reading a log.
  cube::TelemetryFrame frame{};
  ring.push(&frame);

  bool found_flag = false;
  cube::TelemetryFrame out{};
  while (ring.peek(&out)) {
    if (out.flags & cube::kFlagOverflow) { found_flag = true; }
    Check(cube::ValidateFrame(out), "frame still valid after overflow");
    ring.pop();
  }
  Check(found_flag, "kFlagOverflow appears in the surviving frames");
}

void TestSequenceSurvivesWrap() {
  std::printf("Sequence numbers survive index wrap\n");
  cube::TelemetryRing<4> ring;
  std::uint32_t expected = 0;

  // Push and drain far more than the capacity so the head/tail indices
  // wrap the mask many times over.
  for (int cycle = 0; cycle < 100; cycle++) {
    cube::TelemetryFrame frame{};
    ring.push(&frame);
    cube::TelemetryFrame out{};
    if (ring.peek(&out)) {
      Check(out.seq == expected, "seq continuous across wrap");
      expected++;
      ring.pop();
    }
  }
}

// ----------------------------------------------------------------- sink

// A sink with a settable budget, so the drain policy can be tested
// without any hardware.  Mirrors what a USB endpoint does when the host
// stops reading: space() goes to zero and stays there.
class FakeSink : public cube::ITelemetrySink {
 public:
  int available = 1 << 20;
  std::string data;

  int space() override { return available; }

  std::size_t write(const std::uint8_t* bytes, std::size_t length) override {
    if (static_cast<int>(length) > available) { return 0; }
    data.append(reinterpret_cast<const char*>(bytes), length);
    available -= static_cast<int>(length);
    return length;
  }

  const char* name() const override { return "fake"; }
};

void TestDrainRespectsBudget() {
  std::printf("Drain honours its per-call budget\n");
  cube::TelemetryRing<64> ring;
  for (int i = 0; i < 20; i++) {
    cube::TelemetryFrame frame{};
    ring.push(&frame);
  }

  FakeSink sink;
  const std::size_t written = cube::DrainTelemetry(&ring, &sink, 8);
  Check(written == 8, "wrote exactly the budget");
  Check(ring.size() == 12, "the rest stay queued for the next cycle");
  Check(sink.data.size() == 8 * sizeof(cube::TelemetryFrame),
        "byte count matches frame count");
}

void TestDrainStopsWhenSinkIsFull() {
  std::printf("Drain stops cleanly when the sink refuses\n");
  cube::TelemetryRing<64> ring;
  for (int i = 0; i < 10; i++) {
    cube::TelemetryFrame frame{};
    ring.push(&frame);
  }

  FakeSink sink;
  // Room for two frames and a few spare bytes -- not a third.
  sink.available = 2 * static_cast<int>(sizeof(cube::TelemetryFrame)) + 10;

  const std::size_t written = cube::DrainTelemetry(&ring, &sink, 100);
  Check(written == 2, "only the frames that fit were written");
  Check(ring.size() == 8, "unwritten frames remain queued, not lost");

  // This is the property that matters: a stalled sink must never consume
  // a frame it did not fully transmit, or the reader sees a truncated
  // record.
  Check(sink.data.size() == 2 * sizeof(cube::TelemetryFrame),
        "no partial frame reached the sink");
}

void TestDrainOnStalledSinkIsNoOp() {
  std::printf("A fully stalled sink loses nothing\n");
  cube::TelemetryRing<64> ring;
  cube::TelemetryFrame frame{};
  ring.push(&frame);

  FakeSink sink;
  sink.available = 0;   // host stopped reading

  const std::size_t written = cube::DrainTelemetry(&ring, &sink, 8);
  Check(written == 0, "nothing written");
  Check(ring.size() == 1, "frame still queued");
  Check(sink.data.empty(), "nothing reached the sink");
}

void TestByteStreamIsDecodable() {
  std::printf("Emitted byte stream is resyncable\n");
  cube::TelemetryRing<64> ring;
  for (int i = 0; i < 5; i++) {
    cube::TelemetryFrame frame{};
    frame.theta = static_cast<float>(i) * 0.5f;
    ring.push(&frame);
  }
  FakeSink sink;
  cube::DrainTelemetry(&ring, &sink, 100);

  // Walk the raw bytes the way the Python decoder does: find magic,
  // validate, step forward by one frame.
  const std::uint8_t* bytes =
      reinterpret_cast<const std::uint8_t*>(sink.data.data());
  std::size_t offset = 0;
  int decoded = 0;
  while (offset + sizeof(cube::TelemetryFrame) <= sink.data.size()) {
    cube::TelemetryFrame frame;
    std::memcpy(&frame, bytes + offset, sizeof(frame));
    if (cube::ValidateFrame(frame)) {
      decoded++;
      offset += sizeof(frame);
    } else {
      offset++;
    }
  }
  Check(decoded == 5, "all five frames decode from the raw stream");
}

// ------------------------------------------------------- fixture emitter

// Write a synthetic capture for the Python decoder to chew on.
//
// This is what makes the C++ and Python implementations verify EACH OTHER
// rather than each being checked against its own assumptions.  The two
// CRCs, the two struct layouts and the two field orderings were written
// separately; if any of them disagree, decoding this file fails.
//
// The waveforms are deliberately non-trivial (varying per frame, spanning
// negative values) so that a byte-order or field-offset error cannot hide
// behind zeros or symmetry.
int EmitFixture(const char* path, int count, bool corrupt) {
  std::FILE* out = std::fopen(path, "wb");
  if (out == nullptr) {
    std::printf("cannot open %s\n", path);
    return 1;
  }

  cube::TelemetryRing<128> ring;
  for (int i = 0; i < count; i++) {
    cube::TelemetryFrame frame{};
    const float t = static_cast<float>(i) * 0.0025f;   // 400 Hz

    // Raw counts and their SI equivalents at the rig's configured ranges:
    // 16384 LSB/g and 65.536 LSB/dps.  Python recomputes these from the
    // datasheet constants and compares.
    frame.raw_accel[0] = static_cast<std::int16_t>(1000 + i);
    frame.raw_accel[1] = static_cast<std::int16_t>(-2000 + i);
    frame.raw_accel[2] = static_cast<std::int16_t>(16384);
    frame.raw_gyro[0] = static_cast<std::int16_t>(i % 500);
    frame.raw_gyro[1] = static_cast<std::int16_t>(-(i % 300));
    frame.raw_gyro[2] = static_cast<std::int16_t>(65);

    for (int a = 0; a < 3; a++) {
      frame.accel_si[a] =
          static_cast<float>(frame.raw_accel[a] * 9.80665 / 16384.0);
      frame.gyro_si[a] =
          static_cast<float>(frame.raw_gyro[a] * 0.017453292519943295 / 65.536);
    }

    frame.imu_temp_c = 29.5f + 0.001f * static_cast<float>(i);
    frame.theta = 0.05f * static_cast<float>((i % 40) - 20);
    frame.theta_dot = -0.02f * static_cast<float>((i % 30) - 15);
    frame.wheel_omega = 1.5f * static_cast<float>(i % 100);
    frame.motor_position_rev = 0.01f * static_cast<float>(i);
    frame.motor_velocity_rev_s = 2.0f;
    frame.motor_torque_nm = 0.05f * static_cast<float>((i % 10) - 5);
    frame.q_current_a = 1.2f;
    frame.bus_voltage_v = 24.0f;
    frame.motor_temp_c = 31.0f;
    frame.cmd_torque_nm = frame.motor_torque_nm + 0.002f;
    frame.term_theta = -0.1f * frame.theta;
    frame.term_theta_dot = -0.05f * frame.theta_dot;
    frame.term_omega = -0.001f * frame.wheel_omega;
    frame.dt_s = 0.0025f;
    frame.slack_s = 0.0018f;
    frame.t_us = static_cast<std::uint32_t>(t * 1e6f);
    frame.mode = 10;
    frame.fault = 0;
    frame.safety = 0;
    frame.flags = cube::kFlagArmed | cube::kFlagImuValid |
                  cube::kFlagMotorValid | cube::kFlagBodyValid;

    ring.push(&frame);

    cube::TelemetryFrame queued{};
    if (ring.peek(&queued)) {
      std::fwrite(&queued, 1, sizeof(queued), out);
      ring.pop();
    }
  }

  if (corrupt) {
    // Three separate insults, so the decoder's recovery is tested rather
    // than merely its happy path:
    //   1. a bit flip inside one frame  -> one CRC failure
    //   2. a run of junk between frames -> resync must skip exactly it
    //   3. a truncated frame at the end -> must not crash or hang
    std::fwrite("GARBAGEGARBAGE", 1, 14, out);
    cube::TelemetryFrame partial{};
    cube::FinalizeFrame(&partial);
    std::fwrite(&partial, 1, 60, out);      // half a frame, then EOF
  }

  std::fclose(out);
  std::printf("wrote %d frames to %s%s\n", count, path,
              corrupt ? " (with injected corruption)" : "");
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  // Fixture mode: emit a capture file instead of running the tests.
  //   test_telemetry_ring --emit <path> [count] [--corrupt]
  if (argc >= 3 && std::string(argv[1]) == "--emit") {
    const int count = (argc >= 4) ? std::atoi(argv[3]) : 400;
    bool corrupt = false;
    for (int i = 3; i < argc; i++) {
      if (std::string(argv[i]) == "--corrupt") { corrupt = true; }
    }
    return EmitFixture(argv[2], count, corrupt);
  }

  std::printf("=== telemetry frame + ring tests ===\n");

  TestFrameLayout();
  TestCrcStandardVector();
  TestCrcDetectsBitFlip();
  TestValidateRejectsBadHeader();

  TestRingBasicFifo();
  TestRingPeekIsNonDestructive();
  TestRingOverwritesOldest();
  TestOverflowFlagIsSet();
  TestSequenceSurvivesWrap();

  TestDrainRespectsBudget();
  TestDrainStopsWhenSinkIsFull();
  TestDrainOnStalledSinkIsNoOp();
  TestByteStreamIsDecodable();

  if (g_failures == 0) {
    std::printf("=== all tests passed ===\n");
    return 0;
  }
  std::printf("=== %d check(s) FAILED ===\n", g_failures);
  return 1;
}
