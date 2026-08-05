// The telemetry wire format.  One fixed-size binary record per control
// cycle, designed for a lossy packet link even though the first transport
// is USB.
//
// Nothing here may include a driver header or a vendor SDK -- this file is
// compiled by BOTH build systems (CMake for the x86 host, PlatformIO for
// the Teensy), and the static_assert below is what proves the two agree
// about the layout.  If they ever disagree it is a build error rather than
// a silently corrupt log.
//
// WHY BINARY, NOT TEXT
//
// The end transport is WiFi via a Xiao ESP32-C6, which is a *packet*
// medium: datagrams are dropped, reordered and delivered in bursts.  A
// printf stream has no record boundaries, so losing 40 bytes mid-line
// leaves the parser permanently misaligned.  A fixed-size record with a
// magic header, a sequence number and a CRC resynchronises on the very
// next record and reports exactly how much was lost.
//
// It is also the only thing that fits.  At 200 Hz this record is 23.6 kB/s;
// the equivalent ASCII is ~2.5x larger.  See docs -- 115200 baud tops out
// at 11.5 kB/s, so text does not fit the Xiao UART at all.
//
// ENDIANNESS AND FLOAT FORMAT -- DO NOT "FIX" THIS
//
// Cortex-M7 (Teensy), RISC-V (Xiao) and x86-64 (host) are all
// little-endian with IEEE-754 floats, so the packed struct is directly
// memcpy-able across all three.  There is deliberately no htole32()/ntohl()
// anywhere: adding byte swapping would be pure cost for a byte order that
// never varies on this hardware.  Revisit only if a big-endian consumer
// ever appears, and then do it in the Python decoder, not here.
//
// WHY float32 AND NOT double
//
// The i.MX RT1060 has a full VFPv5 FPU that does both in hardware, so the
// only difference is bytes on the wire: 4 vs 8, across 15 float fields, is
// 60 bytes per record -- over a third of the payload.  float32 carries ~7
// decimal digits; the BMI270 is a 16-bit sensor with ~4.8.  The precision
// is not the limiting factor, the sensor is.
#pragma once

#include <cstddef>
#include <cstdint>

namespace cube {

// Resync anchor.  Chosen to be an unlikely byte pair in float data rather
// than something like 0x0000 or 0xFFFF, which appear constantly in
// zero-filled or saturated fields.
constexpr std::uint16_t kTelemetryMagic = 0xA5C3;

// Bump on ANY change to the field layout below.  The decoder refuses a
// version it does not know instead of misparsing it, which is what stops a
// stale firmware from producing plausible-looking garbage.
constexpr std::uint8_t kTelemetryVersion = 1;

// Bit positions within TelemetryFrame::flags.
enum TelemetryFlags : std::uint8_t {
  kFlagArmed = 1u << 0,
  kFlagTorqueClamped = 1u << 1,
  kFlagImuValid = 1u << 2,
  kFlagMotorValid = 1u << 3,
  kFlagBodyValid = 1u << 4,
  kFlagDryRun = 1u << 5,
  // Set when the ring dropped at least one frame since the previous
  // successfully queued frame.  Loss is also visible as a gap in `seq`;
  // this flag tells you the loss happened at the producer (ring full)
  // rather than on the link.
  kFlagOverflow = 1u << 6,
};

// The record.  Field order is chosen so every member lands on its natural
// alignment with no padding: the u32 pair first, then the i16 blocks, then
// an unbroken run of f32 (all at offsets divisible by 4), then the u8 tail.
// `packed` therefore costs no unaligned-access penalty on the M7 -- it is
// here to make the layout an ABI guarantee rather than a compiler whim.
struct __attribute__((packed)) TelemetryFrame {
  std::uint16_t magic;            //   0  kTelemetryMagic
  std::uint8_t length;            //   2  sizeof(TelemetryFrame)
  std::uint8_t version;           //   3  kTelemetryVersion

  std::uint32_t seq;              //   4  monotonic; gaps == dropped frames
  // micros() since boot.  This is 32-bit and therefore WRAPS EVERY 71.6
  // MINUTES.  That is deliberate: unwrapping costs three lines in the
  // Python decoder, whereas a 64-bit timestamp would cost 4 bytes on every
  // single record forever.  The decoder adds 2^32 on each backward step.
  // Any consumer that diffs raw t_us without unwrapping will see a large
  // negative dt once every 71.6 minutes.
  std::uint32_t t_us;             //   8

  // Raw sensor counts, straight off the SPI burst, BEFORE any calibration.
  // Carried alongside the calibrated values so a log can be re-processed
  // with better calibration later -- ImuDriver::read() folds gyro bias in
  // irreversibly, so SI alone is a one-way door.
  std::int16_t raw_accel[3];      //  12
  std::int16_t raw_gyro[3];       //  18

  float accel_si[3];              //  24  m/s^2
  float gyro_si[3];               //  36  rad/s, bias corrected
  // BMI270 die temperature.  Not a nice-to-have: the datasheet gives the
  // gyro zero-rate offset a drift of +-0.015 dps/K, so without temperature
  // beside the gyro a bias drift and a real slow rotation are
  // indistinguishable in post-processing.
  float imu_temp_c;               //  48

  float theta;                    //  52  rad, from StateEstimator
  float theta_dot;                //  56  rad/s
  float wheel_omega;              //  60  rad/s

  float motor_position_rev;       //  64
  float motor_velocity_rev_s;     //  68
  float motor_torque_nm;          //  72  measured
  float q_current_a;              //  76  see note in MoteusDriverWrapper
  float bus_voltage_v;            //  80
  float motor_temp_c;             //  84

  // The setpoint.  Tracking error is (cmd_torque_nm - motor_torque_nm),
  // which is only computable if both are in the same record.
  float cmd_torque_nm;            //  88

  // Control terms BEFORE the clamp, so a log shows which gain dominated
  // and which one saturated.  Matches the documented intent of
  // ControlOutput in BalancingController.hpp.
  float term_theta;               //  92
  float term_theta_dot;           //  96
  float term_omega;               // 100

  // Loop diagnostics.  dt_s is the measured interval, not the nominal one;
  // slack_s is the deadline margin and goes negative on an overrun.
  float dt_s;                     // 104
  float slack_s;                  // 108

  std::uint8_t mode;              // 112  moteus mode
  std::uint8_t fault;             // 113  moteus fault code
  std::uint8_t safety;            // 114  cube::SafetyState
  std::uint8_t flags;             // 115  TelemetryFlags bitfield

  // CRC-16/CCITT-FALSE over bytes [0, 116) -- i.e. everything above,
  // including the header.  Covering the header is what lets the decoder
  // trust `length` and `version` before it acts on them.
  std::uint16_t crc16;            // 116
};

constexpr std::size_t kTelemetryFrameSize = 118;

// The contract between the two build systems and the Python decoder.  If
// this fires, the struct gained padding -- do not "fix" it by widening the
// expected size, fix the field order.
static_assert(sizeof(TelemetryFrame) == kTelemetryFrameSize,
              "TelemetryFrame must be exactly 118 packed bytes");
static_assert(offsetof(TelemetryFrame, crc16) == 116,
              "crc16 must be the last two bytes");
static_assert(offsetof(TelemetryFrame, accel_si) == 24,
              "float block must start 4-byte aligned at offset 24");

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final
// XOR.  Table-driven, 256 entries.
//
// This is deliberately called from the PRODUCER (inside push()), not the
// drain.  ~118 table lookups is well under a microsecond at 600 MHz, and
// keeping it here means the transport never has to look inside a frame --
// which is exactly what lets the USB sink be swapped for the Xiao UART
// sink without touching anything else.
std::uint16_t Crc16Ccitt(const void* data, std::size_t length);

// Fill magic/length/version, compute the CRC over the populated body, and
// store it.  Call this last, after every other field is set.
void FinalizeFrame(TelemetryFrame* frame);

// True if magic, length, version and CRC all check out.  Used by the host
// tests; the real decoder is the Python one.
bool ValidateFrame(const TelemetryFrame& frame);

}  // namespace cube
