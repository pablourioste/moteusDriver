#include "core/TelemetryFrame.hpp"

#include <cstring>

namespace cube {
namespace {

// CRC-16/CCITT-FALSE, poly 0x1021, MSB-first, init 0xFFFF, no reflection,
// no final XOR.  Check value for "123456789" is 0x29B1 -- the standard
// vector, asserted in tests/test_telemetry_ring.cpp.
//
// The table costs 512 bytes of flash and turns the per-bit loop into one
// lookup per byte.  On a 118-byte frame that is 118 iterations instead of
// 944, which is what keeps the CRC affordable inside the control loop.
constexpr std::uint16_t kPoly = 0x1021;

struct Crc16Table {
  std::uint16_t entry[256];

  // constexpr so the table lands in flash as initialised data rather than
  // being built at boot.  Teensy has plenty of flash and no time to spare
  // during bring-up.
  constexpr Crc16Table() : entry() {
    for (int i = 0; i < 256; i++) {
      std::uint16_t crc = static_cast<std::uint16_t>(i << 8);
      for (int bit = 0; bit < 8; bit++) {
        crc = (crc & 0x8000) ? static_cast<std::uint16_t>((crc << 1) ^ kPoly)
                             : static_cast<std::uint16_t>(crc << 1);
      }
      entry[i] = crc;
    }
  }
};

constexpr Crc16Table kTable;

}  // namespace

std::uint16_t Crc16Ccitt(const void* data, std::size_t length) {
  const std::uint8_t* bytes = static_cast<const std::uint8_t*>(data);
  std::uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < length; i++) {
    const std::uint8_t index =
        static_cast<std::uint8_t>((crc >> 8) ^ bytes[i]);
    crc = static_cast<std::uint16_t>((crc << 8) ^ kTable.entry[index]);
  }
  return crc;
}

void FinalizeFrame(TelemetryFrame* frame) {
  frame->magic = kTelemetryMagic;
  frame->length = static_cast<std::uint8_t>(kTelemetryFrameSize);
  frame->version = kTelemetryVersion;
  // Over everything except the CRC field itself, which is the last two
  // bytes.  Header included -- that is what makes `length` and `version`
  // trustworthy before the decoder acts on them.
  frame->crc16 = Crc16Ccitt(frame, kTelemetryFrameSize - sizeof(std::uint16_t));
}

bool ValidateFrame(const TelemetryFrame& frame) {
  if (frame.magic != kTelemetryMagic) { return false; }
  if (frame.length != kTelemetryFrameSize) { return false; }
  if (frame.version != kTelemetryVersion) { return false; }
  const std::uint16_t expected =
      Crc16Ccitt(&frame, kTelemetryFrameSize - sizeof(std::uint16_t));
  return expected == frame.crc16;
}

}  // namespace cube
