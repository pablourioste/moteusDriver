// Where telemetry frames go once they leave the ring.
//
// This two-method interface is the entire seam between "USB today" and
// "Xiao WiFi later".  The frame format, the ring, the Python decoder and
// the notebooks all sit on one side of it and never change; only the
// implementation on the other side is swapped.
//
// The sink is deliberately a DUMB BYTE PIPE.  It is handed a pointer and a
// length and it never looks inside -- no parsing, no reformatting, no CRC.
// That is what lets the Xiao forward frames it knows nothing about, and it
// is why the CRC is computed by the producer in TelemetryRing::push()
// rather than here.
//
// DrainTelemetry() below is the whole drain policy, in one place, so the
// non-blocking discipline cannot be accidentally reimplemented differently
// in each app.
#pragma once

#include <cstddef>
#include <cstdint>

#include "core/TelemetryFrame.hpp"

namespace cube {

class ITelemetrySink {
 public:
  virtual ~ITelemetrySink() = default;

  // Bytes that can be written RIGHT NOW without blocking.  This is the
  // method that makes the whole design work: the drain asks before it
  // writes, so a host that stopped reading its USB endpoint stalls the
  // telemetry and nothing else.  A sink that cannot answer this honestly
  // must return 0 rather than guess.
  virtual int space() = 0;

  // Write exactly `length` bytes.  Only called when space() >= length, so
  // an implementation may assume it will not block.  Returns bytes written.
  virtual std::size_t write(const std::uint8_t* data, std::size_t length) = 0;

  virtual const char* name() const = 0;
};

// Move as many frames as the sink will take, then stop.
//
// Call from the slack time at the end of a control cycle, never from
// inside the deadline.  `budget` caps the work per call so a long backlog
// is paid down over several cycles instead of in one burst that itself
// overruns -- draining 64 frames at once would be exactly the jitter this
// design exists to avoid.
//
// Returns the number of frames written.
template <typename Ring>
std::size_t DrainTelemetry(Ring* ring, ITelemetrySink* sink,
                           std::size_t budget = 8) {
  std::size_t written = 0;
  TelemetryFrame frame;
  while (written < budget) {
    if (sink->space() < static_cast<int>(sizeof(TelemetryFrame))) { break; }
    if (!ring->peek(&frame)) { break; }
    const std::size_t n = sink->write(
        reinterpret_cast<const std::uint8_t*>(&frame), sizeof(frame));
    // A short write means the sink lied about space().  Leave the frame in
    // the ring rather than popping a half-transmitted record -- the reader
    // would see a truncated frame, and resyncing from that costs more than
    // simply retrying next cycle.
    if (n != sizeof(frame)) { break; }
    ring->pop();
    written++;
  }
  return written;
}

}  // namespace cube
