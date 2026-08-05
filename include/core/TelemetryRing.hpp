// Single-producer / single-consumer ring buffer for telemetry frames.
//
// THE ONE PROPERTY THAT MATTERS: push() never blocks, never allocates, and
// never touches I/O.  It is called from inside the control loop, where a
// stall is a missed deadline and a missed deadline on a balancing robot is
// a fall.  Everything else about this file follows from that.
//
// WHY A LOCK-FREE RING AND NOT A MUTEX + std::deque
//
// The control loop is the only producer and the drain is the only
// consumer.  SPSC is the one concurrency case that needs neither a mutex
// nor a compare-and-swap: a release-store on the producer index paired
// with an acquire-load on the consumer index is sufficient, because each
// index has exactly one writer.  A mutex here would risk the drain holding
// the lock while the loop wants it -- priority inversion in the one place
// the system cannot tolerate it.
//
// WHY POWER-OF-TWO CAPACITY
//
// The wrap becomes `& (Capacity - 1)`: one AND instruction.  With an
// arbitrary capacity it is a modulo, i.e. a hardware divide, which on
// Cortex-M7 is multi-cycle and variable-latency.  Constant-time push means
// no division.
//
// WHY OVERWRITE-OLDEST INSTEAD OF BLOCKING OR DROPPING-NEWEST
//
// If the transport stalls -- a WiFi retry, or a USB host that stopped
// reading -- the loop must not stall with it.  A dropped telemetry frame
// costs a gap in a plot; a missed control deadline costs the robot.  So
// the ring drops rather than waits.
//
// It drops the OLDEST because the newest frame is the one describing the
// situation you are most likely debugging, and because the loss is then
// always contiguous in time rather than scattered.
//
// Loss is never silent: the reader sees a jump in `seq`, and the next
// successfully queued frame carries kFlagOverflow.  Between them you can
// tell the difference between "the ring overflowed" and "the link dropped
// it", which are different bugs with different fixes.
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "core/TelemetryFrame.hpp"

namespace cube {

// Capacity must be a power of two.  64 frames is ~7.5 kB and buys ~320 ms
// of slack at 200 Hz -- enough to ride out a WiFi retry, negligible
// against the RT1060's 1 MB of RAM.
template <std::size_t Capacity = 64>
class TelemetryRing {
 public:
  static_assert(Capacity >= 2, "capacity must be at least 2");
  static_assert((Capacity & (Capacity - 1)) == 0,
                "capacity must be a power of two so the wrap is a mask");

  TelemetryRing() = default;

  TelemetryRing(const TelemetryRing&) = delete;
  TelemetryRing& operator=(const TelemetryRing&) = delete;

  // PRODUCER SIDE.  Control loop only.
  //
  // Stamps seq, sets kFlagOverflow if frames were lost since the last
  // successful push, computes the CRC, and stores.  O(1): a bounds check,
  // a 118-byte memcpy, a CRC over 118 bytes, and one release-store.
  //
  // Returns false if this push overwrote an unread frame.  The caller does
  // not need to do anything about that -- the flag and the seq gap already
  // record it -- but direct_motor_test-style harnesses may want to count.
  bool push(TelemetryFrame* frame) {
    const std::uint32_t head = head_.load(std::memory_order_relaxed);
    const std::uint32_t tail = tail_.load(std::memory_order_acquire);

    const bool full = (head - tail) >= Capacity;

    frame->seq = next_seq_++;
    if (overflow_pending_) {
      frame->flags |= kFlagOverflow;
      overflow_pending_ = false;
    }
    FinalizeFrame(frame);

    std::memcpy(&slot_[head & (Capacity - 1)], frame, sizeof(TelemetryFrame));

    if (full) {
      // Overwriting the oldest unread frame.  Advance the consumer index
      // past it so the reader never sees a torn record -- it will see the
      // seq gap instead, which is the honest signal.
      //
      // Safe from here because the consumer only ever moves tail forward,
      // so a concurrent pop() can at worst make this a no-op.
      tail_.store(tail + 1, std::memory_order_release);
      overflow_pending_ = true;
    }

    head_.store(head + 1, std::memory_order_release);
    return !full;
  }

  // CONSUMER SIDE.  Drain only.
  //
  // Copies the oldest frame out without removing it, so the caller can
  // decide whether the sink actually accepted it before committing.  That
  // two-step is what makes a partial write non-destructive.
  bool peek(TelemetryFrame* out) const {
    const std::uint32_t tail = tail_.load(std::memory_order_relaxed);
    const std::uint32_t head = head_.load(std::memory_order_acquire);
    if (tail == head) { return false; }
    std::memcpy(out, &slot_[tail & (Capacity - 1)], sizeof(TelemetryFrame));
    return true;
  }

  // Commit the frame returned by the last peek().
  void pop() {
    const std::uint32_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) { return; }
    tail_.store(tail + 1, std::memory_order_release);
  }

  // Approximate -- the producer may be mid-push.  For diagnostics only;
  // never branch on this in the control loop.
  std::size_t size() const {
    return head_.load(std::memory_order_acquire) -
           tail_.load(std::memory_order_acquire);
  }

  bool empty() const { return size() == 0; }

  std::uint32_t nextSequence() const { return next_seq_; }

 private:
  // Not atomic: producer-private state, only ever touched by push().
  std::uint32_t next_seq_ = 0;
  bool overflow_pending_ = false;

  std::atomic<std::uint32_t> head_{0};
  std::atomic<std::uint32_t> tail_{0};

  TelemetryFrame slot_[Capacity];
};

}  // namespace cube
