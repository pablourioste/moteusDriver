// can_listen -- INTEGRATION.md S3.  CAN-FD receive only.
//
// The Teensy listens while your LAPTOP talks to the moteus over the
// existing fdcanusb.  Nothing here can transmit: no torque, no config, not
// even an acknowledgement.  That is the point -- it proves bit timing,
// termination and transceiver polarity while the board is physically
// incapable of commanding anything.
//
//   Terminal 1 (Windows):  pio device monitor
//   Terminal 2 (WSL):      ./moteus-venv/bin/moteus_tool -t 1 --dump-config
//
// Every frame the tool exchanges should appear here.
//
// ---------------------------------------------------------------------
// BEFORE FLASHING THIS -- moteus on LOGIC POWER ONLY, WHEEL OFF
// ---------------------------------------------------------------------
//
// H3, with a meter, Teensy DISCONNECTED, in this order:
//
//   [1] Transceiver logic level.  Probe RXD: it must idle at 3.3 V.
//       If it reads 5 V, STOP -- the RT1060 is not 5 V tolerant and
//       connecting pin 31 destroys CAN3 RX permanently.  The damage is
//       inside the SoC; there is no repair and no workaround.
//       Only after this passes do you wire pin 31.
//
//   [2] Bus termination.  Everything unpowered, CANH to CANL must read
//       ~60 ohm -- two 120 ohm terminators in parallel.  120 ohm means
//       one is missing; 40 ohm means there are three (the moteus has an
//       onboard jumper).  Wrong termination often passes at 1 Mbit
//       arbitration and fails only in the 5 Mbit data phase, which is an
//       intermittent that costs days.
//
//   [3] Ground continuity.  Teensy GND to moteus GND must beep.  Without
//       a shared ground the link appears to work and then corrupts once
//       motor current flows -- the classic reaction-wheel debugging
//       nightmare.
//
// WIRING (H2):
//   Teensy 30 (CAN3 TX) ---- TXD  |
//   Teensy 31 (CAN3 RX) ---- RXD  | 3.3 V CAN-FD transceiver
//   Teensy 3.3V         ---- VCC  |   (TCAN334, or MCP2562FD with
//   Teensy GND          ---- GND  |    VIO on 3.3 V)
//                                 +-- CANH / CANL ---- moteus
//
// PINS 30/31 ARE NOT NEGOTIABLE.  The Teensy 4.1 has three FlexCAN
// modules but only CAN3 supports Flexible Data-rate, and moteus requires
// FD.  CAN1 and CAN2 cannot talk to it at all.
//
// Teensy powered from USB-C, never VIN -- keeps the MCU off the motor
// rail, so a brownout when the wheel spins cannot reset it.

#include <Arduino.h>
#include <FlexCAN_T4.h>

namespace {

// FlexCAN_T4FD, not FlexCAN_T4: the plain template is CAN 2.0 only.  Its
// setBaudRate() is a no-op stub that exists purely to satisfy the base
// class, so using it here would configure nothing and silently listen at
// the wrong rate.
FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

// moteus runs 1 Mbit arbitration / 5 Mbit data.  Both must match the bus
// exactly; a mismatch in the data phase alone produces frames that pass
// arbitration and then fail, which reads as intermittent corruption.
constexpr uint32_t kArbitrationBaud = 1000000;
constexpr uint32_t kDataBaud = 5000000;

uint32_t g_frames = 0;
uint32_t g_last_report_ms = 0;

// Union of every latched ESR1 error bit seen since the last report.
// See the comment at its use site for why this is accumulated rather
// than sampled at report time.
uint32_t g_err_seen = 0;

// FlexCAN_T4FD has NO error() method -- that exists only on the CAN2.0
// FlexCAN_T4 template (FlexCAN_T4.tpp:1325), which fills its struct from
// an interrupt-populated ring buffer we do not run.  The FD class offers
// nothing equivalent, so the counters are read straight out of the
// peripheral instead.
//
// Which is arguably the better source here anyway: these are the live
// hardware registers, not a snapshot an ISR happened to capture.
//
// Register offsets from imxrt_flexcan.h:20-21, bit positions from the
// library's own decoder (FlexCAN_T4.tpp:1336-1349) so this reports the
// same things `error()` would have.
// Taken from the library's own CAN_DEV_TABLE enum (FlexCAN_T4.h:290)
// rather than written out as a literal, so it cannot drift from the
// base address the `can3` object above is actually driving.
constexpr uintptr_t kCan3Base = static_cast<uintptr_t>(CAN3);

// BIT1, BIT0, ACK, CRC, FRM, STF -- ESR1 bits 15..10.
constexpr uint32_t kEsr1ErrorMask = 0xFC00UL;

inline uint32_t esr1() { return *(volatile uint32_t*)(kCan3Base + 0x20); }
inline uint32_t ecr()  { return *(volatile uint32_t*)(kCan3Base + 0x1C); }

// ECR packs both counters: TX in bits 7:0, RX in bits 15:8.
inline uint8_t txErrCount() { return static_cast<uint8_t>(ecr() & 0xFF); }
inline uint8_t rxErrCount() { return static_cast<uint8_t>((ecr() >> 8) & 0xFF); }

// FLT_CONF, ESR1 bits 5:4.  Bus-off means the peripheral has taken
// itself off the bus entirely -- it is not a warning, it has stopped.
const char* faultConfinement() {
  switch ((esr1() >> 4) & 0x3) {
    case 0:  return "Error Active";
    case 1:  return "Error Passive";
    default: return "BUS OFF";
  }
}

// ESR1 bits 18, 7, 6, 3 -- the same 0x400C8 mask the library decodes.
const char* busState() {
  switch (esr1() & 0x400C8) {
    case 0x40080: return "Idle";
    case 0x40040: return "Transmitting";
    case 0x40008: return "Receiving";
    case 0x0:     return "NOT SYNCHRONISED";
    default:      return "?";
  }
}

// Decode the moteus arbitration ID.  From moteus.h:1325-1329:
//
//   arbitration_id = destination
//                  | (source << 8)
//                  | (reply_required ? 0x8000 : 0)
//                  | (can_prefix << 16)
//
// The host tool is source 0 talking to destination 1, so IDs of 0x8001
// (query, reply required) and 0x0001 (command, no reply) are what you
// expect to see, plus 0x0100 for the board's replies back to source 0.
void describeId(uint32_t id) {
  const uint32_t destination = id & 0xFF;
  const uint32_t source = (id >> 8) & 0x7F;
  const bool reply_required = (id & 0x8000) != 0;

  Serial.printf("id=0x%04lX dst=%lu src=%lu%s",
                (unsigned long)id, (unsigned long)destination,
                (unsigned long)source,
                reply_required ? " reply" : "");
}

}  // namespace

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Serial.println();
  Serial.println("=== can_listen -- S3, receive only ===");
  Serial.println();
  Serial.println("Confirm before continuing:");
  Serial.println("  [ ] transceiver RXD idles at 3.3 V (NOT 5 V)");
  Serial.println("  [ ] CANH-CANL reads ~60 ohm unpowered");
  Serial.println("  [ ] Teensy GND and moteus GND are continuous");
  Serial.println("  [ ] moteus on LOGIC POWER ONLY, wheel OFF");
  Serial.println();

  can3.begin();

  // Exact rates via CANFD_timings_t rather than one of the FLEXCAN_FDRATES
  // presets: the presets stop at CAN_1M_8M and none of them is 1M/5M, so a
  // preset would put the data phase at the wrong rate.
  CANFD_timings_t timings;
  timings.clock = CLK_24MHz;
  timings.baudrate = kArbitrationBaud;
  timings.baudrateFD = kDataBaud;
  timings.propdelay = 190;
  timings.bus_length = 1;
  timings.sample = 75;

  // LISTEN_ONLY is a peripheral mode, not a convention: in it the CAN
  // controller never drives the bus at all -- no acknowledgements, no
  // error frames. The Teensy is physically incapable of commanding the
  // moteus while this firmware is running, which is what makes this step
  // safe to perform with the controller powered.
  can3.setBaudRate(timings, LISTEN_ONLY);

  can3.setRegions(64);   // 64-byte mailboxes -- full FD payload
  can3.enableFIFO();

  Serial.printf("CAN3 up: %lu bps arbitration / %lu bps data, LISTEN ONLY\n",
                (unsigned long)kArbitrationBaud, (unsigned long)kDataBaud);
  Serial.println();
  Serial.println("Now provoke traffic from the host, e.g.");
  Serial.println("  ./moteus-venv/bin/moteus_tool -t 1 --dump-config");
  Serial.println();

  g_last_report_ms = millis();
}

void loop() {
  CANFD_message_t msg;

  // Sample the latched error flags EVERY pass, not once per report.
  // ESR1's error bits are write-1-to-clear and are set for the frame
  // that caused them; polling at 5 s would step straight over a burst.
  // Clearing them here (writing the bits back) is what makes the next
  // read report new errors rather than the same stale ones forever.
  {
    const uint32_t flags = esr1() & kEsr1ErrorMask;
    if (flags != 0) {
      g_err_seen |= flags;
      *(volatile uint32_t*)(kCan3Base + 0x20) = flags;   // w1c
    }
  }

  while (can3.read(msg)) {
    g_frames++;

    // Print the first frames in full, then fall back to a periodic count.
    // Formatting every frame at 5 Mbit would overrun the serial link and
    // make the Teensy the bottleneck rather than the bus.
    if (g_frames <= 20) {
      Serial.print("  ");
      describeId(msg.id);
      Serial.printf(" len=%u%s%s data=", msg.len, msg.brs ? " brs" : "",
                    msg.edl ? " fd" : "");
      for (int i = 0; i < msg.len && i < 16; i++) {
        Serial.printf("%02X ", msg.buf[i]);
      }
      if (msg.len > 16) { Serial.print("..."); }
      Serial.println();
    }
  }

  // Every 5 s: frame count and the error counters.
  //
  // THE ERROR COUNTERS ARE THE REAL CHECK.  Frames arriving proves the
  // wiring is roughly right; counters staying at zero proves the bit
  // timing is right.  A rising count with frames still arriving is the
  // signature of wrong termination -- it survives the 1 Mbit arbitration
  // phase and fails in the 5 Mbit data phase.
  const uint32_t now = millis();
  if (now - g_last_report_ms >= 5000) {
    g_last_report_ms = now;

    const uint8_t tx_err = txErrCount();
    const uint8_t rx_err = rxErrCount();

    Serial.printf("[%lus] frames=%lu  TX err=%u  RX err=%u  %s / %s\r\n",
                  (unsigned long)(now / 1000), (unsigned long)g_frames,
                  tx_err, rx_err, busState(), faultConfinement());

    if (g_frames == 0) {
      Serial.println("  no frames yet -- is the host actually talking?");
      Serial.println("  check: CANH/CANL not swapped, transceiver powered,");
      Serial.println("         pins 30/31 (NOT another CAN module)");
    }

    // Frames arriving proves the wiring is roughly right.  Zero counters
    // prove the bit timing is right.  These are different claims, and only
    // the second one survives the 5 Mbit data phase.
    //
    // g_err_seen accumulates the LATCHED ESR1 flags sampled in loop().
    // Reading them once here would miss almost everything: the bits are
    // write-1-to-clear and a 5 s gap between samples lets a whole error
    // burst come and go unobserved.
    if (rx_err > 0 || tx_err > 0 || g_err_seen != 0) {
      Serial.println("  *** BUS ERRORS -- stop and return to H3. ***");
      if (g_err_seen & (1UL << 12)) { Serial.println("      CRC_ERR"); }
      if (g_err_seen & (1UL << 11)) { Serial.println("      FRM_ERR (form)"); }
      if (g_err_seen & (1UL << 10)) { Serial.println("      STF_ERR (stuffing)"); }
      if (g_err_seen & (3UL << 14)) { Serial.println("      BIT_ERR"); }
      // ACK_ERR is EXPECTED here and is not a fault: in LISTEN_ONLY the
      // peripheral never drives the bus, so it cannot acknowledge.  It is
      // listed only so it does not look like a missing diagnosis.
      if (g_err_seen & (1UL << 13)) {
        Serial.println("      ACK_ERR (expected in listen-only, ignore)");
      }
      Serial.println("  Most likely termination: check ~60 ohm across the bus.");
      Serial.println("  Wrong termination passes at 1 Mbit and fails at 5 Mbit.");
      g_err_seen = 0;
    }
  }
}
