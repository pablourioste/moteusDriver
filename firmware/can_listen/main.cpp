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

    CAN_error_t err;
    can3.error(err, false);  // false: fill the struct, do not print its own

    Serial.printf("[%lus] frames=%lu  TX err=%u  RX err=%u  state=%s\n",
                  (unsigned long)(now / 1000), (unsigned long)g_frames,
                  err.TX_ERR_COUNTER, err.RX_ERR_COUNTER, err.state);

    if (g_frames == 0) {
      Serial.println("  no frames yet -- is the host actually talking?");
      Serial.println("  check: CANH/CANL not swapped, transceiver powered,");
      Serial.println("         pins 30/31 (NOT another CAN module)");
    }

    // Frames arriving proves the wiring is roughly right.  Zero counters
    // prove the bit timing is right.  These are different claims, and only
    // the second one survives the 5 Mbit data phase.
    if (err.RX_ERR_COUNTER > 0 || err.TX_ERR_COUNTER > 0) {
      Serial.println("  *** BUS ERRORS -- stop and return to H3. ***");
      if (err.CRC_ERR) { Serial.println("      CRC_ERR"); }
      if (err.FRM_ERR) { Serial.println("      FRM_ERR (form)"); }
      if (err.STF_ERR) { Serial.println("      STF_ERR (stuffing)"); }
      if (err.BIT0_ERR || err.BIT1_ERR) { Serial.println("      BIT_ERR"); }
      Serial.println("  Most likely termination: check ~60 ohm across the bus.");
      Serial.println("  Wrong termination passes at 1 Mbit and fails at 5 Mbit.");
    }
  }
}
