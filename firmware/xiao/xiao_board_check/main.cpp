// Step X0 -- is the Xiao alive, and does the toolchain work?
//
// The ESP32-C6 counterpart of firmware/teensy/board_check.  No external
// hardware, no wiring: this exists purely to separate "my build/flash
// pipeline is broken" from "my circuit is broken" before anything else is
// attached.  Every later step assumes this one passed.
//
//   pio run -e xiao_board_check -t upload
//   pio device monitor -e xiao_board_check
//
// UNLIKE THE TEENSY, this can usually be flashed directly from WSL.  The
// Teensy reboots into HalfKay on upload and enumerates as a different USB
// device, which drops the usbipd attachment; the ESP32-C6 flashes over its
// native USB and keeps its identity.  If your port does drop anyway, fall
// back to the Teensy pattern (build in WSL, flash from Windows) -- see
// docs/2D_model/XIAO_BRINGUP.md.
//
// What the output tells you:
//   chip / cores / flash    the board definition matched real silicon
//   heap                    the WiFi stack will need ~50 kB of this later
//   reset reason            distinguishes a clean boot from a brownout or
//                           a watchdog reset, which is the first thing you
//                           want to know when a later sketch "just stops"

#include <Arduino.h>

#include <esp_system.h>

namespace {

// Slow, asymmetric blink -- distinguishable at a glance from the Teensy's
// board_check and from a board stuck in its bootloader.
constexpr uint32_t kOnMs = 100;
constexpr uint32_t kOffMs = 900;

const char* resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:  return "POWERON";
    case ESP_RST_EXT:      return "EXT";
    case ESP_RST_SW:       return "SW";
    case ESP_RST_PANIC:    return "PANIC";        // a crash, not a reboot
    case ESP_RST_INT_WDT:  return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT:      return "WDT";
    case ESP_RST_BROWNOUT: return "BROWNOUT";     // supply sag: check USB
    case ESP_RST_SDIO:     return "SDIO";
    default:               return "UNKNOWN";
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  // Native USB CDC needs the host to open the port before output goes
  // anywhere.  Bounded wait so the sketch still runs when nothing is
  // attached -- an unbounded `while (!Serial)` bricks a headless boot.
  while (!Serial && millis() < 3000) {}

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println();
  Serial.println("=== xiao_board_check ===");
  Serial.printf("chip           %s rev %d\n", ESP.getChipModel(),
                ESP.getChipRevision());
  Serial.printf("cores          %d\n", ESP.getChipCores());
  Serial.printf("cpu            %lu MHz\n",
                static_cast<unsigned long>(getCpuFrequencyMhz()));
  Serial.printf("flash          %lu bytes\n",
                static_cast<unsigned long>(ESP.getFlashChipSize()));
  Serial.printf("free heap      %lu bytes\n",
                static_cast<unsigned long>(ESP.getFreeHeap()));
  Serial.printf("sdk            %s\n", ESP.getSdkVersion());
  Serial.printf("reset reason   %s\n", resetReasonName(esp_reset_reason()));
  Serial.println("PASS if the lines above are populated and tick follows");
}

void loop() {
  static uint32_t ticks = 0;

  digitalWrite(LED_BUILTIN, HIGH);
  delay(kOnMs);
  digitalWrite(LED_BUILTIN, LOW);
  delay(kOffMs);

  // Heap is reprinted every tick because the interesting failure is a heap
  // that shrinks over time -- a leak in the WiFi path shows up here long
  // before it shows up as a crash.
  Serial.printf("tick %lu  uptime %lu ms  heap %lu\n",
                static_cast<unsigned long>(++ticks),
                static_cast<unsigned long>(millis()),
                static_cast<unsigned long>(ESP.getFreeHeap()));
}
