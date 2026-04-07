#include <Arduino.h>

#include "src/config.h"
#include "src/events.h"

#include "src/ir_module.h"
#include "src/led_module.h"
#include "src/elevator_module.h"
#include "src/pi_link.h"
#include "src/scene_controller.h"

static void print_banner() {
  Serial.println();
  Serial.println("===== SceneStateController (Arduino Prototype) =====");
  Serial.print("SSC_MODE="); Serial.println(SSC_MODE);
  Serial.println("Pi5 -> ESP32 commands:");
  Serial.println("  MOVE <floor>");
  Serial.println("  LED <pattern>  (0=IDLE 1=MOVING 2=ARRIVED 3=ERROR)");
  Serial.println("====================================================");
}

static void apply_led_override(uint8_t pattern_id) {
  switch (pattern_id) {
    case 0: led_set_pattern(LEDP_IDLE); break;
    case 1: led_set_pattern(LEDP_MOVING); break;
    case 2: led_set_pattern(LEDP_ARRIVED); break;
    case 3: led_set_pattern(LEDP_ERROR); break;
    default: break;
  }
}

#if SSC_IR_LOG_ENABLE
static void log_ir_event(const Event &event) {
  Serial.print("IR RX protocol=");
  Serial.print(event.data.ir.protocol);
  Serial.print(" addr=0x");
  Serial.print(event.data.ir.addr, HEX);
  Serial.print(" cmd=0x");
  Serial.println(event.data.ir.cmd, HEX);
}
#endif

void setup() {
  Serial.begin(115200);
  delay(1200);
  print_banner();

  pi_link_setup();

  if (SSC_MODE == 0 || SSC_MODE == 1) ir_setup();
  if (SSC_MODE == 0 || SSC_MODE == 2) led_setup();
  if (SSC_MODE == 0 || SSC_MODE == 3) elevator_setup();
  if (SSC_MODE == 0) scene_setup();

  Serial.println("READY");
}

void loop() {
  const uint32_t now_ms = millis();

  // Pi link
  if (SSC_MODE == 0 || SSC_MODE == 4) {
    Event e;
    if (pi_link_poll(e)) {
      if (SSC_MODE == 4) {
        pi_link_send_event(e);
      } else {
        if (e.type == EVT_PI_CMD_LED) apply_led_override(e.data.led.pattern_id);
        scene_handle_event(e);
      }
    }
  }

  // IR
  if (SSC_MODE == 0 || SSC_MODE == 1) {
    Event e;
    if (ir_poll(e)) {
#if SSC_IR_LOG_ENABLE
      log_ir_event(e);
#endif
      if (SSC_MODE == 1) {
        pi_link_send_event(e);
      } else {
        scene_handle_event(e);
      }
    }
  }

  // Elevator
  if (SSC_MODE == 0 || SSC_MODE == 3) {
    Event ev_e;
    elevator_tick(now_ms, &ev_e);
    if (ev_e.type != EVT_NONE) {
      if (SSC_MODE == 3) pi_link_send_event(ev_e);
      else scene_handle_event(ev_e);
    }
  }

  // LED tick
  if (SSC_MODE == 0 || SSC_MODE == 2) {
    led_tick(now_ms);
  }

  // Scene tick
  if (SSC_MODE == 0) {
    scene_tick(now_ms);
  }
}
