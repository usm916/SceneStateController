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

static uint8_t s_runtime_mode = SSC_MODE;
static bool s_ir_ready = false;
static bool s_led_ready = false;
static bool s_elevator_ready = false;
static bool s_scene_ready = false;

static bool mode_is(uint8_t mode) {
  return s_runtime_mode == 0 || s_runtime_mode == mode;
}

static void ensure_modules_for_mode(uint8_t mode) {
  if ((mode == 0 || mode == 1) && !s_ir_ready) {
    ir_setup();
    s_ir_ready = true;
  }
  if ((mode == 0 || mode == 2) && !s_led_ready) {
    led_setup();
    s_led_ready = true;
  }
  if ((mode == 0 || mode == 3) && !s_elevator_ready) {
    elevator_setup();
    s_elevator_ready = true;
  }
  if (mode == 0 && !s_scene_ready) {
    scene_setup();
    s_scene_ready = true;
  }
}

static void set_runtime_mode(uint8_t mode) {
  if (mode > 4) return;
  s_runtime_mode = mode;
  ensure_modules_for_mode(mode);
  if (mode == 0) scene_setup();
  Serial.print("MODE ");
  Serial.println(s_runtime_mode);
}

static void poll_mode_switch_from_serial() {
  static char s_cmd_buf[16];
  static uint8_t s_cmd_len = 0;
  static bool s_capturing = false;

  while (Serial.available()) {
    const char p = (char)Serial.peek();
    if (!s_capturing) {
      if (p != 's' && p != 'S') return;
      s_capturing = true;
      s_cmd_len = 0;
    }

    const char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      s_cmd_buf[s_cmd_len] = '\0';
      s_capturing = false;

      if ((s_cmd_len == 2) && (s_cmd_buf[0] == 's' || s_cmd_buf[0] == 'S') && (s_cmd_buf[1] >= '0' && s_cmd_buf[1] <= '4')) {
        set_runtime_mode((uint8_t)(s_cmd_buf[1] - '0'));
      } else {
        Serial.println("MODE_CMD usage: s0..s4");
      }
      s_cmd_len = 0;
      return;
    }

    if (s_cmd_len < sizeof(s_cmd_buf) - 1) {
      s_cmd_buf[s_cmd_len++] = c;
    } else {
      s_capturing = false;
      s_cmd_len = 0;
      Serial.println("MODE_CMD too long");
      return;
    }
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

void setup() {
  Serial.begin(115200);
  delay(1200);
  print_banner();

  pi_link_setup();
  s_runtime_mode = SSC_MODE;
  ensure_modules_for_mode(s_runtime_mode);

  Serial.print("INITIAL MODE=");
  Serial.println(s_runtime_mode);
  Serial.println("Type s0..s4 + Enter to switch mode at runtime.");
  Serial.println("READY");
}

void loop() {
  const uint32_t now_ms = millis();
  poll_mode_switch_from_serial();
  ir_poll_serial_command();

  // Pi link
  if (mode_is(4)) {
    Event e;
    if (pi_link_poll(e)) {
      if (s_runtime_mode == 4) {
        pi_link_send_event(e);
      } else {
        if (e.type == EVT_PI_CMD_LED) apply_led_override(e.data.led.pattern_id);
        scene_handle_event(e);
      }
    }
  }

  // IR
  if (mode_is(1)) {
    Event e;
    if (ir_poll(e)) {
#if SSC_IR_LOG_ENABLE
      log_ir_event(e);
#endif
      if (s_runtime_mode == 1) {
        pi_link_send_event(e);
      } else {
        scene_handle_event(e);
      }
    }
  }

  // Elevator
  if (mode_is(3)) {
    Event ev_e;
    elevator_tick(now_ms, &ev_e);
    if (ev_e.type != EVT_NONE) {
      if (s_runtime_mode == 3) pi_link_send_event(ev_e);
      else scene_handle_event(ev_e);
    }
  }

  // LED tick
  if (mode_is(2)) {
    led_tick(now_ms);
  }

  // Scene tick
  if (s_runtime_mode == 0) {
    scene_tick(now_ms);
  }
}
