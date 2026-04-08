#include <Arduino.h>

#include "src/config.h"
#include "src/events.h"

#include "src/elevator_module.h"
#include "src/ir_module.h"
#include "src/led_module.h"
#include "src/pi_link.h"
#include "src/serial_console.h"
#include "src/scene_controller.h"
#include "src/console_logger.h"

static ConsoleLogger s_log(Serial);

static uint8_t s_runtime_mode = SSC_MODE;
static bool s_ir_ready = false;
static bool s_led_ready = false;
static bool s_elevator_ready = false;
static bool s_scene_ready = false;

static void apply_led_override(uint8_t pattern_id);
static bool mode_is(uint8_t mode);
static void ensure_modules_for_mode(uint8_t mode);
static void set_runtime_mode(uint8_t mode);

static void apply_led_override(uint8_t pattern_id) {
  switch (pattern_id) {
    case 0: led_set_pattern(LEDP_IDLE); break;
    case 1: led_set_pattern(LEDP_MOVING); break;
    case 2: led_set_pattern(LEDP_ARRIVED); break;
    case 3: led_set_pattern(LEDP_ERROR); break;
    default: break;
  }
}

static bool mode_is(uint8_t mode) {
  return s_runtime_mode == 0 || s_runtime_mode == mode;
}

static void ensure_modules_for_mode(uint8_t mode) {
  if ((mode == 0 || mode == 1) && !s_ir_ready) {
    Serial.println("Setting up IR...");
    ir_setup();
    s_ir_ready = true;
  }
  if ((mode == 0 || mode == 2) && !s_led_ready) {
    Serial.println("Setting up LED...");
    led_setup();
    s_led_ready = true;
  }
  if ((mode == 0 || mode == 3) && !s_elevator_ready) {
    Serial.println("Setting up elevator...");
    elevator_setup();
    s_elevator_ready = true;
  }
  if (mode == 0 && !s_scene_ready) {
    Serial.println("Setting up scene...");
    scene_setup();
    s_scene_ready = true;
  }

  Serial.println("setuped");
}

static void set_runtime_mode(uint8_t mode) {
  if (mode > 4) return;
  const uint8_t prev_mode = s_runtime_mode;
  s_runtime_mode = mode;
  ensure_modules_for_mode(mode);
  if (mode == 0) scene_setup();
  s_log.print_mode_change(prev_mode, s_runtime_mode);
}

void setup() {
  Serial.begin(SSC_USB_SERIAL_BAUD);
  delay(1200);
  s_log.print_banner();

  pi_link_setup();
  s_runtime_mode = SSC_MODE;
  ensure_modules_for_mode(s_runtime_mode);
  s_log.print_startup(s_runtime_mode);
}

void loop() {
  const uint32_t now_ms = millis();
  Event serial_event = {EVT_NONE, 0, {}};

  if (serial_console_poll(s_log, set_runtime_mode, &serial_event)) {
    if (s_runtime_mode == 0) {
      if (serial_event.type == EVT_PI_CMD_LED) apply_led_override(serial_event.data.led.pattern_id);
      scene_handle_event(serial_event);
    } else if (s_runtime_mode == 2 && serial_event.type == EVT_PI_CMD_LED) {
      apply_led_override(serial_event.data.led.pattern_id);
    } else if (s_runtime_mode == 3 && serial_event.type == EVT_PI_CMD_MOVE) {
      elevator_command_move_to(serial_event.data.move.target_floor);
    }
  }

  if (mode_is(1)) {
    Event e;
    if (ir_poll(e)) {
#if SSC_IR_LOG_ENABLE
      // s_log.print_ir_event(e);
#endif
      if (s_runtime_mode == 1) {
        pi_link_send_event(e);
      } else {
        scene_handle_event(e);
      }
    }
  }

  if (mode_is(3)) {
    Event ev_e;
    elevator_tick(now_ms, &ev_e);
    if (ev_e.type != EVT_NONE) {
      if (s_runtime_mode == 3) {
        pi_link_send_event(ev_e);
      } else {
        scene_handle_event(ev_e);
      }
    }
  }

  if (mode_is(2)) {
    led_tick(now_ms);
  }

  if (s_runtime_mode == 0) {
    scene_tick(now_ms);
  }
}
