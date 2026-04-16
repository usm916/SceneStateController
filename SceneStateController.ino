#include <Arduino.h>

#include "src/config.h"
#include "src/events.h"
#include "src/WebOtaBlinkApp.h"

#include "src/elevator_module.h"
#include "src/ir_module.h"
#include "src/led_module.h"
#include "src/pi_link.h"
#include "src/serial_console.h"
#include "src/scene_controller.h"
#include "src/console_logger.h"
#include "src/shared_serial.h"
#include "src/button_position_store.h"

WebOtaBlinkApp app;

static ConsoleLogger s_log(Serial);

static uint8_t s_runtime_mode = SSC_MODE;
static bool s_ir_ready = false;
static bool s_led_ready = false;
static bool s_elevator_ready = false;
static bool s_scene_ready = false;
static int8_t s_manual_spin_dir = 0;
static RemoteButton s_last_floor_btn = BTN_NONE;
static RemoteButton s_last_control_btn = BTN_NONE;
static constexpr uint16_t kManualSpinReleaseGraceMs = 150;
static constexpr uint16_t kManualSpinRampMs = 500;
static uint32_t s_manual_spin_last_hold_ms = 0;
static uint32_t s_manual_spin_press_start_ms = 0;
static uint32_t s_manual_spin_decel_start_ms = 0;
static uint16_t s_manual_spin_decel_start_speed = 0;
static uint16_t s_manual_spin_current_speed = 0;
static uint32_t s_motor_timer_last_ms = 0;
static uint32_t s_led_timer_last_ms = 0;
static constexpr uint16_t kMotorTimerIntervalMs = 1;
static constexpr uint16_t kLedTimerIntervalMs = 10;

static void apply_led_override(uint8_t pattern_id);
static bool mode_is(uint8_t mode);
static void ensure_modules_for_mode(uint8_t mode);
static void set_runtime_mode(uint8_t mode);
static uint16_t calc_ramp_up_speed(uint32_t now_ms);
static uint16_t calc_ramp_down_speed(uint32_t now_ms);
static void command_manual_spin(int8_t dir, uint16_t speed_steps_per_sec);
static uint16_t manual_spin_speed_cap();
static bool timer_due(uint32_t now_ms, uint32_t* last_ms, uint16_t interval_ms);
static void run_motor_timer(uint32_t now_ms);
static void run_led_timer(uint32_t now_ms);

static uint16_t manual_spin_speed_cap() {
  const float move_speed_f = elevator_move_max_speed();
  return (move_speed_f > 0.0f) ? (uint16_t)move_speed_f : 1;
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

static uint16_t calc_ramp_up_speed(uint32_t now_ms) {
  const uint32_t elapsed = now_ms - s_manual_spin_press_start_ms;
  const uint16_t max_speed = manual_spin_speed_cap();
  if (elapsed >= kManualSpinRampMs) return max_speed;
  const uint32_t speed = ((uint32_t)max_speed * elapsed) / kManualSpinRampMs;
  return (uint16_t)((speed == 0) ? 1 : speed);
}

static uint16_t calc_ramp_down_speed(uint32_t now_ms) {
  const uint32_t elapsed = now_ms - s_manual_spin_decel_start_ms;
  if (elapsed >= kManualSpinRampMs) return 0;
  const uint32_t remain = kManualSpinRampMs - elapsed;
  const uint32_t speed = ((uint32_t)s_manual_spin_decel_start_speed * remain) / kManualSpinRampMs;
  return (uint16_t)speed;
}

static void command_manual_spin(int8_t dir, uint16_t speed_steps_per_sec) {
  if (speed_steps_per_sec == 0) return;
  if (dir > 0) {
    elevator_command_spin_cw(speed_steps_per_sec);
  } else if (dir < 0) {
    elevator_command_spin_ccw(speed_steps_per_sec);
  }
}

static bool timer_due(uint32_t now_ms, uint32_t* last_ms, uint16_t interval_ms) {
  if (*last_ms == 0 || (uint32_t)(now_ms - *last_ms) >= interval_ms) {
    *last_ms = now_ms;
    return true;
  }
  return false;
}

static void run_motor_timer(uint32_t now_ms) {
  if (!mode_is(3)) return;
  if (!timer_due(now_ms, &s_motor_timer_last_ms, kMotorTimerIntervalMs)) return;

  Event ev_e = {EVT_NONE, 0, {}};
  elevator_tick(now_ms, &ev_e);
  if (ev_e.type != EVT_NONE) {
    if (s_runtime_mode == 3) {
      pi_link_send_event(ev_e);
    } else {
      scene_handle_event(ev_e);
    }
  }
}

static void run_led_timer(uint32_t now_ms) {
  if (!mode_is(2)) return;
  if (!timer_due(now_ms, &s_led_timer_last_ms, kLedTimerIntervalMs)) return;
  led_tick(now_ms);
}

void setup() {
  Serial.begin(SSC_USB_SERIAL_BAUD);
  delay(1200);
  app.begin();
  s_log.print_banner();

  shared_serial_setup();
  pi_link_setup();
  button_position_store_setup();
  s_runtime_mode = SSC_MODE;
  ensure_modules_for_mode(s_runtime_mode);
  s_log.print_startup(s_runtime_mode);
}

void loop() {
  app.loop();

  uint32_t now_ms = millis();
  shared_serial_pump();
  Event serial_event = {EVT_NONE, 0, {}};

  if (s_runtime_mode == 3) {
    handleSerialInput();
  } else if (serial_console_poll(s_log, set_runtime_mode, &serial_event)) {
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

    if (s_elevator_ready) {
      const RemoteButton current_btn = ir_active_btn();
      const bool prev_pressed = (current_btn == BTN_PREV);
      const bool next_pressed = (current_btn == BTN_NEXT);
      const bool prev_released = ir_btn_released(BTN_PREV);
      const bool next_released = ir_btn_released(BTN_NEXT);
      uint8_t floor_btn_index = 0;
      const bool floor_btn_pressed = button_position_store_index_from_remote(current_btn, &floor_btn_index);
      const bool control_btn_pressed =
          current_btn == BTN_POWER || current_btn == BTN_EQ || current_btn == BTN_VOL_DOWN || current_btn == BTN_VOL_UP ||
          current_btn == BTN_MUTE;

      if (floor_btn_pressed && current_btn != s_last_floor_btn) {
        int32_t target_steps = 0;
        if (button_position_store_target(floor_btn_index, &target_steps)) {
          handleInput(target_steps);
        } else {
          Serial.print("BTN_");
          Serial.print(floor_btn_index);
          Serial.println(" is not recorded. Use rec_<button> first.");
        }
        s_last_floor_btn = current_btn;
        s_manual_spin_dir = 0;
        s_manual_spin_current_speed = 0;
        s_manual_spin_decel_start_ms = 0;
        s_manual_spin_decel_start_speed = 0;
      } else if (!floor_btn_pressed) {
        s_last_floor_btn = BTN_NONE;
      }

      if (control_btn_pressed && current_btn != s_last_control_btn) {
        if (current_btn == BTN_POWER) {
          elevator_command_emergency_stop();
        } else if (current_btn == BTN_EQ) {
          elevator_command_start_calibration();
        } else if (current_btn == BTN_VOL_DOWN) {
          elevator_command_home_zero();
        } else if (current_btn == BTN_VOL_UP) {
          elevator_command_home_top();
        } else if (current_btn == BTN_MUTE) {
          button_position_store_set_zero(elevator_current_position_steps());
          Serial.println("mute: set current position as zero.");
        }
        s_last_control_btn = current_btn;
        s_manual_spin_dir = 0;
        s_manual_spin_current_speed = 0;
        s_manual_spin_decel_start_ms = 0;
        s_manual_spin_decel_start_speed = 0;
      } else if (!control_btn_pressed) {
        s_last_control_btn = BTN_NONE;
      }

      if (prev_pressed && !next_pressed) {
        s_manual_spin_last_hold_ms = now_ms;
        s_manual_spin_decel_start_ms = 0;
        s_manual_spin_decel_start_speed = 0;
        if (s_manual_spin_dir != -1) {
          s_manual_spin_dir = -1;
          s_manual_spin_press_start_ms = now_ms;
        }
        const uint16_t ramp_speed = calc_ramp_up_speed(now_ms);
        command_manual_spin(-1, ramp_speed);
        s_manual_spin_current_speed = ramp_speed;
      } else if (next_pressed && !prev_pressed) {
        s_manual_spin_last_hold_ms = now_ms;
        s_manual_spin_decel_start_ms = 0;
        s_manual_spin_decel_start_speed = 0;
        if (s_manual_spin_dir != 1) {
          s_manual_spin_dir = 1;
          s_manual_spin_press_start_ms = now_ms;
        }
        const uint16_t ramp_speed = calc_ramp_up_speed(now_ms);
        command_manual_spin(1, ramp_speed);
        s_manual_spin_current_speed = ramp_speed;
      } else if (!floor_btn_pressed && (current_btn == BTN_NONE || prev_released || next_released) &&
                 s_manual_spin_dir != 0 &&
                 (uint32_t)(now_ms - s_manual_spin_last_hold_ms) >= kManualSpinReleaseGraceMs) {
        if (s_manual_spin_decel_start_ms == 0) {
          s_manual_spin_decel_start_ms = now_ms;
          s_manual_spin_decel_start_speed =
              (s_manual_spin_current_speed > 0) ? s_manual_spin_current_speed : manual_spin_speed_cap();
        }
        const uint16_t decel_speed = calc_ramp_down_speed(now_ms);
        if (decel_speed > 0) {
          command_manual_spin(s_manual_spin_dir, decel_speed);
          s_manual_spin_current_speed = decel_speed;
        } else {
          elevator_stop();
          s_manual_spin_dir = 0;
          s_manual_spin_current_speed = 0;
          s_manual_spin_decel_start_ms = 0;
          s_manual_spin_decel_start_speed = 0;
        }
      }
    }
  }

  now_ms = millis();
  run_motor_timer(now_ms);
  run_led_timer(now_ms);

  if (s_runtime_mode == 0) {
    now_ms = millis();
    scene_tick(now_ms);
  }
}
