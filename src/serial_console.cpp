#include "serial_console.h"

#include "console_logger.h"
#include "elevator_module.h"
#include "ir_module.h"
#include "scene_controller.h"
#include "led_module.h"
#include "config.h"
#include "shared_serial.h"
#include "tmc2209_module.h"
#include "button_position_store.h"
#include "espnow_link.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

namespace {
char s_serial_line[64] = {0};
SharedSerialCursor s_serial_cursor = {0};
bool s_cursor_initialized = false;

bool parse_int(const char* s, int32_t& out) {
  char* endp = nullptr;
  const long v = strtol(s, &endp, 10);
  if (endp == s) return false;
  out = (int32_t)v;
  return true;
}

bool parse_rec_command(const char* line, uint8_t len, uint8_t* out_index,
                       int32_t* out_relative_steps, bool* out_has_relative) {
  if (line == nullptr || out_index == nullptr || out_relative_steps == nullptr || out_has_relative == nullptr) return false;
  if (len < 5 || strncmp(line, "rec_", 4) != 0) return false;

  const char* payload = line + 4;
  const char* sep = strchr(payload, '_');

  char btn_token[12] = {0};
  if (sep == nullptr) {
    strncpy(btn_token, payload, sizeof(btn_token) - 1);
    *out_has_relative = false;
  } else {
    const size_t btn_len = (size_t)(sep - payload);
    if (btn_len == 0 || btn_len >= sizeof(btn_token)) return false;
    memcpy(btn_token, payload, btn_len);
    btn_token[btn_len] = '\0';
    *out_has_relative = true;
    if (!parse_int(sep + 1, *out_relative_steps)) return false;
  }

  return button_position_store_parse_button_token(btn_token, out_index);
}

bool parse_led_scene_token(const char* token, LedStripScene* out_scene) {
  if (token == nullptr || out_scene == nullptr) return false;
  if (strcmp(token, "SOLID") == 0 || strcmp(token, "solid") == 0) {
    *out_scene = LEDSCENE_SOLID;
    return true;
  }
  if (strcmp(token, "CHASE") == 0 || strcmp(token, "chase") == 0) {
    *out_scene = LEDSCENE_CHASE;
    return true;
  }
  if (strcmp(token, "BLINK") == 0 || strcmp(token, "blink") == 0) {
    *out_scene = LEDSCENE_BLINK;
    return true;
  }
  if (strcmp(token, "RANDOM") == 0 || strcmp(token, "random") == 0 ||
      strcmp(token, "RANDOM_LONG") == 0 || strcmp(token, "random_long") == 0) {
    *out_scene = LEDSCENE_RANDOM_LONG_BLINK_THEN_ON;
    return true;
  }
  if (strcmp(token, "CRASH") == 0 || strcmp(token, "crash") == 0) {
    *out_scene = LEDSCENE_CRASH;
    return true;
  }
  if (strcmp(token, "EMERGENCY") == 0 || strcmp(token, "emergency") == 0) {
    *out_scene = LEDSCENE_EMERGENCY_RED;
    return true;
  }
  if (strcmp(token, "BLACKOUT") == 0 || strcmp(token, "blackout") == 0 ||
      strcmp(token, "DARK") == 0 || strcmp(token, "dark") == 0) {
    *out_scene = LEDSCENE_BLACKOUT;
    return true;
  }
  if (strcmp(token, "FADEIN3S") == 0 || strcmp(token, "fadein3s") == 0 ||
      strcmp(token, "FADE_IN_3S") == 0 || strcmp(token, "fade_in_3s") == 0) {
    *out_scene = LEDSCENE_FADE_IN_3S;
    return true;
  }
  if (strcmp(token, "FADEOUT3S") == 0 || strcmp(token, "fadeout3s") == 0 ||
      strcmp(token, "FADE_OUT_3S") == 0 || strcmp(token, "fade_out_3s") == 0) {
    *out_scene = LEDSCENE_FADE_OUT_3S;
    return true;
  }
  return false;
}

bool parse_scene_token(const char* token, SceneId* out_scene) {
  if (token == nullptr || out_scene == nullptr) return false;
  if (strcmp(token, "IDLE") == 0 || strcmp(token, "idle") == 0 || strcmp(token, "SCENE_IDLE") == 0) {
    *out_scene = SCENE_IDLE;
    return true;
  }
  if (strcmp(token, "MOVE") == 0 || strcmp(token, "move") == 0 || strcmp(token, "SCENE_MOVE") == 0) {
    *out_scene = SCENE_MOVE;
    return true;
  }
  if (strcmp(token, "ARRIVED") == 0 || strcmp(token, "arrived") == 0 || strcmp(token, "SCENE_ARRIVED") == 0) {
    *out_scene = SCENE_ARRIVED;
    return true;
  }
  if (strcmp(token, "ERROR") == 0 || strcmp(token, "error") == 0 || strcmp(token, "SCENE_ERROR") == 0) {
    *out_scene = SCENE_ERROR;
    return true;
  }
  return false;
}

void print_system_info() {
  Serial.println("=== SSC INFO ===");
  Serial.print("scene=");
  Serial.println(scene_name(scene_current()));
  Serial.print("elevator_state=");
  Serial.println(ev_state_name(elevator_state()));
  Serial.print("floor_current=");
  Serial.println(elevator_floor());
  Serial.print("floor_target=");
  Serial.println(elevator_target_floor());
  Serial.print("pos_current_steps=");
  Serial.println(elevator_current_position_steps());
  Serial.print("pos_target_steps=");
  Serial.println(elevator_target_position_steps());
  Serial.print("distance_to_go_steps=");
  Serial.println(elevator_distance_to_go_steps());
  Serial.print("moving=");
  Serial.println(elevator_is_moving() ? 1 : 0);
  Serial.print("motor_lag_count=");
  Serial.println(elevator_motor_lag_count());
  Serial.print("motor_lag_last_interval_ms=");
  Serial.println(elevator_motor_lag_last_interval_ms());
  Serial.print("motor_lag_max_interval_ms=");
  Serial.println(elevator_motor_lag_max_interval_ms());
  Serial.print("motor_lag_accumulated_ms=");
  Serial.println(elevator_motor_lag_accumulated_ms());
  Serial.print("led_global_brightness_pct=");
  Serial.println(led_global_brightness_pct());
  Serial.print("calibration_valid=");
  Serial.println(elevator_has_valid_calibration() ? 1 : 0);
  Serial.print("calibration_in_progress=");
  Serial.println(elevator_calibration_in_progress() ? 1 : 0);
  Serial.print("homed_zero=");
  Serial.println(elevator_is_homed_zero() ? 1 : 0);
  Serial.print("top_limit_steps=");
  Serial.println(elevator_top_limit_steps());
  Serial.print("top_margin_steps=");
  Serial.println(elevator_top_margin_steps());
  Serial.print("bottom_margin_steps=");
  Serial.println(elevator_bottom_margin_steps());
  Serial.print("tmc_run_current_ma=");
  Serial.println(tmc2209_run_current_ma());
  Serial.print("tmc_hold_current_pct=");
  Serial.println(tmc2209_hold_current_pct());
  Serial.print("move_max_speed_steps_per_sec=");
  Serial.println(elevator_move_max_speed());
  Serial.print("move_accel_steps_per_sec2=");
  Serial.println(elevator_move_acceleration());
  button_position_store_print_info(Serial);
  Serial.println("=== /SSC INFO ===");
}

bool handle_serial_line(ConsoleLogger& log, const char* line, uint8_t len,
                        void (*set_runtime_mode_fn)(uint8_t), Event* out_event) {
  if (len >= 2 && (line[0] == 's' || line[0] == 'S')) {
    int32_t mode_mask = 0;
    if (parse_int(line + 1, mode_mask) && mode_mask >= 0 && mode_mask <= 15) {
      set_runtime_mode_fn((uint8_t)mode_mask);
      return false;
    }
    Serial.println("MODE_CMD expects s<0..15> (bitmask: 1=IR 2=LED 4=EV 8=SCENE, 0=ALL)");
    return false;
  }
  if (len == 2 && (line[0] == 'm' || line[0] == 'M') && line[1] >= '0' && line[1] <= '3') {
    ir_set_decode_mode((uint8_t)(line[1] - '0'));
    return false;
  }
  if (len >= 2 && (line[0] == 'e' || line[0] == 'E')) {
    int32_t target_steps = 0;
    if (parse_int(line + 1, target_steps)) {
      handleInput(target_steps);
      return false;
    }
  }
  if (len >= 6 && strncmp(line, "MOVE ", 5) == 0 && out_event != nullptr) {
    int32_t floor = 0;
    if (parse_int(line + 5, floor)) {
      out_event->type = EVT_PI_CMD_MOVE;
      out_event->ts_ms = millis();
      out_event->data.move.target_floor = floor;
      return true;
    }
  }
  if (len >= 5 && strncmp(line, "LED ", 4) == 0 && out_event != nullptr) {
    int32_t pattern = 0;
    if (parse_int(line + 4, pattern)) {
      out_event->type = EVT_PI_CMD_LED;
      out_event->ts_ms = millis();
      out_event->data.led.pattern_id = (uint8_t)pattern;
      return true;
    }
  }
  if (len >= 10 && (strncmp(line, "LEDSCENE ", 9) == 0 || strncmp(line, "ledscene ", 9) == 0)) {
    char scope[8] = {0};
    char sceneToken[16] = {0};
    if (sscanf(line + 9, "%7s %15s", scope, sceneToken) == 2) {
      LedStripScene scene = LEDSCENE_SOLID;
      if (!parse_led_scene_token(sceneToken, &scene)) {
        Serial.println("LEDSCENE scene must be SOLID/CHASE/BLINK/RANDOM/CRASH/EMERGENCY/BLACKOUT/FADEIN3S/FADEOUT3S");
        return false;
      }

      if (strcmp(scope, "ALL") == 0 || strcmp(scope, "all") == 0) {
        for (uint8_t strip = 0; strip < SSC_LED_STRIP_COUNT; ++strip) {
          led_set_strip_scene(strip, scene);
        }
        if (espnow_link_is_manager()) {
          (void)espnow_link_send_all_scene(scene);
        }
        Serial.println("LEDSCENE ALL updated");
        return false;
      }

      int32_t stripIndex = 0;
      if (parse_int(scope, stripIndex) && stripIndex >= 0 && stripIndex < SSC_LED_STRIP_COUNT) {
        led_set_strip_scene((uint8_t)stripIndex, scene);
        if (espnow_link_is_manager()) {
          (void)espnow_link_send_strip_scene((uint8_t)stripIndex, scene);
        }
        Serial.print("LEDSCENE strip=");
        Serial.print(stripIndex);
        Serial.println(" updated");
        return false;
      }
    }
    Serial.println("LEDSCENE usage: LEDSCENE <0..5|ALL> <SOLID|CHASE|BLINK|RANDOM|CRASH|EMERGENCY|BLACKOUT|FADEIN3S|FADEOUT3S>");
    return false;
  }
  if (len >= 7 && (strncmp(line, "SCENE ", 6) == 0 || strncmp(line, "scene ", 6) == 0)) {
    SceneId scene = SCENE_IDLE;
    if (!parse_scene_token(line + 6, &scene)) {
      Serial.println("SCENE usage: SCENE <IDLE|MOVE|ARRIVED|ERROR>");
      return false;
    }
    if (!scene_select(scene)) {
      Serial.println("SCENE apply failed");
      return false;
    }
    Serial.print("SCENE ");
    Serial.println(scene_name(scene_current()));
    return false;
  }
  if ((len >= 12 && strncmp(line, "brightness_", 11) == 0) ||
      (len >= 12 && strncmp(line, "BRIGHTNESS_", 11) == 0)) {
    int32_t brightness_pct = 0;
    if (parse_int(line + 11, brightness_pct) && brightness_pct >= 0 && brightness_pct <= 100) {
      led_set_global_brightness_pct((uint8_t)brightness_pct);
      if (espnow_link_is_manager()) {
        (void)espnow_link_send_brightness_pct((uint8_t)brightness_pct);
      }
      Serial.print("brightness=");
      Serial.println(led_global_brightness_pct());
      return false;
    }
    Serial.println("brightness format: brightness_<0..100>");
    return false;
  }
  if (len >= 9 && strncmp(line, "TMC RUN ", 8) == 0) {
    int32_t current_ma = 0;
    if (parse_int(line + 8, current_ma) && current_ma > 0 && current_ma <= 2000) {
      tmc2209_set_run_current_ma((uint16_t)current_ma);
      elevator_set_motor_run_current_ma((uint16_t)current_ma);
      Serial.print("TMC run_current(mA)=");
      Serial.println(tmc2209_run_current_ma());
      return false;
    }
  }
  if (len >= 10 && strncmp(line, "TMC HOLD ", 9) == 0) {
    int32_t hold_pct = 0;
    if (parse_int(line + 9, hold_pct) && hold_pct >= 0 && hold_pct <= 100) {
      tmc2209_set_hold_current_pct((uint8_t)hold_pct);
      elevator_set_motor_hold_current_pct((uint8_t)hold_pct);
      Serial.print("TMC hold_current(%)=");
      Serial.println(tmc2209_hold_current_pct());
      return false;
    }
  }
  if (len == 8 && strncmp(line, "TMC INFO", 8) == 0) {
    Serial.print("TMC run_current(mA)=");
    Serial.println(tmc2209_run_current_ma());
    Serial.print("TMC hold_current(%)=");
    Serial.println(tmc2209_hold_current_pct());
    return false;
  }
  if ((len == 4 && strncmp(line, "INFO", 4) == 0) ||
      (len == 4 && strncmp(line, "info", 4) == 0)) {
    print_system_info();
    return false;
  }
  if ((len == 4 && strncmp(line, "mute", 4) == 0) ||
      (len == 4 && strncmp(line, "MUTE", 4) == 0)) {
    button_position_store_set_zero(elevator_current_position_steps());
    Serial.println("mute: current position set to zero base.");
    return false;
  }
  if ((len == 4 && strncmp(line, "help", 4) == 0) ||
      (len == 4 && strncmp(line, "HELP", 4) == 0)) {
    log.print_mode_usage();
    return false;
  }
  if ((len == 9 && strncmp(line, "save_pref", 9) == 0) ||
      (len == 9 && strncmp(line, "SAVE_PREF", 9) == 0)) {
    const bool btn_saved = button_position_store_save();
    const bool motion_saved = elevator_save_motion_profile();
    const bool tmc_saved = tmc2209_save_current_settings();
    Serial.println((btn_saved && motion_saved && tmc_saved) ? "save_pref:ok" : "save_pref:failed");
    return false;
  }
  if ((len >= 9 && strncmp(line, "current_", 8) == 0) ||
      (len >= 9 && strncmp(line, "CURRENT_", 8) == 0)) {
    int32_t current_ma = 0;
    if (parse_int(line + 8, current_ma) && current_ma > 0 && current_ma <= 2000) {
      tmc2209_set_run_current_ma((uint16_t)current_ma);
      elevator_set_motor_run_current_ma((uint16_t)current_ma);
      (void)tmc2209_save_current_settings();
      Serial.print("current_");
      Serial.println(tmc2209_run_current_ma());
      return false;
    }
    Serial.println("current format: current_<1..2000>");
    return false;
  }
  if ((len >= 7 && strncmp(line, "speed_", 6) == 0) ||
      (len >= 7 && strncmp(line, "SPEED_", 6) == 0)) {
    int32_t speed = 0;
    if (parse_int(line + 6, speed) && speed > 0) {
      if (elevator_set_move_max_speed((float)speed)) {
        (void)elevator_save_motion_profile();
        Serial.print("speed=");
        Serial.println((int32_t)elevator_move_max_speed());
      }
      return false;
    }
    Serial.println("speed format: speed_<steps_per_sec>");
    return false;
  }
  if ((len >= 7 && strncmp(line, "accel_", 6) == 0) ||
      (len >= 7 && strncmp(line, "ACCEL_", 6) == 0)) {
    int32_t accel = 0;
    if (parse_int(line + 6, accel) && accel > 0) {
      if (elevator_set_move_acceleration((float)accel)) {
        (void)elevator_save_motion_profile();
        Serial.print("accel=");
        Serial.println((int32_t)elevator_move_acceleration());
      }
      return false;
    }
    Serial.println("accel format: accel_<steps_per_sec2>");
    return false;
  }
  if ((len >= 5 && strncmp(line, "rec_", 4) == 0) ||
      (len >= 5 && strncmp(line, "REC_", 4) == 0)) {
    char normalized[24] = {0};
    const uint8_t max_len = (len < sizeof(normalized) - 1) ? len : (sizeof(normalized) - 1);
    for (uint8_t i = 0; i < max_len; ++i) {
      char c = line[i];
      if (c >= 'A' && c <= 'Z') c = (char)(c - 'A' + 'a');
      normalized[i] = c;
    }

    uint8_t btn_index = 0;
    int32_t relative_steps = 0;
    bool has_relative = false;
    if (parse_rec_command(normalized, (uint8_t)strlen(normalized), &btn_index, &relative_steps, &has_relative)) {
      if (has_relative) {
        button_position_store_record_relative(btn_index, relative_steps);
      } else {
        button_position_store_record_current(btn_index, elevator_current_position_steps());
        button_position_store_relative(btn_index, &relative_steps);
      }
      Serial.print("rec_");
      Serial.print(btn_index);
      Serial.print("=");
      Serial.println(relative_steps);
      return false;
    }

    Serial.println("rec format: rec_<0..9> or rec_<0..9>_<steps> (BTN_x accepted)");
    return false;
  }
  if (len >= sizeof(s_serial_line) - 1) {
    log.print_mode_cmd_too_long();
    return false;
  }
  log.print_mode_usage();
  return false;
}
}  // namespace

bool serial_console_poll(ConsoleLogger& log,
                         void (*set_runtime_mode_fn)(uint8_t),
                         Event* out_event) {
  if (!s_cursor_initialized) {
    shared_serial_cursor_init(&s_serial_cursor);
    s_cursor_initialized = true;
  }

  if (out_event != nullptr) out_event->type = EVT_NONE;

  while (shared_serial_read_line(&s_serial_cursor, s_serial_line, sizeof(s_serial_line))) {
    const uint8_t len = (uint8_t)strlen(s_serial_line);
    const bool has_event =
        handle_serial_line(log, s_serial_line, len, set_runtime_mode_fn, out_event);
    if (has_event) return true;
  }

  return false;
}
