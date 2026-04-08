#include "serial_console.h"

#include "console_logger.h"
#include "ir_module.h"
#include "shared_serial.h"
#include "tmc2209_module.h"
#include <stdlib.h>
#include <string.h>

namespace {
char s_serial_line[24] = {0};
SharedSerialCursor s_serial_cursor = {0};
bool s_cursor_initialized = false;

bool parse_int(const char* s, int32_t& out) {
  char* endp = nullptr;
  const long v = strtol(s, &endp, 10);
  if (endp == s) return false;
  out = (int32_t)v;
  return true;
}

bool handle_serial_line(ConsoleLogger& log, const char* line, uint8_t len,
                        void (*set_runtime_mode_fn)(uint8_t), Event* out_event) {
  if (len == 2 && (line[0] == 's' || line[0] == 'S') && line[1] >= '0' && line[1] <= '4') {
    set_runtime_mode_fn((uint8_t)(line[1] - '0'));
    return false;
  }
  if (len == 2 && (line[0] == 'm' || line[0] == 'M') && line[1] >= '0' && line[1] <= '3') {
    ir_set_decode_mode((uint8_t)(line[1] - '0'));
    return false;
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
  if (len >= 11 && strncmp(line, "TMC MOTOR ", 10) == 0) {
    int32_t current_ma = 0;
    if (parse_int(line + 10, current_ma) && current_ma > 0 && current_ma <= 2000) {
      tmc2209_set_motor_current_ma((uint16_t)current_ma);
      Serial.print("TMC motor_current(mA)=");
      Serial.println(tmc2209_motor_current_ma());
      return false;
    }
  }
  if (len >= 9 && strncmp(line, "TMC RUN ", 8) == 0) {
    int32_t current_ma = 0;
    if (parse_int(line + 8, current_ma) && current_ma > 0 && current_ma <= 2000) {
      tmc2209_set_run_current_ma((uint16_t)current_ma);
      Serial.print("TMC run_current(mA)=");
      Serial.println(tmc2209_run_current_ma());
      return false;
    }
  }
  if (len >= 10 && strncmp(line, "TMC HOLD ", 9) == 0) {
    int32_t hold_pct = 0;
    if (parse_int(line + 9, hold_pct) && hold_pct >= 0 && hold_pct <= 100) {
      tmc2209_set_hold_current_pct((uint8_t)hold_pct);
      Serial.print("TMC hold_current(%)=");
      Serial.println(tmc2209_hold_current_pct());
      return false;
    }
  }
  if (len == 8 && strncmp(line, "TMC INFO", 8) == 0) {
    Serial.print("TMC motor_current(mA)=");
    Serial.println(tmc2209_motor_current_ma());
    Serial.print("TMC run_current(mA)=");
    Serial.println(tmc2209_run_current_ma());
    Serial.print("TMC hold_current(%)=");
    Serial.println(tmc2209_hold_current_pct());
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
