#include "serial_console.h"

#include "console_logger.h"
#include "ir_module.h"
#include <stdlib.h>
#include <string.h>

namespace {
char s_serial_line[24] = {0};
uint8_t s_serial_line_len = 0;

void clear_serial_line() {
  s_serial_line_len = 0;
  s_serial_line[0] = '\0';
}

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
  if (len == 1 && line[0] >= '0' && line[0] <= '3') {
    ir_set_decode_mode((uint8_t)(line[0] - '0'));
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
  if (out_event != nullptr) out_event->type = EVT_NONE;

  while (Serial.available()) {
    const char c = (char)Serial.read();
    log.print_serial_echo(c);
    if (c == '\r') continue;
    if (c == '\n') {
      const bool has_event =
          handle_serial_line(log, s_serial_line, s_serial_line_len, set_runtime_mode_fn, out_event);
      clear_serial_line();
      if (has_event) return true;
      continue;
    }
    if (s_serial_line_len < sizeof(s_serial_line) - 1) {
      s_serial_line[s_serial_line_len++] = c;
      s_serial_line[s_serial_line_len] = '\0';

      if (s_serial_line_len == 1 && s_serial_line[0] >= '0' && s_serial_line[0] <= '3') {
        handle_serial_line(log, s_serial_line, s_serial_line_len, set_runtime_mode_fn, out_event);
        clear_serial_line();
        continue;
      }
      if (s_serial_line_len == 2 &&
          (s_serial_line[0] == 's' || s_serial_line[0] == 'S') &&
          s_serial_line[1] >= '0' && s_serial_line[1] <= '4') {
        handle_serial_line(log, s_serial_line, s_serial_line_len, set_runtime_mode_fn, out_event);
        clear_serial_line();
      }
    }
  }
  return false;
}
