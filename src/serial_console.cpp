#include "serial_console.h"

#include "console_logger.h"
#include "ir_module.h"

namespace {
char s_serial_line[24] = {0};
uint8_t s_serial_line_len = 0;

void handle_serial_line(ConsoleLogger& log, const char* line, uint8_t len,
                        void (*set_runtime_mode_fn)(uint8_t)) {
  if (len == 2 && (line[0] == 's' || line[0] == 'S') && line[1] >= '0' && line[1] <= '4') {
    set_runtime_mode_fn((uint8_t)(line[1] - '0'));
    return;
  }
  if (len == 1 && line[0] >= '0' && line[0] <= '3') {
    ir_set_decode_mode((uint8_t)(line[0] - '0'));
    return;
  }
  if (len >= sizeof(s_serial_line) - 1) {
    log.print_mode_cmd_too_long();
    return;
  }
  log.print_mode_usage();
}
}  // namespace

void serial_console_poll(ConsoleLogger& log, void (*set_runtime_mode_fn)(uint8_t)) {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    log.print_serial_echo(c);
    if (c == '\r') continue;
    if (c == '\n') {
      handle_serial_line(log, s_serial_line, s_serial_line_len, set_runtime_mode_fn);
      s_serial_line_len = 0;
      s_serial_line[0] = '\0';
      continue;
    }
    if (s_serial_line_len < sizeof(s_serial_line) - 1) {
      s_serial_line[s_serial_line_len++] = c;
      s_serial_line[s_serial_line_len] = '\0';
    }
  }
}
