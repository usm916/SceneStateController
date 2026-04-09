#include "shared_serial.h"

#include <string.h>

namespace {
constexpr uint8_t kSharedSerialLineCount = 32;
constexpr uint8_t kSharedSerialLineMaxLen = 127;

char s_build_line[kSharedSerialLineMaxLen + 1] = {0};
uint8_t s_build_len = 0;

char s_lines[kSharedSerialLineCount][kSharedSerialLineMaxLen + 1] = {{0}};
uint16_t s_line_seq[kSharedSerialLineCount] = {0};
uint8_t s_write_index = 0;
uint16_t s_next_seq = 0;

void push_line(const char* line) {
  strncpy(s_lines[s_write_index], line, kSharedSerialLineMaxLen);
  s_lines[s_write_index][kSharedSerialLineMaxLen] = '\0';
  s_line_seq[s_write_index] = s_next_seq++;
  s_write_index = (uint8_t)((s_write_index + 1) % kSharedSerialLineCount);
}

bool try_get_line_by_seq(uint16_t seq, const char** out_line) {
  for (uint8_t i = 0; i < kSharedSerialLineCount; ++i) {
    if (s_line_seq[i] == seq) {
      *out_line = s_lines[i];
      return true;
    }
  }
  return false;
}
}  // namespace

void shared_serial_setup() {
  memset(s_line_seq, 0xFF, sizeof(s_line_seq));
  s_build_len = 0;
  s_build_line[0] = '\0';
  s_write_index = 0;
  s_next_seq = 0;
}

void shared_serial_pump() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      s_build_line[s_build_len] = '\0';
      if (s_build_len > 0) {
        push_line(s_build_line);
      }
      s_build_len = 0;
      s_build_line[0] = '\0';
      continue;
    }

    if (s_build_len < kSharedSerialLineMaxLen) {
      s_build_line[s_build_len++] = c;
      s_build_line[s_build_len] = '\0';
    }
  }
}

void shared_serial_cursor_init(SharedSerialCursor* cursor) {
  if (cursor == nullptr) return;
  const uint16_t oldest_available =
      (s_next_seq > kSharedSerialLineCount) ? (uint16_t)(s_next_seq - kSharedSerialLineCount) : 0;
  cursor->next_seq = oldest_available;
}

bool shared_serial_read_line(SharedSerialCursor* cursor, char* out_line, size_t out_size) {
  if (cursor == nullptr || out_line == nullptr || out_size == 0) return false;

  const uint16_t oldest_available =
      (s_next_seq > kSharedSerialLineCount) ? (uint16_t)(s_next_seq - kSharedSerialLineCount) : 0;
  if (cursor->next_seq < oldest_available) {
    cursor->next_seq = oldest_available;
  }

  const char* line = nullptr;
  if (!try_get_line_by_seq(cursor->next_seq, &line)) {
    return false;
  }

  strncpy(out_line, line, out_size - 1);
  out_line[out_size - 1] = '\0';
  cursor->next_seq++;
  return true;
}
