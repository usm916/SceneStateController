#include "pi_link.h"
#include "pi_protocol.h"
#include <string.h>

static char s_line[128];
static uint8_t s_len = 0;

void pi_link_setup() {
  s_len = 0;
}

static bool parse_int(const char* s, int32_t& out) {
  char* endp = nullptr;
  long v = strtol(s, &endp, 10);
  if (endp == s) return false;
  out = (int32_t)v;
  return true;
}

bool pi_link_poll(Event& out) {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      s_line[s_len] = 0;
      s_len = 0;

      if (strncmp(s_line, "MOVE ", 5) == 0) {
        int32_t floor;
        if (parse_int(s_line + 5, floor)) {
          out.type = EVT_PI_CMD_MOVE;
          out.ts_ms = millis();
          out.data.move.target_floor = floor;
          return true;
        }
      } else if (strncmp(s_line, "LED ", 4) == 0) {
        int32_t pat;
        if (parse_int(s_line + 4, pat)) {
          out.type = EVT_PI_CMD_LED;
          out.ts_ms = millis();
          out.data.led.pattern_id = (uint8_t)pat;
          return true;
        }
      }
    } else {
      if (s_len < sizeof(s_line) - 1) s_line[s_len++] = c;
    }
  }
  return false;
}

void pi_link_send_event(const Event& e) {
  switch (e.type) {
    case EVT_IR_BUTTON:
      ssc_send_ir(Serial, e.data.ir.protocol, e.data.ir.addr, e.data.ir.cmd);
      break;
    case EVT_EV_ARRIVED:
      ssc_send_ev(Serial, "EV_ARRIVED");
      break;
    case EVT_EV_ERROR:
      ssc_send_err(Serial, e.data.error_code);
      ssc_send_ev(Serial, "EV_ERROR");
      break;
    default:
      break;
  }
}
