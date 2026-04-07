#include <Arduino.h>

#include "src/config.h"
#include "src/events.h"

#include "src/elevator_module.h"
#include "src/ir_module.h"
#include "src/led_module.h"
#include "src/pi_link.h"
#include "src/scene_controller.h"
#include "src/console_logger.h"


class ModeCommandParser {
 public:
  bool poll(Stream& io, uint8_t* out_mode) {
    while (io.available()) {
      const char p = (char)io.peek();
      if (!capturing_) {
        if (p != 's' && p != 'S') return false;
        capturing_ = true;
        len_ = 0;
      }

      const char c = (char)io.read();
      Serial.print("RX: ");
      if (c == '\r') {
        Serial.println("\\r");
      } else if (c == '\n') {
        Serial.println("\\n");
      } else {
        Serial.println(c);
      }
      if (c == '\r') continue;

      if (c == '\n') {
        buf_[len_] = '\0';
        capturing_ = false;

        const bool valid =
            (len_ == 2) &&
            (buf_[0] == 's' || buf_[0] == 'S') &&
            (buf_[1] >= '0' && buf_[1] <= '4');
        len_ = 0;

        if (valid) {
          *out_mode = (uint8_t)(buf_[1] - '0');
          return true;
        }
        invalid_ = true;
        return false;
      }

      if (len_ < sizeof(buf_) - 1) {
        buf_[len_++] = c;
      } else {
        capturing_ = false;
        len_ = 0;
        too_long_ = true;
        return false;
      }
    }
    return false;
  }

  bool consume_too_long() {
    const bool v = too_long_;
    too_long_ = false;
    return v;
  }

  bool consume_invalid() {
    const bool v = invalid_;
    invalid_ = false;
    return v;
  }

  bool is_capturing() const {
    return capturing_;
  }

 private:
  char buf_[16] = {0};
  uint8_t len_ = 0;
  bool capturing_ = false;
  bool too_long_ = false;
  bool invalid_ = false;
};

static ConsoleLogger s_log(Serial);
static char s_serial_line[24] = {0};
static uint8_t s_serial_line_len = 0;

static uint8_t s_runtime_mode = SSC_MODE;
static bool s_ir_ready = false;
static bool s_led_ready = false;
static bool s_elevator_ready = false;
static bool s_scene_ready = false;
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
  const uint8_t prev_mode = s_runtime_mode;
  s_runtime_mode = mode;
  ensure_modules_for_mode(mode);
  if (mode == 0) scene_setup();
  s_log.print_mode_change(prev_mode, s_runtime_mode);
}

static void handle_serial_line(const char* line, uint8_t len) {
  if (len == 2 && (line[0] == 's' || line[0] == 'S') && line[1] >= '0' && line[1] <= '4') {
    set_runtime_mode((uint8_t)(line[1] - '0'));
    return;
  }
  if (len == 1 && line[0] >= '0' && line[0] <= '3') {
    ir_set_decode_mode((uint8_t)(line[0] - '0'));
static void poll_mode_switch_from_serial() {
  if (!Serial.available()) return;

  const char p = (char)Serial.peek();
  if (!s_mode_parser.is_capturing() && p != 's' && p != 'S') return;

  uint8_t mode = 0;
  if (s_mode_parser.poll(Serial, &mode)) {
    set_runtime_mode(mode);
    return;
  }
  if (len >= sizeof(s_serial_line) - 1) {
    s_log.print_mode_cmd_too_long();
    return;
  }
  s_log.print_mode_usage();
}

static void poll_serial_console() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    s_log.print_serial_echo(c);
    if (c == '\r') continue;
    if (c == '\n') {
      handle_serial_line(s_serial_line, s_serial_line_len);
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

static void poll_unhandled_serial_echo() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    s_log.print_serial_echo(c);
  }
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
  poll_serial_console();
  poll_mode_switch_from_serial();
  ir_poll_serial_command();
  poll_unhandled_serial_echo();

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

  if (mode_is(1)) {
    Event e;
    if (ir_poll(e)) {
#if SSC_IR_LOG_ENABLE
      s_log.print_ir_event(e);
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
