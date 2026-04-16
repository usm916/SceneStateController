#include "ir_module.h"
#include "config.h"
#include <IRremote.hpp>

static uint8_t s_decode_mode = 0; // 0=AUTO 1=NEC 2=AEHA 3=SONY
static RemoteButton s_active_btn = BTN_NONE;
static RemoteButton s_released_btn = BTN_NONE;
static uint32_t s_active_btn_until_ms = 0;
static constexpr uint16_t kIrHoldMs = 200;

static void update_button_lifecycle(uint32_t now_ms) {
  if (s_active_btn != BTN_NONE && now_ms > s_active_btn_until_ms) {
    s_released_btn = s_active_btn;
    s_active_btn = BTN_NONE;
    s_active_btn_until_ms = 0;
  }
}

static const char* mode_name(uint8_t m) {
  switch (m) {
    case 0: return "AUTO";
    case 1: return "NEC";
    case 2: return "AEHA";
    case 3: return "SONY";
    default: return "?";
  }
}

static bool protocol_match(decode_type_t proto) {
  if (s_decode_mode == 0) return true;
  switch (s_decode_mode) {
    case 1: return proto == NEC;
    case 2: return proto == PANASONIC || proto == JVC;
    case 3: return proto == SONY;
    default: return true;
  }
}

static bool is_remote_mapping_protocol(decode_type_t proto) {
  return proto == NEC || (uint8_t)proto == 7;
}

static bool is_remote_mapping_hit(const IRData& d) {
  return is_remote_mapping_protocol(d.protocol) && d.address == 0x00;
}

static void print_raw_timing() {
  Serial.print("RAW(us): ");
  for (uint16_t i = 1; i < IrReceiver.irparams.rawlen; i++) {
    const uint32_t us = (uint32_t)IrReceiver.irparams.rawbuf[i] * USECPERTICK;
    Serial.print((i & 1) ? "M" : "S");
    Serial.print(us);
    if (i + 1 < IrReceiver.irparams.rawlen) Serial.print(", ");
  }
  Serial.println();
}

static void print_menu() {
  Serial.println();
  Serial.println("=== IR Decode Mode ===");
  Serial.println("0 : AUTO");
  Serial.println("1 : NEC");
  Serial.println("2 : AEHA");
  Serial.println("3 : SONY");
  Serial.println("======================");
}

void ir_set_decode_mode(uint8_t mode) {
  if (mode > 3) return;
  s_decode_mode = mode;
  Serial.print("Mode set to: ");
  Serial.println(mode_name(s_decode_mode));
  print_menu();
}

void ir_setup() {
  IrReceiver.begin(SSC_PIN_IR, DISABLE_LED_FEEDBACK);
  Serial.println();
  Serial.println("===== ESP32 IR Debug Boot =====");
  Serial.print("IR pin: GPIO");
  Serial.println(SSC_PIN_IR);
  Serial.println("IRremote 4.x detected");
  Serial.println("Commands: 0(AUTO) 1(NEC) 2(AEHA) 3(SONY)");
  Serial.println("Raw timing is always printed");
  Serial.println("================================");
  print_menu();
}

bool ir_poll(Event& out) {
  update_button_lifecycle(millis());
  if (!IrReceiver.decode()) return false;

  auto &d = IrReceiver.decodedIRData;
  const bool matched = protocol_match(d.protocol);
  const bool map_hit = is_remote_mapping_hit(d);
  // Serial.print("MODE=");
  // Serial.print(mode_name(s_decode_mode));
  // Serial.print("  protocol=");
  // Serial.print(getProtocolString(d.protocol));

  if (matched) {
    // Serial.print("  address=0x");
    // Serial.print((uint16_t)d.address, HEX);
    // Serial.print("  command=0x");
    // Serial.println((uint16_t)d.command, HEX);

    out.type = EVT_IR_BUTTON;
    out.ts_ms = millis();
    out.data.ir.protocol = (uint8_t)d.protocol;
    out.data.ir.addr = (uint16_t)d.address;
    out.data.ir.cmd  = (uint16_t)d.command;

    if (map_hit) {
      const RemoteButton next_btn = (RemoteButton)(uint8_t)d.command;
      if (s_active_btn != BTN_NONE && s_active_btn != next_btn) {
        s_released_btn = s_active_btn;
      }
      s_active_btn = next_btn;
      s_active_btn_until_ms = millis() + kIrHoldMs;
    }
  } else {
    Serial.println("  (filtered)");
  }

  // print_raw_timing();

  IrReceiver.resume();
  return matched;
}

bool ir_btn(RemoteButton btn) {
  const uint32_t now_ms = millis();
  update_button_lifecycle(now_ms);
  return now_ms <= s_active_btn_until_ms && s_active_btn == btn;
}

bool ir_btn_released(RemoteButton btn) {
  update_button_lifecycle(millis());
  if (s_released_btn == btn) {
    s_released_btn = BTN_NONE;
    return true;
  }
  return false;
}

bool ir_any_btn() {
  update_button_lifecycle(millis());
  return s_active_btn != BTN_NONE;
}

RemoteButton ir_active_btn() {
  update_button_lifecycle(millis());
  return s_active_btn;
}

RemoteButton ir_get_latest_button() {
  Event ignored = {};
  ignored.type = EVT_NONE;
  (void)ir_poll(ignored);
  return ir_active_btn();
}

void ir_inject_button(RemoteButton btn, uint16_t hold_ms) {
  const uint32_t now_ms = millis();
  update_button_lifecycle(now_ms);

  if (btn == BTN_NONE) {
    if (s_active_btn != BTN_NONE) {
      s_released_btn = s_active_btn;
    }
    s_active_btn = BTN_NONE;
    s_active_btn_until_ms = 0;
    return;
  }

  if (s_active_btn != BTN_NONE && s_active_btn != btn) {
    s_released_btn = s_active_btn;
  }
  s_active_btn = btn;
  s_active_btn_until_ms = now_ms + ((hold_ms > 0) ? hold_ms : kIrHoldMs);
}
