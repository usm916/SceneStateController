#include "ir_module.h"
#include "config.h"
#include <IRremote.hpp>

static uint8_t s_decode_mode = 0; // 0=AUTO 1=NEC 2=AEHA 3=SONY

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

void ir_poll_serial_command() {
  if (!Serial.available()) return;
  const char p = (char)Serial.peek();
  if (p < '0' || p > '3') return;

  const char c = (char)Serial.read();
  Serial.print("RX: ");
  Serial.println(c);
  while (Serial.available()) {
    const char t = (char)Serial.peek();
    if (t == '\r' || t == '\n') {
      (void)Serial.read();
    } else {
      break;
    }
  }

  s_decode_mode = (uint8_t)(c - '0');
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
  if (!IrReceiver.decode()) return false;

  auto &d = IrReceiver.decodedIRData;
  const bool matched = protocol_match(d.protocol);
  Serial.print("MODE=");
  Serial.print(mode_name(s_decode_mode));
  Serial.print("  protocol=");
  Serial.print(getProtocolString(d.protocol));

  if (matched) {
    Serial.print("  address=0x");
    Serial.print((uint16_t)d.address, HEX);
    Serial.print("  command=0x");
    Serial.println((uint16_t)d.command, HEX);

    out.type = EVT_IR_BUTTON;
    out.ts_ms = millis();
    out.data.ir.protocol = (uint8_t)d.protocol;
    out.data.ir.addr = (uint16_t)d.address;
    out.data.ir.cmd  = (uint16_t)d.command;
  } else {
    Serial.println("  (filtered)");
  }

  print_raw_timing();

  IrReceiver.resume();
  return matched;
}
