#include "ir_module.h"
#include "config.h"
#include <IRremote.hpp>

static void log_ir_decoded(const IRData& d) {
  Serial.print("IR RX protocol=");
  Serial.print((uint8_t)d.protocol);
  Serial.print(" addr=0x");
  Serial.print((uint16_t)d.address, HEX);
  Serial.print(" cmd=0x");
  Serial.println((uint16_t)d.command, HEX);
}

void ir_setup() {
  IrReceiver.begin(SSC_PIN_IR, DISABLE_LED_FEEDBACK);
}

bool ir_poll(Event& out) {
  if (!IrReceiver.decode()) return false;

  auto &d = IrReceiver.decodedIRData;
  out.type = EVT_IR_BUTTON;
  out.ts_ms = millis();
  out.data.ir.protocol = (uint8_t)d.protocol;
  out.data.ir.addr = (uint16_t)d.address;
  out.data.ir.cmd  = (uint16_t)d.command;
  log_ir_decoded(d);

  IrReceiver.resume();
  return true;
}
