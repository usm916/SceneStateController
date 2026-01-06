#include "ir_module.h"
#include "config.h"
#include <IRremote.hpp>

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

  IrReceiver.resume();
  return true;
}
