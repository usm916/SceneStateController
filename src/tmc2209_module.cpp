#include "tmc2209_module.h"
#include "config.h"
#include <Arduino.h>
#include <TMCStepper.h>

static HardwareSerial& s_ser = Serial1;
static constexpr float RSENSE = 0.11f;
static TMC2209Stepper s_drv(&s_ser, RSENSE, 0);

void tmc2209_setup() {
  Serial.println("Setting up elevator...0");
  #if SSC_TMC_UART_ONEWIRE
  s_ser.begin(SSC_TMC_UART_BAUD, SERIAL_8N1, SSC_TMC_UART_ONEWIRE_PIN, SSC_TMC_UART_ONEWIRE_PIN);
  #else
  s_ser.begin(SSC_TMC_UART_BAUD, SERIAL_8N1, SSC_TMC_UART_RX_PIN, SSC_TMC_UART_TX_PIN);
  #endif
  pinMode(SSC_PIN_EN, OUTPUT);
  digitalWrite(SSC_PIN_EN, LOW);
  delay(50);
  s_drv.begin();
  tmc2209_configure_defaults();
}

void tmc2209_configure_defaults() {
  s_drv.pdn_disable(true);
  s_drv.I_scale_analog(false);
  s_drv.toff(4);
  s_drv.blank_time(24);
  s_drv.rms_current(650);
  s_drv.microsteps(16);
  s_drv.en_spreadCycle(false);
  s_drv.TPWMTHRS(0);
}

void tmc2209_set_enable(bool en) {
  digitalWrite(SSC_PIN_EN, en ? LOW : HIGH);
}

uint16_t tmc2209_status() {
  return s_drv.DRV_STATUS();
}
