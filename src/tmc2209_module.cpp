#include "tmc2209_module.h"
#include "config.h"
#include <Arduino.h>
#include <Preferences.h>
#include <TMCStepper.h>

static HardwareSerial& s_ser = Serial1;
static constexpr float RSENSE = SSC_TMC_RSENSE_OHM;
static TMC2209Stepper s_drv(&s_ser, RSENSE, 0);
static uint16_t s_run_current_ma = SSC_TMC_MOTOR_CURRENT_MA;
static uint8_t s_hold_current_pct = SSC_TMC_HOLD_CURRENT_PCT;
static Preferences s_prefs;

static constexpr char kPrefsNs[] = "tmc2209";
static constexpr char kPrefsVerKey[] = "ver";
static constexpr char kPrefsRunKey[] = "run_ma";
static constexpr char kPrefsHoldKey[] = "hold_pct";
static constexpr uint32_t kPrefsVersion = 1;
static constexpr uint16_t kRunCurrentMinMa = 1;
static constexpr uint16_t kRunCurrentMaxMa = 2000;

static uint16_t clamp_run_current_ma(uint16_t current_ma) {
  if (current_ma < kRunCurrentMinMa) return kRunCurrentMinMa;
  if (current_ma > kRunCurrentMaxMa) return kRunCurrentMaxMa;
  return current_ma;
}

static uint8_t clamp_hold_pct(uint8_t hold_pct) {
  return hold_pct > 100 ? 100 : hold_pct;
}

static void apply_current_settings() {
  s_run_current_ma = clamp_run_current_ma(s_run_current_ma);
  s_hold_current_pct = clamp_hold_pct(s_hold_current_pct);
  s_drv.rms_current(s_run_current_ma, s_hold_current_pct);
}

static void load_current_settings() {
  s_run_current_ma = SSC_TMC_MOTOR_CURRENT_MA;
  s_hold_current_pct = SSC_TMC_HOLD_CURRENT_PCT;

  if (!s_prefs.begin(kPrefsNs, true)) return;
  const uint32_t version = s_prefs.getUInt(kPrefsVerKey, 0);
  if (version == kPrefsVersion) {
    s_run_current_ma = clamp_run_current_ma((uint16_t)s_prefs.getUShort(kPrefsRunKey, SSC_TMC_MOTOR_CURRENT_MA));
    s_hold_current_pct = clamp_hold_pct((uint8_t)s_prefs.getUChar(kPrefsHoldKey, SSC_TMC_HOLD_CURRENT_PCT));
  }
  s_prefs.end();
}

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
  load_current_settings();
  tmc2209_configure_defaults();
}

void tmc2209_configure_defaults() {
  s_drv.pdn_disable(true);
  s_drv.I_scale_analog(false);
  s_drv.toff(SSC_TMC_TOFF);
  s_drv.blank_time(SSC_TMC_BLANK_TIME);
  apply_current_settings();
  s_drv.microsteps(SSC_TMC_MICROSTEPS);
  s_drv.en_spreadCycle(SSC_TMC_ENABLE_SPREADCYCLE != 0);
  s_drv.TPWMTHRS(SSC_TMC_TPWMTHRS);
}

void tmc2209_set_run_current_ma(uint16_t current_ma) {
  s_run_current_ma = clamp_run_current_ma(current_ma);
  apply_current_settings();
}

void tmc2209_set_hold_current_pct(uint8_t hold_pct) {
  s_hold_current_pct = clamp_hold_pct(hold_pct);
  apply_current_settings();
}

bool tmc2209_save_current_settings() {
  if (!s_prefs.begin(kPrefsNs, false)) return false;
  s_prefs.putUInt(kPrefsVerKey, kPrefsVersion);
  s_prefs.putUShort(kPrefsRunKey, s_run_current_ma);
  s_prefs.putUChar(kPrefsHoldKey, s_hold_current_pct);
  s_prefs.end();
  return true;
}

uint16_t tmc2209_run_current_ma() {
  return s_run_current_ma;
}

uint8_t tmc2209_hold_current_pct() {
  return s_hold_current_pct;
}

void tmc2209_set_enable(bool en) {
  digitalWrite(SSC_PIN_EN, en ? LOW : HIGH);
}

uint16_t tmc2209_status() {
  return s_drv.DRV_STATUS();
}
