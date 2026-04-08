#pragma once
#include <stdint.h>

void tmc2209_setup();
void tmc2209_set_enable(bool en);
void tmc2209_configure_defaults();
void tmc2209_set_motor_current_ma(uint16_t current_ma);
void tmc2209_set_run_current_ma(uint16_t current_ma);
void tmc2209_set_hold_current_pct(uint8_t hold_pct);
uint16_t tmc2209_motor_current_ma();
uint16_t tmc2209_run_current_ma();
uint8_t tmc2209_hold_current_pct();
uint16_t tmc2209_status();
