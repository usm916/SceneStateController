#pragma once
#include <stdint.h>

void tmc2209_setup();
void tmc2209_set_enable(bool en);
void tmc2209_configure_defaults();
uint16_t tmc2209_status();
