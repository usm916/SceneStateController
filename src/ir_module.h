#pragma once
#include "events.h"
#include <Arduino.h>

void ir_setup();
bool ir_poll(Event& out);
void ir_poll_serial_command();
void ir_set_decode_mode(uint8_t mode);
