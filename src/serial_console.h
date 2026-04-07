#pragma once

#include <Arduino.h>

class ConsoleLogger;

void serial_console_poll(ConsoleLogger& log, void (*set_runtime_mode_fn)(uint8_t));
