#pragma once

#include <Arduino.h>
#include "events.h"

class ConsoleLogger;

bool serial_console_poll(ConsoleLogger& log,
                         void (*set_runtime_mode_fn)(uint8_t),
                         Event* out_event);
