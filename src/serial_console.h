#pragma once

#include <Arduino.h>
#include "events.h"

class ConsoleLogger;

typedef bool (*serial_wifi_slot_setter_t)(int slot, const char* ssid, const char* pass);

bool serial_console_poll(ConsoleLogger& log,
                         void (*set_runtime_mode_fn)(uint8_t),
                         serial_wifi_slot_setter_t wifi_slot_setter_fn,
                         Event* out_event);
