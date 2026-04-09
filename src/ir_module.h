#pragma once
#include "events.h"
#include <Arduino.h>

enum RemoteButton : uint8_t {
  BTN_NONE       = 0xFF,
  BTN_POWER      = 0x45,
  BTN_MODE       = 0x46,
  BTN_MUTE       = 0x47,

  BTN_PLAYPAUSE  = 0x44,
  BTN_PREV       = 0x40,
  BTN_NEXT       = 0x43,

  BTN_EQ         = 0x07,
  BTN_VOL_DOWN   = 0x15,
  BTN_VOL_UP     = 0x09,

  BTN_0          = 0x16,
  BTN_RPT        = 0x19,
  BTN_CLOCK      = 0x0D,

  BTN_1          = 0x0C,
  BTN_2          = 0x18,
  BTN_3          = 0x5E,

  BTN_4          = 0x08,
  BTN_5          = 0x1C,
  BTN_6          = 0x5A,

  BTN_7          = 0x42,
  BTN_8          = 0x52,
  BTN_9          = 0x4A
};

void ir_setup();
bool ir_poll(Event& out);
void ir_set_decode_mode(uint8_t mode);
bool ir_btn(RemoteButton btn);
bool ir_btn_released(RemoteButton btn);
bool ir_any_btn();
RemoteButton ir_active_btn();
RemoteButton ir_get_latest_button();
inline bool btn(RemoteButton btn_code) { return ir_btn(btn_code); }
