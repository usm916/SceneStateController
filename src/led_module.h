#pragma once
#include <stdint.h>

enum LedPattern : uint8_t {
  LEDP_IDLE = 0,
  LEDP_MOVING,
  LEDP_ARRIVED,
  LEDP_ERROR,
};

void led_setup();
void led_set_pattern(LedPattern p);
void led_tick(uint32_t now_ms);
