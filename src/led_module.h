#pragma once
#include <stdint.h>

enum LedPattern : uint8_t {
  LEDP_IDLE = 0,
  LEDP_MOVING,
  LEDP_ARRIVED,
  LEDP_ERROR,
};

enum LedStripScene : uint8_t {
  LEDSCENE_SOLID = 0,
  LEDSCENE_CHASE,
  LEDSCENE_BLINK,
};

void led_setup();
void led_set_pattern(LedPattern p);
void led_set_strip_scene(uint8_t strip_index, LedStripScene scene);
void led_tick(uint32_t now_ms);
