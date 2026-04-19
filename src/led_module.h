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
  LEDSCENE_RANDOM_LONG_BLINK_THEN_ON,
  LEDSCENE_CRASH,
  LEDSCENE_EMERGENCY_RED,
  LEDSCENE_BLACKOUT,
  LEDSCENE_FADE_IN_3S,
  LEDSCENE_FADE_OUT_3S,
};

void led_setup();
void led_set_pattern(LedPattern p);
void led_set_strip_scene(uint8_t strip_index, LedStripScene scene);
bool led_set_global_brightness_pct(uint8_t brightness_pct);
uint8_t led_global_brightness_pct();
void led_load_saved_brightness();
bool led_save_global_brightness_pct();
void led_tick(uint32_t now_ms);
