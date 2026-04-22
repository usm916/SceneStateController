#pragma once
#include <stdint.h>

struct LedPoint {
  uint8_t x;
  uint8_t y;
  uint8_t val;
};

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
void led_set_pattern(LedPattern p, bool force_reset = false);
void led_set_strip_scene(uint8_t strip_index, LedStripScene scene);
bool led_set_global_brightness_pct(uint8_t brightness_pct);
uint8_t led_global_brightness_pct();
void led_load_saved_brightness();
bool led_save_global_brightness_pct();
void led_set_updates_enabled(bool enabled);
bool led_updates_enabled();
void led_tick(uint32_t now_ms);
const char* led_rmt_status_text();
const LedPoint* led_points(uint8_t strip_index, uint16_t* out_count);
bool led_point_brightness(uint8_t strip_index, uint16_t point_index, uint8_t* out_brightness);
