#include "led_module.h"
#include "config.h"
#include <FastLED.h>

static CRGB s_leds[SSC_LED_COUNT];
static LedPattern s_pattern = LEDP_IDLE;
static uint32_t s_last_ms = 0;
static uint16_t s_chase_pos = 0;
static bool s_blink_on = false;

void led_setup() {
  FastLED.addLeds<WS2812B, SSC_PIN_WS2812B, GRB>(s_leds, SSC_LED_COUNT);
  FastLED.setBrightness(SSC_LED_BRIGHTNESS);
  for (int i = 0; i < SSC_LED_COUNT; i++) s_leds[i] = CRGB::Black;
  FastLED.show();
}

void led_set_pattern(LedPattern p) {
  s_pattern = p;
  s_last_ms = 0;
  s_chase_pos = 0;
  s_blink_on = false;
}

static void show_solid(const CRGB& c) {
  for (int i = 0; i < SSC_LED_COUNT; i++) s_leds[i] = c;
  FastLED.show();
}

static void show_chase(const CRGB& c) {
  for (int i = 0; i < SSC_LED_COUNT; i++) s_leds[i] = CRGB::Black;
  s_leds[s_chase_pos % SSC_LED_COUNT] = c;
  FastLED.show();
  s_chase_pos++;
}

static void show_blink(const CRGB& c) {
  s_blink_on = !s_blink_on;
  show_solid(s_blink_on ? c : CRGB::Black);
}

void led_tick(uint32_t now_ms) {
  const uint32_t interval_ms =
    (s_pattern == LEDP_MOVING) ? 40 :
    (s_pattern == LEDP_ERROR)  ? 200 : 0;

  if (interval_ms == 0) {
    if (s_last_ms == 0) {
      if (s_pattern == LEDP_IDLE)    show_solid(CRGB(0, 0, 16));
      if (s_pattern == LEDP_ARRIVED) show_solid(CRGB(0, 0, 64));
      s_last_ms = now_ms;
    }
    return;
  }

  if (now_ms - s_last_ms < interval_ms) return;
  s_last_ms = now_ms;

  if (s_pattern == LEDP_MOVING) show_chase(CRGB(0, 16, 0));
  if (s_pattern == LEDP_ERROR)  show_blink(CRGB(64, 0, 0));
}
