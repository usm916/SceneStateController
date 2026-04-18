#include "led_module.h"
#include "config.h"
#include <FastLED.h>

static_assert(SSC_LED_STRIP_COUNT == 6, "This firmware currently assumes exactly 6 LED strips.");

static CRGB s_leds[SSC_LED_STRIP_COUNT][SSC_LED_STRIP_LEN];
static LedPattern s_pattern = LEDP_IDLE;
static LedStripScene s_strip_scenes[SSC_LED_STRIP_COUNT];

static constexpr LedStripScene kIdleScenes[SSC_LED_STRIP_COUNT] = {
  LEDSCENE_SOLID,
  LEDSCENE_SOLID,
  LEDSCENE_SOLID,
  LEDSCENE_SOLID,
  LEDSCENE_SOLID,
  LEDSCENE_SOLID,
};

static constexpr LedStripScene kMovingScenes[SSC_LED_STRIP_COUNT] = {
  LEDSCENE_CHASE,
  LEDSCENE_SOLID,
  LEDSCENE_CHASE,
  LEDSCENE_SOLID,
  LEDSCENE_CHASE,
  LEDSCENE_SOLID,
};

static constexpr LedStripScene kArrivedScenes[SSC_LED_STRIP_COUNT] = {
  LEDSCENE_SOLID,
  LEDSCENE_BLINK,
  LEDSCENE_SOLID,
  LEDSCENE_BLINK,
  LEDSCENE_SOLID,
  LEDSCENE_BLINK,
};

static constexpr LedStripScene kErrorScenes[SSC_LED_STRIP_COUNT] = {
  LEDSCENE_BLINK,
  LEDSCENE_BLINK,
  LEDSCENE_BLINK,
  LEDSCENE_BLINK,
  LEDSCENE_BLINK,
  LEDSCENE_BLINK,
};

static uint32_t s_last_ms = 0;
static uint16_t s_chase_pos[SSC_LED_STRIP_COUNT] = {0};
static bool s_blink_on = false;

static CRGB strip_base_color(uint8_t strip_index) {
  switch (strip_index) {
    case 0: return CRGB(40, 0, 0);    // Red
    case 1: return CRGB(0, 40, 0);    // Green
    case 2: return CRGB(0, 0, 40);    // Blue
    case 3: return CRGB(40, 28, 0);   // Amber
    case 4: return CRGB(24, 0, 40);   // Purple
    case 5: return CRGB(0, 32, 32);   // Cyan
    default: return CRGB(16, 16, 16);
  }
}

static void add_strip_controller(uint8_t strip_index) {
  switch (strip_index) {
    case 0:
      FastLED.addLeds<WS2812B, SSC_PIN_WS2812B_0, GRB>(s_leds[0], SSC_LED_STRIP_LEN);
      break;
    case 1:
      FastLED.addLeds<WS2812B, SSC_PIN_WS2812B_1, GRB>(s_leds[1], SSC_LED_STRIP_LEN);
      break;
    case 2:
      FastLED.addLeds<WS2812B, SSC_PIN_WS2812B_2, GRB>(s_leds[2], SSC_LED_STRIP_LEN);
      break;
    case 3:
      FastLED.addLeds<WS2812B, SSC_PIN_WS2812B_3, GRB>(s_leds[3], SSC_LED_STRIP_LEN);
      break;
    case 4:
      FastLED.addLeds<WS2812B, SSC_PIN_WS2812B_4, GRB>(s_leds[4], SSC_LED_STRIP_LEN);
      break;
    case 5:
      FastLED.addLeds<WS2812B, SSC_PIN_WS2812B_5, GRB>(s_leds[5], SSC_LED_STRIP_LEN);
      break;
    default:
      break;
  }
}

void led_setup() {
  for (uint8_t strip = 0; strip < SSC_LED_STRIP_COUNT; strip++) {
    add_strip_controller(strip);
    s_strip_scenes[strip] = LEDSCENE_SOLID;
    s_chase_pos[strip] = 0;
  }

  FastLED.setBrightness(SSC_LED_BRIGHTNESS);
  for (uint8_t strip = 0; strip < SSC_LED_STRIP_COUNT; strip++) {
    for (uint16_t i = 0; i < SSC_LED_STRIP_LEN; i++) {
      s_leds[strip][i] = CRGB::Black;
    }
  }
  FastLED.show();
}

static void apply_scene_profile(const LedStripScene* profile) {
  for (uint8_t strip = 0; strip < SSC_LED_STRIP_COUNT; strip++) {
    s_strip_scenes[strip] = profile[strip];
    s_chase_pos[strip] = 0;
  }
}

void led_set_pattern(LedPattern p) {
  s_pattern = p;
  s_last_ms = 0;
  s_blink_on = false;

  switch (p) {
    case LEDP_MOVING:
      apply_scene_profile(kMovingScenes);
      break;
    case LEDP_ARRIVED:
      apply_scene_profile(kArrivedScenes);
      break;
    case LEDP_ERROR:
      apply_scene_profile(kErrorScenes);
      break;
    case LEDP_IDLE:
    default:
      apply_scene_profile(kIdleScenes);
      break;
  }
}

void led_set_strip_scene(uint8_t strip_index, LedStripScene scene) {
  if (strip_index >= SSC_LED_STRIP_COUNT) return;
  s_strip_scenes[strip_index] = scene;
  s_chase_pos[strip_index] = 0;
}

static void paint_strip_solid(uint8_t strip_index, const CRGB& color) {
  for (uint16_t i = 0; i < SSC_LED_STRIP_LEN; i++) {
    s_leds[strip_index][i] = color;
  }
}

static void paint_strip_chase(uint8_t strip_index, const CRGB& color) {
  for (uint16_t i = 0; i < SSC_LED_STRIP_LEN; i++) {
    s_leds[strip_index][i] = CRGB::Black;
  }
  s_leds[strip_index][s_chase_pos[strip_index] % SSC_LED_STRIP_LEN] = color;
  s_chase_pos[strip_index]++;
}

static void render_strip(uint8_t strip_index) {
  const CRGB base = strip_base_color(strip_index);

  switch (s_strip_scenes[strip_index]) {
    case LEDSCENE_CHASE:
      paint_strip_chase(strip_index, base);
      break;
    case LEDSCENE_BLINK:
      paint_strip_solid(strip_index, s_blink_on ? base : CRGB::Black);
      break;
    case LEDSCENE_SOLID:
    default:
      paint_strip_solid(strip_index, base);
      break;
  }
}

void led_tick(uint32_t now_ms) {
  const uint32_t interval_ms =
      (s_pattern == LEDP_MOVING) ? 40 :
      (s_pattern == LEDP_ERROR) ? 200 :
      (s_pattern == LEDP_ARRIVED) ? 250 : 0;

  if (interval_ms != 0) {
    if (now_ms - s_last_ms < interval_ms) return;
    s_last_ms = now_ms;
  } else if (s_last_ms != 0) {
    return;
  } else {
    s_last_ms = now_ms;
  }

  if (s_pattern == LEDP_ERROR || s_pattern == LEDP_ARRIVED) {
    s_blink_on = !s_blink_on;
  } else {
    s_blink_on = true;
  }

  for (uint8_t strip = 0; strip < SSC_LED_STRIP_COUNT; strip++) {
    render_strip(strip);
  }

  FastLED.show();
}
