#include "led_module.h"
#include "config.h"
#include <Arduino.h>
#include <FastLED.h>
#include <Preferences.h>

static_assert(SSC_LED_STRIP_COUNT == 6, "This firmware currently assumes exactly 6 LED strips.");
static_assert(SSC_LED_TARGET_FPS > 0, "SSC_LED_TARGET_FPS must be greater than 0.");

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
  LEDSCENE_CHASE,
  LEDSCENE_CHASE,
  LEDSCENE_CHASE,
  LEDSCENE_CHASE,
  LEDSCENE_CHASE,
};

static constexpr LedStripScene kArrivedScenes[SSC_LED_STRIP_COUNT] = {
  LEDSCENE_RANDOM_LONG_BLINK_THEN_ON,
  LEDSCENE_RANDOM_LONG_BLINK_THEN_ON,
  LEDSCENE_RANDOM_LONG_BLINK_THEN_ON,
  LEDSCENE_RANDOM_LONG_BLINK_THEN_ON,
  LEDSCENE_RANDOM_LONG_BLINK_THEN_ON,
  LEDSCENE_RANDOM_LONG_BLINK_THEN_ON,
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
static uint32_t s_blink_last_toggle_ms = 0;
static uint8_t s_global_brightness_pct = 100;
static constexpr const char* kLedPrefsNamespace = "led";
static constexpr const char* kLedBrightnessKey = "brightness";
static uint32_t s_scene_start_ms[SSC_LED_STRIP_COUNT] = {0};
static uint32_t s_random_next_toggle_ms[SSC_LED_STRIP_COUNT][SSC_LED_STRIP_LEN] = {{0}};
static bool s_random_led_on[SSC_LED_STRIP_COUNT][SSC_LED_STRIP_LEN] = {{false}};
static uint32_t s_crash_next_toggle_ms[SSC_LED_STRIP_COUNT] = {0};
static bool s_crash_on[SSC_LED_STRIP_COUNT] = {false};

static CRGB strip_base_color(uint8_t strip_index) {
  if (strip_index >= SSC_LED_STRIP_COUNT) return CRGB(16, 16, 16);
  const SscRgbColor& c = SSC_LED_STRIP_BASE_COLORS[strip_index];
  return CRGB(c.r, c.g, c.b);
}

static CRGB apply_brightness(const CRGB& base, uint8_t brightness) {
  CRGB scaled = base;
  scaled.nscale8_video(brightness);
  return scaled;
}

static uint8_t to_fastled_master_brightness(uint8_t brightness_pct) {
  const uint16_t scaled = ((uint16_t)SSC_LED_BRIGHTNESS * brightness_pct) / 100;
  return (uint8_t)scaled;
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
  randomSeed(micros());
  led_load_saved_brightness();

  for (uint8_t strip = 0; strip < SSC_LED_STRIP_COUNT; strip++) {
    add_strip_controller(strip);
    s_strip_scenes[strip] = LEDSCENE_SOLID;
    s_chase_pos[strip] = 0;
  }

  FastLED.setBrightness(to_fastled_master_brightness(s_global_brightness_pct));
  for (uint8_t strip = 0; strip < SSC_LED_STRIP_COUNT; strip++) {
    for (uint16_t i = 0; i < SSC_LED_STRIP_LEN; i++) {
      s_leds[strip][i] = CRGB::Black;
    }
  }
  FastLED.show();
}

bool led_set_global_brightness_pct(uint8_t brightness_pct) {
  if (brightness_pct > 100) return false;
  s_global_brightness_pct = brightness_pct;
  FastLED.setBrightness(to_fastled_master_brightness(s_global_brightness_pct));
  FastLED.show();
  return true;
}

uint8_t led_global_brightness_pct() {
  return s_global_brightness_pct;
}

void led_load_saved_brightness() {
  Preferences prefs;
  if (!prefs.begin(kLedPrefsNamespace, true)) return;
  const uint8_t saved_brightness = prefs.getUChar(kLedBrightnessKey, s_global_brightness_pct);
  prefs.end();
  (void)led_set_global_brightness_pct(saved_brightness);
}

bool led_save_global_brightness_pct() {
  Preferences prefs;
  if (!prefs.begin(kLedPrefsNamespace, false)) return false;
  const size_t written = prefs.putUChar(kLedBrightnessKey, s_global_brightness_pct);
  prefs.end();
  return written > 0;
}

static void apply_scene_profile(const LedStripScene* profile) {
  const uint32_t now_ms = millis();
  for (uint8_t strip = 0; strip < SSC_LED_STRIP_COUNT; strip++) {
    s_strip_scenes[strip] = profile[strip];
    s_chase_pos[strip] = 0;
    s_scene_start_ms[strip] = now_ms;
    for (uint16_t i = 0; i < SSC_LED_STRIP_LEN; i++) {
      s_random_led_on[strip][i] = false;
      s_random_next_toggle_ms[strip][i] = 0;
    }
    s_crash_next_toggle_ms[strip] = 0;
    s_crash_on[strip] = false;
  }
}

void led_set_pattern(LedPattern p) {
  s_pattern = p;
  s_last_ms = 0;
  s_blink_on = false;
  s_blink_last_toggle_ms = 0;

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
  s_scene_start_ms[strip_index] = millis();
  for (uint16_t i = 0; i < SSC_LED_STRIP_LEN; i++) {
    s_random_led_on[strip_index][i] = false;
    s_random_next_toggle_ms[strip_index][i] = 0;
  }
  s_crash_next_toggle_ms[strip_index] = 0;
  s_crash_on[strip_index] = false;
}

static void paint_strip_solid(uint8_t strip_index, const CRGB& color) {
  for (uint16_t i = 0; i < SSC_LED_STRIP_LEN; i++) {
    s_leds[strip_index][i] = color;
  }
}

static void paint_strip_chase(uint8_t strip_index, const CRGB& base) {
  for (uint16_t i = 0; i < SSC_LED_STRIP_LEN; i++) {
    s_leds[strip_index][i] = CRGB::Black;
  }
  s_leds[strip_index][s_chase_pos[strip_index] % SSC_LED_STRIP_LEN] = apply_brightness(base, 255);
  s_chase_pos[strip_index]++;
}

static void paint_strip_random_long_blink_then_on(uint8_t strip_index, const CRGB& base, uint32_t now_ms) {
  const bool settle_on = (now_ms - s_scene_start_ms[strip_index]) >= 3000;

  for (uint16_t i = 0; i < SSC_LED_STRIP_LEN; i++) {
    if (settle_on) {
      s_leds[strip_index][i] = apply_brightness(base, 255);
      continue;
    }

    if (s_random_next_toggle_ms[strip_index][i] == 0) {
      s_random_led_on[strip_index][i] = (random(0, 100) < 30);
      s_random_next_toggle_ms[strip_index][i] = now_ms + (uint32_t)random(120, 650);
    } else if (now_ms >= s_random_next_toggle_ms[strip_index][i]) {
      s_random_led_on[strip_index][i] = !s_random_led_on[strip_index][i];
      s_random_next_toggle_ms[strip_index][i] = now_ms + (uint32_t)random(180, 800);
    }

    s_leds[strip_index][i] = apply_brightness(base, s_random_led_on[strip_index][i] ? 255 : 0);
  }
}

static void paint_strip_fade_in_3s(uint8_t strip_index, const CRGB& base, uint32_t now_ms) {
  const uint32_t elapsed_ms = now_ms - s_scene_start_ms[strip_index];
  const uint8_t brightness = (elapsed_ms >= 3000) ? 255 : (uint8_t)((elapsed_ms * 255UL) / 3000UL);
  paint_strip_solid(strip_index, apply_brightness(base, brightness));
}

static void paint_strip_fade_out_3s(uint8_t strip_index, const CRGB& base, uint32_t now_ms) {
  const uint32_t elapsed_ms = now_ms - s_scene_start_ms[strip_index];
  const uint8_t brightness = (elapsed_ms >= 3000) ? 0 : (uint8_t)(255 - ((elapsed_ms * 255UL) / 3000UL));
  paint_strip_solid(strip_index, apply_brightness(base, brightness));
}

static void paint_strip_crash_global_random_then_on(uint8_t strip_index, const CRGB& base, uint32_t now_ms) {
  const bool settle_on = (now_ms - s_scene_start_ms[strip_index]) >= 3000;
  if (settle_on) {
    paint_strip_solid(strip_index, apply_brightness(base, 255));
    return;
  }

  if (s_crash_next_toggle_ms[strip_index] == 0) {
    s_crash_on[strip_index] = (random(0, 100) < 50);
    s_crash_next_toggle_ms[strip_index] = now_ms + (uint32_t)random(80, 420);
  } else if (now_ms >= s_crash_next_toggle_ms[strip_index]) {
    s_crash_on[strip_index] = !s_crash_on[strip_index];
    s_crash_next_toggle_ms[strip_index] = now_ms + (uint32_t)random(120, 520);
  }

  paint_strip_solid(strip_index, s_crash_on[strip_index] ? apply_brightness(base, 255) : CRGB::Black);
}

static void render_strip(uint8_t strip_index, uint32_t now_ms) {
  const CRGB base = strip_base_color(strip_index);
  const CRGB error_color = CRGB(64, 0, 0);

  switch (s_strip_scenes[strip_index]) {
    case LEDSCENE_CHASE:
      paint_strip_chase(strip_index, base);
      break;
    case LEDSCENE_BLINK:
      if (s_pattern == LEDP_ERROR) {
        paint_strip_solid(strip_index, s_blink_on ? error_color : CRGB::Black);
      } else {
        paint_strip_solid(strip_index, s_blink_on ? apply_brightness(base, 255) : CRGB::Black);
      }
      break;
    case LEDSCENE_RANDOM_LONG_BLINK_THEN_ON:
      paint_strip_random_long_blink_then_on(strip_index, base, now_ms);
      break;
    case LEDSCENE_CRASH:
      paint_strip_crash_global_random_then_on(strip_index, base, now_ms);
      break;
    case LEDSCENE_EMERGENCY_RED:
      paint_strip_solid(strip_index, CRGB::Red);
      break;
    case LEDSCENE_BLACKOUT:
      paint_strip_solid(strip_index, CRGB::Black);
      break;
    case LEDSCENE_FADE_IN_3S:
      paint_strip_fade_in_3s(strip_index, base, now_ms);
      break;
    case LEDSCENE_FADE_OUT_3S:
      paint_strip_fade_out_3s(strip_index, base, now_ms);
      break;
    case LEDSCENE_SOLID:
    default:
      paint_strip_solid(strip_index, apply_brightness(base, 255));
      break;
  }
}

void led_tick(uint32_t now_ms) {
  const uint32_t interval_ms = 1000UL / (uint32_t)SSC_LED_TARGET_FPS;
  if (interval_ms == 0) return;

  if (s_last_ms == 0) {
    s_last_ms = now_ms;
  }

  if ((now_ms - s_last_ms) < interval_ms) {
    return;
  }
  s_last_ms += interval_ms;
  if ((now_ms - s_last_ms) >= interval_ms) s_last_ms = now_ms;

  if (s_pattern == LEDP_ERROR || s_pattern == LEDP_ARRIVED) {
    const uint32_t blink_interval_ms = (s_pattern == LEDP_ERROR) ? 200UL : 50UL;
    if (s_blink_last_toggle_ms == 0 || (now_ms - s_blink_last_toggle_ms) >= blink_interval_ms) {
      s_blink_on = !s_blink_on;
      s_blink_last_toggle_ms = now_ms;
    }
  } else {
    s_blink_on = true;
  }

  for (uint8_t strip = 0; strip < SSC_LED_STRIP_COUNT; strip++) {
    render_strip(strip, now_ms);
  }

  FastLED.show();
}
