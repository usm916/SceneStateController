#include "led_module.h"
#include "config.h"
#include <Arduino.h>
#include <FastLED.h>
#include <Preferences.h>

static_assert(SSC_LED_STRIP_COUNT == 6, "This firmware currently assumes exactly 6 LED strips.");
static_assert(SSC_LED_STRIP_COUNT <= 255, "LedPoint.y must fit in uint8_t.");
static_assert(SSC_LED_STRIP_LEN <= 255, "LedPoint.x must fit in uint8_t.");
static_assert(SSC_LED_TARGET_FPS > 0, "SSC_LED_TARGET_FPS must be greater than 0.");
static_assert(SSC_LED_ACTIVE_STRIP_COUNT <= SSC_LED_STRIP_COUNT,
              "SSC_LED_ACTIVE_STRIP_COUNT must be <= SSC_LED_STRIP_COUNT.");

static CRGB s_leds[SSC_LED_STRIP_COUNT][SSC_LED_STRIP_LEN];
static LedPoint s_led_points[SSC_LED_STRIP_COUNT][SSC_LED_STRIP_LEN];
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
static uint8_t s_blink_level = 255;
static uint8_t s_global_brightness_pct = 100;
static constexpr const char* kLedPrefsNamespace = "led";
static constexpr const char* kLedBrightnessKey = "brightness";
static constexpr const char* kLedNoiseBaseMinKey = "noise_base";
static constexpr const char* kLedNoiseAmplitudeMaxKey = "noise_amp";
static constexpr const char* kLedNoiseSpeedKey = "noise_speed";
static constexpr const char* kLedBaseColorCandidateIndexKey = "base_color_i";
static constexpr const char* kLedStripColorPresetBlobKey = "strip_color_p";
static uint32_t s_scene_start_ms[SSC_LED_STRIP_COUNT] = {0};
static uint32_t s_random_next_toggle_ms[SSC_LED_STRIP_COUNT][SSC_LED_STRIP_LEN] = {{0}};
static bool s_random_led_on[SSC_LED_STRIP_COUNT][SSC_LED_STRIP_LEN] = {{false}};
static uint32_t s_crash_next_toggle_ms[SSC_LED_STRIP_COUNT] = {0};
static bool s_crash_on[SSC_LED_STRIP_COUNT] = {false};
static uint8_t s_noise_base_min = 40;
static uint8_t s_noise_amplitude_max = 80;
static uint8_t s_noise_speed = 24;
static uint8_t s_base_color_candidate_index = 0;
static uint8_t s_strip_color_preset_index[SSC_LED_STRIP_COUNT] = {0};
static constexpr uint8_t kColorGuardThreshold = 10;
static constexpr uint8_t kColorGuardFloor = 6;

static CRGB strip_base_color(uint8_t strip_index) {
  if (strip_index >= SSC_LED_STRIP_COUNT) return CRGB(16, 16, 16);
  if (SSC_LED_STRIP_COLOR_PRESET_COUNT > 0) {
    const uint8_t preset = s_strip_color_preset_index[strip_index] % SSC_LED_STRIP_COLOR_PRESET_COUNT;
    const SscRgbColor& c = SSC_LED_STRIP_COLOR_PRESETS[preset];
    return CRGB(c.r, c.g, c.b);
  }
  if (SSC_LED_BASE_COLOR_CANDIDATE_COUNT > 0) {
    const SscRgbColor& c = SSC_LED_BASE_COLOR_CANDIDATES[s_base_color_candidate_index];
    return CRGB(c.r, c.g, c.b);
  }
  const SscRgbColor& c = SSC_LED_STRIP_BASE_COLORS[strip_index];
  return CRGB(c.r, c.g, c.b);
}

static CRGB apply_brightness(const CRGB& base, uint8_t brightness) {
  CRGB scaled = base;
  scaled.nscale8_video(brightness);
  const bool base_has_all_channels = (base.r > 0) && (base.g > 0) && (base.b > 0);
  if (!base_has_all_channels) return scaled;

  const bool has_zero_channel = (scaled.r == 0) || (scaled.g == 0) || (scaled.b == 0);
  if (!has_zero_channel) return scaled;

  const uint8_t max_channel = max(scaled.r, max(scaled.g, scaled.b));
  if (max_channel < kColorGuardThreshold) {
    if (scaled.r < kColorGuardFloor) scaled.r = kColorGuardFloor;
    if (scaled.g < kColorGuardFloor) scaled.g = kColorGuardFloor;
    if (scaled.b < kColorGuardFloor) scaled.b = kColorGuardFloor;
    return scaled;
  }

  if (scaled.r == 0) scaled.r = kColorGuardFloor;
  if (scaled.g == 0) scaled.g = kColorGuardFloor;
  if (scaled.b == 0) scaled.b = kColorGuardFloor;
  return scaled;
}

static CRGB apply_brightness_linear(const CRGB& base, uint8_t brightness) {
  CRGB scaled = base;
  scaled.nscale8(brightness);
  return scaled;
}

static uint8_t to_fastled_master_brightness(uint8_t brightness_pct) {
  const uint16_t scaled = ((uint16_t)SSC_LED_BRIGHTNESS * brightness_pct) / 100;
  return (uint8_t)scaled;
}

static uint8_t led_value_from_color(const CRGB& c) {
  return max(c.r, max(c.g, c.b));
}

static void sync_point_values_for_strip(uint8_t strip_index) {
  if (strip_index >= SSC_LED_STRIP_COUNT) return;
  for (uint16_t i = 0; i < SSC_LED_STRIP_LEN; i++) {
    s_led_points[strip_index][i].val = led_value_from_color(s_leds[strip_index][i]);
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
  randomSeed(micros());
  led_load_saved_brightness();
  led_load_saved_noise_params();
  led_load_saved_base_color_candidate_index();
  for (uint8_t strip = 0; strip < SSC_LED_STRIP_COUNT; ++strip) {
    s_strip_color_preset_index[strip] = 0;
  }
  led_load_saved_strip_color_presets();

  for (uint8_t strip = 0; strip < SSC_LED_ACTIVE_STRIP_COUNT; strip++) {
    add_strip_controller(strip);
    s_strip_scenes[strip] = LEDSCENE_BLACKOUT;
    s_chase_pos[strip] = 0;
  }

  FastLED.setBrightness(to_fastled_master_brightness(s_global_brightness_pct));
  FastLED.setDither(1);
  for (uint8_t strip = 0; strip < SSC_LED_STRIP_COUNT; strip++) {
    for (uint16_t i = 0; i < SSC_LED_STRIP_LEN; i++) {
      s_leds[strip][i] = CRGB::Black;
      s_led_points[strip][i].x = (uint8_t)i;
      s_led_points[strip][i].y = strip;
      s_led_points[strip][i].val = 0;
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

uint8_t led_cycle_base_color_candidate() {
  if (SSC_LED_BASE_COLOR_CANDIDATE_COUNT == 0) return 0;
  s_base_color_candidate_index = (uint8_t)((s_base_color_candidate_index + 1) % SSC_LED_BASE_COLOR_CANDIDATE_COUNT);
  return s_base_color_candidate_index;
}

uint8_t led_base_color_candidate_index() {
  return s_base_color_candidate_index;
}

bool led_set_base_color_candidate_index(uint8_t index) {
  if (SSC_LED_BASE_COLOR_CANDIDATE_COUNT == 0) return false;
  if (index >= SSC_LED_BASE_COLOR_CANDIDATE_COUNT) return false;
  s_base_color_candidate_index = index;
  return true;
}

void led_load_saved_base_color_candidate_index() {
  Preferences prefs;
  if (!prefs.begin(kLedPrefsNamespace, true)) return;
  const uint8_t saved = prefs.getUChar(kLedBaseColorCandidateIndexKey, s_base_color_candidate_index);
  prefs.end();
  (void)led_set_base_color_candidate_index(saved);
}

bool led_save_base_color_candidate_index() {
  Preferences prefs;
  if (!prefs.begin(kLedPrefsNamespace, false)) return false;
  const size_t written = prefs.putUChar(kLedBaseColorCandidateIndexKey, s_base_color_candidate_index);
  prefs.end();
  return written > 0;
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

void led_load_saved_noise_params() {
  Preferences prefs;
  if (!prefs.begin(kLedPrefsNamespace, true)) return;
  const uint8_t savedBase = prefs.getUChar(kLedNoiseBaseMinKey, s_noise_base_min);
  const uint8_t savedAmp = prefs.getUChar(kLedNoiseAmplitudeMaxKey, s_noise_amplitude_max);
  const uint8_t savedSpeed = prefs.getUChar(kLedNoiseSpeedKey, s_noise_speed);
  prefs.end();
  (void)led_set_noise_params(savedBase, savedAmp, savedSpeed);
}

bool led_save_noise_params() {
  Preferences prefs;
  if (!prefs.begin(kLedPrefsNamespace, false)) return false;
  const size_t baseWritten = prefs.putUChar(kLedNoiseBaseMinKey, s_noise_base_min);
  const size_t ampWritten = prefs.putUChar(kLedNoiseAmplitudeMaxKey, s_noise_amplitude_max);
  const size_t speedWritten = prefs.putUChar(kLedNoiseSpeedKey, s_noise_speed);
  prefs.end();
  return baseWritten > 0 && ampWritten > 0 && speedWritten > 0;
}

bool led_set_strip_color_preset(uint8_t strip_index, uint8_t preset_index) {
  if (strip_index >= SSC_LED_STRIP_COUNT) return false;
  if (preset_index >= SSC_LED_STRIP_COLOR_PRESET_COUNT) return false;
  s_strip_color_preset_index[strip_index] = preset_index;
  return true;
}

bool led_set_all_strip_color_preset(uint8_t preset_index) {
  if (preset_index >= SSC_LED_STRIP_COLOR_PRESET_COUNT) return false;
  for (uint8_t strip = 0; strip < SSC_LED_STRIP_COUNT; ++strip) {
    s_strip_color_preset_index[strip] = preset_index;
  }
  return true;
}

uint8_t led_strip_color_preset(uint8_t strip_index) {
  if (strip_index >= SSC_LED_STRIP_COUNT) return 0;
  return s_strip_color_preset_index[strip_index];
}

void led_load_saved_strip_color_presets() {
  Preferences prefs;
  if (!prefs.begin(kLedPrefsNamespace, true)) return;
  uint8_t saved[SSC_LED_STRIP_COUNT] = {0};
  const size_t got = prefs.getBytes(kLedStripColorPresetBlobKey, saved, sizeof(saved));
  prefs.end();
  if (got != sizeof(saved)) return;
  for (uint8_t strip = 0; strip < SSC_LED_STRIP_COUNT; ++strip) {
    const uint8_t preset = saved[strip];
    if (preset < SSC_LED_STRIP_COLOR_PRESET_COUNT) {
      s_strip_color_preset_index[strip] = preset;
    }
  }
}

bool led_save_strip_color_presets() {
  Preferences prefs;
  if (!prefs.begin(kLedPrefsNamespace, false)) return false;
  const size_t written = prefs.putBytes(kLedStripColorPresetBlobKey,
                                        s_strip_color_preset_index,
                                        sizeof(s_strip_color_preset_index));
  prefs.end();
  return written == sizeof(s_strip_color_preset_index);
}

void led_set_updates_enabled(bool enabled) {
  if (enabled) {
    s_last_ms = 0;
  }
}

bool led_updates_enabled() {
  return true;
}

static void apply_scene_profile(const LedStripScene* profile) {
  const uint32_t now_ms = millis();
  for (uint8_t strip = 0; strip < SSC_LED_ACTIVE_STRIP_COUNT; strip++) {
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

void led_set_pattern(LedPattern p, bool force_reset) {
  if (!force_reset && s_pattern == p) return;
  s_pattern = p;
  s_last_ms = 0;
  s_blink_level = 255;

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
  if (strip_index >= SSC_LED_ACTIVE_STRIP_COUNT) return;
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

bool led_set_noise_params(uint8_t base_min, uint8_t amplitude_max, uint8_t speed) {
  if ((uint16_t)base_min + (uint16_t)amplitude_max > 255) return false;
  if (speed == 0) return false;
  s_noise_base_min = base_min;
  s_noise_amplitude_max = amplitude_max;
  s_noise_speed = speed;
  return true;
}

void led_get_noise_params(uint8_t* out_base_min, uint8_t* out_amplitude_max, uint8_t* out_speed) {
  if (out_base_min != nullptr) *out_base_min = s_noise_base_min;
  if (out_amplitude_max != nullptr) *out_amplitude_max = s_noise_amplitude_max;
  if (out_speed != nullptr) *out_speed = s_noise_speed;
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

static uint8_t smoothstep_progress_8(uint32_t elapsed_ms, uint32_t duration_ms) {
  if (elapsed_ms >= duration_ms) return 255;
  const uint32_t t_q10 = (elapsed_ms * 1024UL) / duration_ms;  // 0..1023
  const uint32_t t2_q10 = (t_q10 * t_q10) >> 10;
  const uint32_t t3_q10 = (t2_q10 * t_q10) >> 10;
  const uint32_t eased_q10 = (3UL * t2_q10) - (2UL * t3_q10);  // smoothstep
  return (uint8_t)((eased_q10 * 255UL + 511UL) / 1023UL);
}

static constexpr uint32_t kLedFadeDurationMs = 5000UL;

static void paint_strip_fade_in_3s(uint8_t strip_index, const CRGB& base, uint32_t now_ms) {
  const uint32_t elapsed_ms = now_ms - s_scene_start_ms[strip_index];
  const uint8_t brightness = smoothstep_progress_8(elapsed_ms, kLedFadeDurationMs);
  paint_strip_solid(strip_index, apply_brightness_linear(base, brightness));
}

static void paint_strip_fade_out_3s(uint8_t strip_index, const CRGB& base, uint32_t now_ms) {
  const uint32_t elapsed_ms = now_ms - s_scene_start_ms[strip_index];
  const uint8_t brightness = (uint8_t)(255 - smoothstep_progress_8(elapsed_ms, kLedFadeDurationMs));
  paint_strip_solid(strip_index, apply_brightness_linear(base, brightness));
}

static void paint_strip_crash_global_random_then_on(uint8_t strip_index, const CRGB& base, uint32_t now_ms) {
  const bool settle_on = (now_ms - s_scene_start_ms[strip_index]) >= 1500;
  if (settle_on) {
    paint_strip_solid(strip_index, apply_brightness(base, 255));
    return;
  }

  if (s_crash_next_toggle_ms[strip_index] == 0) {
    s_crash_on[strip_index] = (random(0, 100) < 50);
    s_crash_next_toggle_ms[strip_index] = now_ms + (uint32_t)random(20, 80);
  } else if (now_ms >= s_crash_next_toggle_ms[strip_index]) {
    s_crash_on[strip_index] = !s_crash_on[strip_index];
    s_crash_next_toggle_ms[strip_index] = now_ms + (uint32_t)random(40, 240);
  }

  paint_strip_solid(strip_index, s_crash_on[strip_index] ? apply_brightness(base, 255) : CRGB::Black);
}

static void paint_strip_noise_flame(uint8_t strip_index, const CRGB& base, uint32_t now_ms) {
  const uint16_t spatial_step = 37;
  const uint16_t temporal = (uint16_t)((now_ms * (uint32_t)s_noise_speed) / 8U);
  for (uint16_t i = 0; i < SSC_LED_STRIP_LEN; i++) {
    const uint8_t noise = inoise8((uint16_t)(i * spatial_step), temporal);
    const uint8_t scaled_noise = scale8(noise, s_noise_amplitude_max);
    const uint8_t shimmer = (uint8_t)random8(0, 12);
    const uint8_t brightness = (uint8_t)min(255, (int)s_noise_base_min + (int)scaled_noise + (int)shimmer);
    s_leds[strip_index][i] = apply_brightness(base, brightness);
  }
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
        paint_strip_solid(strip_index, apply_brightness_linear(error_color, s_blink_level));
      } else {
        paint_strip_solid(strip_index, apply_brightness(base, s_blink_level));
      }
      break;
    case LEDSCENE_RANDOM_LONG_BLINK_THEN_ON:
      paint_strip_random_long_blink_then_on(strip_index, base, now_ms);
      break;
    case LEDSCENE_NOISE_FLAME:
      paint_strip_noise_flame(strip_index, base, now_ms);
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
  sync_point_values_for_strip(strip_index);
}

static bool has_blink_scene_active() {
  for (uint8_t strip = 0; strip < SSC_LED_ACTIVE_STRIP_COUNT; strip++) {
    if (s_strip_scenes[strip] == LEDSCENE_BLINK) return true;
  }
  return false;
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

  if (s_pattern == LEDP_ERROR || s_pattern == LEDP_ARRIVED || has_blink_scene_active()) {
    const uint32_t blink_period_ms = (s_pattern == LEDP_ERROR) ? 500UL : 1200UL;
    const uint32_t phase_ms = now_ms % blink_period_ms;
    const uint8_t phase_8 = (uint8_t)((phase_ms * 255UL) / blink_period_ms);
    s_blink_level = sin8(phase_8);
  } else {
    s_blink_level = 255;
  }

  for (uint8_t strip = 0; strip < SSC_LED_ACTIVE_STRIP_COUNT; strip++) {
    render_strip(strip, now_ms);
  }

  FastLED.show();
}

const char* led_rmt_status_text() {
#if defined(FASTLED_RMT5)
  return (FASTLED_RMT5 != 0) ? "enabled (RMT5)" : "disabled (FASTLED_RMT5=0)";
#elif defined(FASTLED_RMT_BUILTIN_DRIVER)
  return (FASTLED_RMT_BUILTIN_DRIVER != 0) ? "enabled (RMT4)" : "disabled (FASTLED_RMT_BUILTIN_DRIVER=0)";
#elif defined(FASTLED_ESP32_I2S)
  return (FASTLED_ESP32_I2S != 0) ? "disabled (I2S backend)" : "disabled (FASTLED_ESP32_I2S=0)";
#elif defined(ARDUINO_ARCH_ESP32)
  return "auto (ESP32 backend selected by FastLED)";
#else
  return "n/a (non-ESP32 build)";
#endif
}

const LedPoint* led_points(uint8_t strip_index, uint16_t* out_count) {
  if (out_count != nullptr) *out_count = 0;
  if (strip_index >= SSC_LED_STRIP_COUNT) return nullptr;
  if (out_count != nullptr) *out_count = SSC_LED_STRIP_LEN;
  return s_led_points[strip_index];
}

bool led_point_brightness(uint8_t strip_index, uint16_t point_index, uint8_t* out_brightness) {
  if (out_brightness == nullptr) return false;
  if (strip_index >= SSC_LED_STRIP_COUNT) return false;
  if (point_index >= SSC_LED_STRIP_LEN) return false;
  *out_brightness = s_led_points[strip_index][point_index].val;
  return true;
}
