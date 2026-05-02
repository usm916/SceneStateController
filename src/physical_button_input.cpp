#include "physical_button_input.h"

#include "config.h"
#include "ir_module.h"
#include "runtime_mode.h"

#include <Arduino.h>

namespace {

constexpr uint8_t kButtonPins[6] = {
    SSC_PIN_WS2812B_0,
    SSC_PIN_WS2812B_1,
    SSC_PIN_WS2812B_2,
    SSC_PIN_WS2812B_3,
    SSC_PIN_WS2812B_4,
    SSC_PIN_WS2812B_5,
};

constexpr RemoteButton kMappedButtons[6] = {
    (RemoteButton)SSC_SWITCH_BUTTON_CODE_0,
    (RemoteButton)SSC_SWITCH_BUTTON_CODE_1,
    (RemoteButton)SSC_SWITCH_BUTTON_CODE_2,
    (RemoteButton)SSC_SWITCH_BUTTON_CODE_3,
    (RemoteButton)SSC_SWITCH_BUTTON_CODE_4,
    (RemoteButton)SSC_SWITCH_BUTTON_CODE_5,
};

struct DebounceState {
  bool raw_pressed;
  bool stable_pressed;
  uint32_t last_change_ms;
};

DebounceState s_states[6] = {};
uint32_t s_last_poll_ms = 0;
uint32_t s_last_raw_log_ms = 0;
uint32_t s_last_debounced_log_ms = 0;

bool read_pressed(uint8_t pin) {
  const int value = digitalRead(pin);
#if SSC_SWITCH_ACTIVE_LOW
  return value == LOW;
#else
  return value == HIGH;
#endif
}

}  // namespace

bool physical_button_input_enabled() {
#if SSC_LED_PIN_MODE == SSC_LED_PIN_MODE_SWITCH
  return true;
#else
  return (runtime_mode_get() & RUNTIME_MODE_LED) == 0;
#endif
}

void physical_button_input_setup() {
  if (!physical_button_input_enabled()) return;

  for (uint8_t i = 0; i < 6; i++) {
#if SSC_SWITCH_USE_INPUT_PULLUP
    pinMode(kButtonPins[i], INPUT_PULLUP);
#else
    pinMode(kButtonPins[i], INPUT);
#endif
    const bool pressed = read_pressed(kButtonPins[i]);
    s_states[i].raw_pressed = pressed;
    s_states[i].stable_pressed = pressed;
    s_states[i].last_change_ms = millis();
  }

  Serial.println("Physical button mode: WS2812B pins configured as digital inputs.");
  Serial.print("Physical button config: active_level=");
#if SSC_SWITCH_ACTIVE_LOW
  Serial.print("LOW");
#else
  Serial.print("HIGH");
#endif
  Serial.print(", input_mode=");
#if SSC_SWITCH_USE_INPUT_PULLUP
  Serial.println("INPUT_PULLUP");
#else
  Serial.println("INPUT");
#endif

  for (uint8_t i = 0; i < 6; i++) {
    Serial.print("Physical button map: pin=");
    Serial.print(kButtonPins[i]);
    Serial.print(" -> code=0x");
    Serial.println((uint8_t)kMappedButtons[i], HEX);
  }
}

void physical_button_input_poll() {
  if (!physical_button_input_enabled()) return;

  const uint32_t now_ms = millis();
  if ((uint32_t)(now_ms - s_last_poll_ms) < SSC_SWITCH_POLL_INTERVAL_MS) return;
  s_last_poll_ms = now_ms;

  bool raw_pressed_values[6] = {};

  for (uint8_t i = 0; i < 6; i++) {
    const bool raw_pressed = read_pressed(kButtonPins[i]);
    raw_pressed_values[i] = raw_pressed;
    if (raw_pressed != s_states[i].raw_pressed) {
      s_states[i].raw_pressed = raw_pressed;
      s_states[i].last_change_ms = now_ms;
    }

#if SSC_SWITCH_DEBOUNCE_ENABLE
    if (raw_pressed == s_states[i].stable_pressed) continue;
    if ((uint32_t)(now_ms - s_states[i].last_change_ms) < 20) continue;
    const bool next_pressed = raw_pressed;
#else
    if (raw_pressed == s_states[i].stable_pressed) continue;
    const bool next_pressed = raw_pressed;
#endif

    s_states[i].stable_pressed = next_pressed;
    if (next_pressed) {
      if (kMappedButtons[i] == BTN_NONE) continue;
      ir_inject_button(kMappedButtons[i], 250);
    } else if (ir_active_btn() == kMappedButtons[i]) {
      ir_inject_button(BTN_NONE);
    }
  }

  if ((uint32_t)(now_ms - s_last_raw_log_ms) >= SSC_SWITCH_RAW_LOG_INTERVAL_MS) {
    s_last_raw_log_ms = now_ms;
    Serial.printf("SW RAW %lu | %u:%u %u:%u %u:%u %u:%u %u:%u %u:%u\n",
                  (unsigned long)now_ms,
                  kButtonPins[0], raw_pressed_values[0] ? 1 : 0,
                  kButtonPins[1], raw_pressed_values[1] ? 1 : 0,
                  kButtonPins[2], raw_pressed_values[2] ? 1 : 0,
                  kButtonPins[3], raw_pressed_values[3] ? 1 : 0,
                  kButtonPins[4], raw_pressed_values[4] ? 1 : 0,
                  kButtonPins[5], raw_pressed_values[5] ? 1 : 0);
  }

#if SSC_SWITCH_DEBOUNCE_ENABLE
  if ((uint32_t)(now_ms - s_last_debounced_log_ms) >= SSC_SWITCH_RAW_LOG_INTERVAL_MS) {
    s_last_debounced_log_ms = now_ms;
    Serial.printf("SW DEB %lu | %u:%u %u:%u %u:%u %u:%u %u:%u %u:%u\n",
                  (unsigned long)now_ms,
                  kButtonPins[0], s_states[0].stable_pressed ? 1 : 0,
                  kButtonPins[1], s_states[1].stable_pressed ? 1 : 0,
                  kButtonPins[2], s_states[2].stable_pressed ? 1 : 0,
                  kButtonPins[3], s_states[3].stable_pressed ? 1 : 0,
                  kButtonPins[4], s_states[4].stable_pressed ? 1 : 0,
                  kButtonPins[5], s_states[5].stable_pressed ? 1 : 0);
  }
#endif
}
