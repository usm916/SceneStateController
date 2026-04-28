#include "physical_button_input.h"

#include "config.h"
#include "ir_module.h"

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

constexpr uint16_t kDebounceMs = 20;

struct DebounceState {
  bool raw_pressed;
  bool stable_pressed;
  uint32_t last_change_ms;
};

DebounceState s_states[6] = {};

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
  return false;
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
}

void physical_button_input_poll() {
  if (!physical_button_input_enabled()) return;

  const uint32_t now_ms = millis();
  for (uint8_t i = 0; i < 6; i++) {
    const bool raw_pressed = read_pressed(kButtonPins[i]);
    if (raw_pressed != s_states[i].raw_pressed) {
      s_states[i].raw_pressed = raw_pressed;
      s_states[i].last_change_ms = now_ms;
    }

    if (raw_pressed == s_states[i].stable_pressed) continue;
    if ((now_ms - s_states[i].last_change_ms) < kDebounceMs) continue;

    s_states[i].stable_pressed = raw_pressed;
    if (raw_pressed) {
      if (kMappedButtons[i] == BTN_NONE) continue;
      ir_inject_button(kMappedButtons[i], 250);
    } else if (ir_active_btn() == kMappedButtons[i]) {
      ir_inject_button(BTN_NONE);
    }
  }
}
