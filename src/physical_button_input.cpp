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
uint32_t s_last_hold_inject_ms[6] = {};
uint32_t s_press_start_ms[6] = {};

constexpr uint16_t kSwitchInjectHoldMs = 250;
constexpr uint16_t kHoldRefreshIntervalMs = 120;

bool is_hold_button(RemoteButton btn) {
  return btn == BTN_PREV || btn == BTN_NEXT;
}

bool has_release_based_long_press(uint8_t index) {
  return index == 0 || index == 3 || index == 4;  // D18, D22, D32
}

RemoteButton resolve_release_button(uint8_t index, uint32_t press_duration_ms) {
  switch (index) {
    case 0:  // D18
      if (press_duration_ms < SSC_SWITCH_LONG_PRESS_SHORT_MAX_MS) return (RemoteButton)SSC_SWITCH_D18_SHORT_BUTTON_CODE;
      if (press_duration_ms < SSC_SWITCH_LONG_PRESS_MEDIUM_MAX_MS) return (RemoteButton)SSC_SWITCH_D18_MEDIUM_BUTTON_CODE;
      return (RemoteButton)SSC_SWITCH_D18_LONG_BUTTON_CODE;
    case 3:  // D22
      if (press_duration_ms < SSC_SWITCH_LONG_PRESS_SHORT_MAX_MS) return (RemoteButton)SSC_SWITCH_D22_SHORT_BUTTON_CODE;
      if (press_duration_ms < SSC_SWITCH_LONG_PRESS_MEDIUM_MAX_MS) return (RemoteButton)SSC_SWITCH_D22_MEDIUM_BUTTON_CODE;
      return (RemoteButton)SSC_SWITCH_D22_LONG_BUTTON_CODE;
    case 4:  // D32
      if (press_duration_ms < SSC_SWITCH_LONG_PRESS_SHORT_MAX_MS) return (RemoteButton)SSC_SWITCH_D32_SHORT_BUTTON_CODE;
      if (press_duration_ms < SSC_SWITCH_LONG_PRESS_MEDIUM_MAX_MS) return (RemoteButton)SSC_SWITCH_D32_MEDIUM_BUTTON_CODE;
      return (RemoteButton)SSC_SWITCH_D32_LONG_BUTTON_CODE;
    default:
      return kMappedButtons[index];
  }
}

bool read_pressed(uint8_t pin) {
  const int value = digitalRead(pin);
#if SSC_SWITCH_ACTIVE_LOW
  return value == LOW;
#else
  return value == HIGH;
#endif
}

void log_switch_transition(uint32_t now_ms,
                           uint8_t index,
                           const char* stage,
                           bool pressed,
                           uint32_t elapsed_ms,
                           RemoteButton mapped_button) {
#if SSC_IR_LOG_ENABLE
  Serial.printf("SW %s %lu | idx=%u pin=%u state=%u elapsed=%lu code=0x%02X\n",
                stage,
                (unsigned long)now_ms,
                (unsigned int)index,
                (unsigned int)kButtonPins[index],
                pressed ? 1U : 0U,
                (unsigned long)elapsed_ms,
                (uint8_t)mapped_button);
#else
  (void)now_ms;
  (void)index;
  (void)stage;
  (void)pressed;
  (void)elapsed_ms;
  (void)mapped_button;
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
    s_press_start_ms[i] = pressed ? millis() : 0;
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

  for (uint8_t i = 0; i < 6; i++) {
    const bool raw_pressed = read_pressed(kButtonPins[i]);
    if (raw_pressed != s_states[i].raw_pressed) {
      s_states[i].raw_pressed = raw_pressed;
      s_states[i].last_change_ms = now_ms;
      log_switch_transition(now_ms, i, "RAW", raw_pressed, 0, kMappedButtons[i]);
    }

#if SSC_SWITCH_DEBOUNCE_ENABLE
    if (raw_pressed == s_states[i].stable_pressed) continue;
    const uint32_t stable_for_ms = (uint32_t)(now_ms - s_states[i].last_change_ms);
    if (stable_for_ms < 20) {
      log_switch_transition(now_ms, i, "WAIT", raw_pressed, stable_for_ms, kMappedButtons[i]);
      continue;
    }
    const bool next_pressed = raw_pressed;
#else
    if (raw_pressed == s_states[i].stable_pressed) continue;
    const bool next_pressed = raw_pressed;
#endif

    s_states[i].stable_pressed = next_pressed;
    log_switch_transition(now_ms, i, "DEB", next_pressed,
                          (uint32_t)(now_ms - s_states[i].last_change_ms),
                          kMappedButtons[i]);
    if (next_pressed) {
      s_press_start_ms[i] = now_ms;
      if (has_release_based_long_press(i)) {
        log_switch_transition(now_ms, i, "PRESS", next_pressed, 0, kMappedButtons[i]);
        continue;
      }
      if (kMappedButtons[i] == BTN_NONE) {
        log_switch_transition(now_ms, i, "SKIP", next_pressed, 0, kMappedButtons[i]);
        continue;
      }
      ir_inject_button(kMappedButtons[i], kSwitchInjectHoldMs);
      s_last_hold_inject_ms[i] = now_ms;
      log_switch_transition(now_ms, i, "INJECT", next_pressed, kSwitchInjectHoldMs, kMappedButtons[i]);
    } else {
      const uint32_t press_duration_ms =
          (s_press_start_ms[i] == 0) ? 0 : (uint32_t)(now_ms - s_press_start_ms[i]);
      s_press_start_ms[i] = 0;

      if (has_release_based_long_press(i)) {
        const RemoteButton release_button = resolve_release_button(i, press_duration_ms);
        if (release_button != BTN_NONE) {
          ir_inject_button(release_button, kSwitchInjectHoldMs);
          s_last_hold_inject_ms[i] = now_ms;
          log_switch_transition(now_ms, i, "REL-INJECT", next_pressed, press_duration_ms, release_button);
        } else {
          log_switch_transition(now_ms, i, "REL-SKIP", next_pressed, press_duration_ms, release_button);
        }
      } else if (ir_active_btn() == kMappedButtons[i]) {
        ir_inject_button(BTN_NONE);
        s_last_hold_inject_ms[i] = 0;
        log_switch_transition(now_ms, i, "RELEASE", next_pressed, press_duration_ms, kMappedButtons[i]);
      }
    }

    continue;
  }

  for (uint8_t i = 0; i < 6; i++) {
    if (!s_states[i].stable_pressed) continue;
    if (!is_hold_button(kMappedButtons[i])) continue;
    if ((uint32_t)(now_ms - s_last_hold_inject_ms[i]) < kHoldRefreshIntervalMs) continue;
    ir_inject_button(kMappedButtons[i], kSwitchInjectHoldMs);
    s_last_hold_inject_ms[i] = now_ms;
    log_switch_transition(now_ms, i, "HOLD", true, kSwitchInjectHoldMs, kMappedButtons[i]);
  }

}
