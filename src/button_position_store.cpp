#include "button_position_store.h"

#include <Preferences.h>
#include <string.h>

namespace {
constexpr char kPrefsNs[] = "btn_pos";
constexpr char kPrefsVerKey[] = "ver";
constexpr char kPrefsZeroKey[] = "zero";
constexpr char kPrefsMaskKey[] = "mask";
constexpr char kPrefsRelPrefix[] = "r";
constexpr uint32_t kPrefsVersion = 1;
constexpr uint8_t kMaxButtons = 10;

Preferences s_prefs;
int32_t s_zero_steps = 0;
int32_t s_relative_steps[kMaxButtons] = {0};
uint16_t s_valid_mask = 0;

bool is_valid_index(uint8_t index) {
  return index < kMaxButtons;
}

void pref_key_for_index(uint8_t index, char* out_key, size_t out_key_size) {
  if (out_key_size < 4) return;
  out_key[0] = kPrefsRelPrefix[0];
  out_key[1] = (char)('0' + (index / 10));
  out_key[2] = (char)('0' + (index % 10));
  out_key[3] = '\0';
}

bool parse_digit_button(RemoteButton button, uint8_t* out_index) {
  switch (button) {
    case BTN_0: *out_index = 0; return true;
    case BTN_1: *out_index = 1; return true;
    case BTN_2: *out_index = 2; return true;
    case BTN_3: *out_index = 3; return true;
    case BTN_4: *out_index = 4; return true;
    case BTN_5: *out_index = 5; return true;
    case BTN_6: *out_index = 6; return true;
    case BTN_7: *out_index = 7; return true;
    case BTN_8: *out_index = 8; return true;
    case BTN_9: *out_index = 9; return true;
    default: return false;
  }
}
}  // namespace

void button_position_store_setup() {
  (void)button_position_store_load();
}

bool button_position_store_load() {
  s_zero_steps = 0;
  s_valid_mask = 0;
  for (uint8_t i = 0; i < kMaxButtons; ++i) {
    s_relative_steps[i] = 0;
  }

  if (!s_prefs.begin(kPrefsNs, true)) return false;

  const uint32_t version = s_prefs.getUInt(kPrefsVerKey, 0);
  if (version == kPrefsVersion) {
    s_zero_steps = s_prefs.getInt(kPrefsZeroKey, 0);
    s_valid_mask = (uint16_t)s_prefs.getUShort(kPrefsMaskKey, 0);

    for (uint8_t i = 0; i < kMaxButtons; ++i) {
      char key[4] = {0};
      pref_key_for_index(i, key, sizeof(key));
      s_relative_steps[i] = s_prefs.getInt(key, 0);
    }
  }

  s_prefs.end();
  return true;
}

bool button_position_store_save() {
  if (!s_prefs.begin(kPrefsNs, false)) return false;

  s_prefs.putUInt(kPrefsVerKey, kPrefsVersion);
  s_prefs.putInt(kPrefsZeroKey, s_zero_steps);
  s_prefs.putUShort(kPrefsMaskKey, s_valid_mask);

  for (uint8_t i = 0; i < kMaxButtons; ++i) {
    char key[4] = {0};
    pref_key_for_index(i, key, sizeof(key));
    s_prefs.putInt(key, s_relative_steps[i]);
  }

  s_prefs.end();
  return true;
}

void button_position_store_set_zero(int32_t current_steps) {
  s_zero_steps = current_steps;
}

int32_t button_position_store_zero_steps() {
  return s_zero_steps;
}

bool button_position_store_parse_button_token(const char* token, uint8_t* out_index) {
  if (token == nullptr || out_index == nullptr) return false;

  if (strncmp(token, "BTN_", 4) == 0) {
    token += 4;
  }

  if (strlen(token) != 1 || token[0] < '0' || token[0] > '9') return false;

  *out_index = (uint8_t)(token[0] - '0');
  return true;
}

bool button_position_store_index_from_remote(RemoteButton button, uint8_t* out_index) {
  if (out_index == nullptr) return false;
  return parse_digit_button(button, out_index);
}

bool button_position_store_record_relative(uint8_t index, int32_t relative_steps) {
  if (!is_valid_index(index)) return false;
  s_relative_steps[index] = relative_steps;
  s_valid_mask |= (uint16_t)(1U << index);
  return true;
}

bool button_position_store_record_current(uint8_t index, int32_t current_steps) {
  if (!is_valid_index(index)) return false;
  return button_position_store_record_relative(index, current_steps - s_zero_steps);
}

bool button_position_store_has(uint8_t index) {
  if (!is_valid_index(index)) return false;
  return (s_valid_mask & (uint16_t)(1U << index)) != 0;
}

bool button_position_store_relative(uint8_t index, int32_t* out_relative_steps) {
  if (!is_valid_index(index) || out_relative_steps == nullptr) return false;
  if (!button_position_store_has(index)) return false;
  *out_relative_steps = s_relative_steps[index];
  return true;
}

bool button_position_store_target(uint8_t index, int32_t* out_target_steps) {
  if (out_target_steps == nullptr) return false;
  int32_t relative_steps = 0;
  if (!button_position_store_relative(index, &relative_steps)) return false;
  *out_target_steps = s_zero_steps + relative_steps;
  return true;
}

void button_position_store_print_info(Stream& out) {
  out.print("btn_zero_steps=");
  out.println(s_zero_steps);

  for (uint8_t i = 0; i < kMaxButtons; ++i) {
    out.print("btn_");
    out.print(i);
    out.print("_set=");
    out.println(button_position_store_has(i) ? 1 : 0);

    if (!button_position_store_has(i)) continue;

    int32_t relative_steps = 0;
    int32_t target_steps = 0;
    button_position_store_relative(i, &relative_steps);
    button_position_store_target(i, &target_steps);

    out.print("btn_");
    out.print(i);
    out.print("_relative_steps=");
    out.println(relative_steps);
    out.print("btn_");
    out.print(i);
    out.print("_target_steps=");
    out.println(target_steps);
  }
}
