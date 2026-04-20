#pragma once

#include <stdint.h>

#include "led_module.h"

enum EspnowLinkRole : uint8_t {
  ESPNOW_LINK_ROLE_OFF = 0,
  ESPNOW_LINK_ROLE_MANAGER = 1,
  ESPNOW_LINK_ROLE_LED_NODE = 2,
};

struct EspnowLinkConfig {
  uint8_t role;
  uint8_t channel;
  uint8_t peer_mac[6];
};

void espnow_link_set_config(const EspnowLinkConfig& config);
EspnowLinkConfig espnow_link_get_config();

void espnow_link_setup();
void espnow_link_poll();

bool espnow_link_is_manager();
bool espnow_link_is_led_node();

bool espnow_link_send_updates_enabled(bool enabled);
bool espnow_link_send_brightness_pct(uint8_t brightness_pct);
bool espnow_link_send_pattern(LedPattern pattern);
bool espnow_link_send_strip_scene(uint8_t strip_index, LedStripScene scene);
bool espnow_link_send_all_scene(LedStripScene scene);
