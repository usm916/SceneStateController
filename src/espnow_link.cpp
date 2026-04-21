#include "espnow_link.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <string.h>

#include "config.h"

namespace {

static constexpr uint8_t kRoleOff = 0;
static constexpr uint8_t kRoleManager = 1;
static constexpr uint8_t kRoleLedNode = 2;
static constexpr uint8_t kPacketMagic = 0xA7;
static constexpr uint8_t kPacketVersion = 1;
static const uint8_t kDefaultPeerMac[6] = SSC_ESPNOW_LINK_PEER_MAC;
static EspnowLinkConfig s_config = {
    ESPNOW_LINK_ROLE_OFF,
    SSC_ESPNOW_LINK_CHANNEL,
    {kDefaultPeerMac[0], kDefaultPeerMac[1], kDefaultPeerMac[2], kDefaultPeerMac[3], kDefaultPeerMac[4], kDefaultPeerMac[5]},
};

enum LedLinkCmd : uint8_t {
  CMD_SET_UPDATES = 1,
  CMD_SET_BRIGHTNESS = 2,
  CMD_SET_PATTERN = 3,
  CMD_SET_STRIP_SCENE = 4,
  CMD_SET_ALL_SCENE = 5,
};

struct __attribute__((packed)) LedLinkPacket {
  uint8_t magic;
  uint8_t version;
  uint8_t cmd;
  uint8_t seq;
  int16_t value1;
  int16_t value2;
};

static uint8_t s_seq = 0;
static bool s_ready = false;
static bool s_peer_added = false;
static constexpr size_t kRxQueueSize = 8;
static LedLinkPacket s_rx_queue[kRxQueueSize];
static volatile uint8_t s_rx_head = 0;
static volatile uint8_t s_rx_tail = 0;
static volatile uint32_t s_rx_dropped = 0;
static volatile uint32_t s_rx_received = 0;

bool link_enabled() {
#if SSC_ESPNOW_LINK_ENABLE
  return true;
#else
  return false;
#endif
}

uint8_t role() {
  return s_config.role;
}

bool queue_push(const LedLinkPacket& packet) {
  const uint8_t next = (uint8_t)((s_rx_head + 1) % kRxQueueSize);
  if (next == s_rx_tail) {
    s_rx_dropped++;
    return false;
  }
  s_rx_queue[s_rx_head] = packet;
  s_rx_head = next;
  s_rx_received++;
  return true;
}

bool queue_pop(LedLinkPacket* out_packet) {
  if (out_packet == nullptr || s_rx_tail == s_rx_head) {
    return false;
  }
  *out_packet = s_rx_queue[s_rx_tail];
  s_rx_tail = (uint8_t)((s_rx_tail + 1) % kRxQueueSize);
  return true;
}

void on_recv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  (void)info;
  if (!link_enabled() || role() != kRoleLedNode) return;
  if (data == nullptr || len != (int)sizeof(LedLinkPacket)) return;

  LedLinkPacket packet = {};
  memcpy(&packet, data, sizeof(packet));
  if (packet.magic != kPacketMagic || packet.version != kPacketVersion) return;
  (void)queue_push(packet);
}

void apply_packet(const LedLinkPacket& packet) {
  switch (packet.cmd) {
    case CMD_SET_UPDATES:
      led_set_updates_enabled(packet.value1 != 0);
      break;
    case CMD_SET_BRIGHTNESS:
      if (packet.value1 >= 0 && packet.value1 <= 100) {
        led_set_global_brightness_pct((uint8_t)packet.value1);
      }
      break;
    case CMD_SET_PATTERN:
      if (packet.value1 >= (int16_t)LEDP_IDLE && packet.value1 <= (int16_t)LEDP_ERROR) {
        led_set_pattern((LedPattern)packet.value1);
      }
      break;
    case CMD_SET_STRIP_SCENE:
      if (packet.value1 >= 0 && packet.value1 < SSC_LED_STRIP_COUNT &&
          packet.value2 >= (int16_t)LEDSCENE_SOLID && packet.value2 <= (int16_t)LEDSCENE_FADE_OUT_3S) {
        led_set_strip_scene((uint8_t)packet.value1, (LedStripScene)packet.value2);
      }
      break;
    case CMD_SET_ALL_SCENE:
      if (packet.value1 >= (int16_t)LEDSCENE_SOLID && packet.value1 <= (int16_t)LEDSCENE_FADE_OUT_3S) {
        for (uint8_t strip = 0; strip < SSC_LED_STRIP_COUNT; ++strip) {
          led_set_strip_scene(strip, (LedStripScene)packet.value1);
        }
      }
      break;
    default:
      break;
  }
}

bool send_packet(LedLinkCmd cmd, int16_t value1, int16_t value2) {
  if (!s_ready || !s_peer_added || role() != kRoleManager) return false;
  LedLinkPacket packet = {kPacketMagic, kPacketVersion, (uint8_t)cmd, ++s_seq, value1, value2};
  const esp_err_t err = esp_now_send(s_config.peer_mac, (const uint8_t*)&packet, sizeof(packet));
  return err == ESP_OK;
}

}  // namespace

void espnow_link_set_config(const EspnowLinkConfig& config) {
  s_config = config;
  if (s_config.role != ESPNOW_LINK_ROLE_MANAGER && s_config.role != ESPNOW_LINK_ROLE_LED_NODE) {
    s_config.role = ESPNOW_LINK_ROLE_OFF;
  }
  if (s_config.channel > 14) {
    s_config.channel = SSC_ESPNOW_LINK_CHANNEL;
  }
}

EspnowLinkConfig espnow_link_get_config() {
  return s_config;
}

void espnow_link_setup() {
  if (!link_enabled() || role() == kRoleOff) return;

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESPNOW] init failed");
    return;
  }

  esp_now_register_recv_cb(on_recv);

  esp_now_peer_info_t peer_info = {};
  memcpy(peer_info.peer_addr, s_config.peer_mac, 6);
  peer_info.ifidx = WIFI_IF_STA;
  peer_info.channel = s_config.channel;
  peer_info.encrypt = false;

  if (!esp_now_is_peer_exist(s_config.peer_mac)) {
    if (esp_now_add_peer(&peer_info) != ESP_OK) {
      Serial.println("[ESPNOW] add peer failed");
      return;
    }
  }

  s_ready = true;
  s_peer_added = true;
  Serial.println(role() == kRoleManager ? "[ESPNOW] link MANAGER ready" : "[ESPNOW] link NODE ready");
}

void espnow_link_poll() {
  if (!s_ready || role() != kRoleLedNode) return;
  LedLinkPacket packet = {};
  while (queue_pop(&packet)) {
    apply_packet(packet);
  }
}

bool espnow_link_is_manager() {
  return link_enabled() && role() == kRoleManager;
}

bool espnow_link_is_led_node() {
  return link_enabled() && role() == kRoleLedNode;
}

bool espnow_link_send_updates_enabled(bool enabled) {
  return send_packet(CMD_SET_UPDATES, enabled ? 1 : 0, 0);
}

bool espnow_link_send_brightness_pct(uint8_t brightness_pct) {
  return send_packet(CMD_SET_BRIGHTNESS, brightness_pct, 0);
}

bool espnow_link_send_pattern(LedPattern pattern) {
  return send_packet(CMD_SET_PATTERN, (int16_t)pattern, 0);
}

bool espnow_link_send_strip_scene(uint8_t strip_index, LedStripScene scene) {
  return send_packet(CMD_SET_STRIP_SCENE, strip_index, (int16_t)scene);
}

bool espnow_link_send_all_scene(LedStripScene scene) {
  return send_packet(CMD_SET_ALL_SCENE, (int16_t)scene, 0);
}
