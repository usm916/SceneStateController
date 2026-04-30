#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include "scene_state.h"
#include "espnow_link.h"

class WebOtaBlinkApp
{
public:
  void begin();
  void loop();
  bool getSavedRuntimeMode(uint8_t* out_mode) const;
  bool setWifiSlot(int slot, const String& ssid, const String& pass);

private:
  static constexpr int kMaxWifiSlots = 3;
  static constexpr int kWifiRetryPerSsid = 1;
  static constexpr unsigned long kWifiConnectTimeoutMs = 8000;

  struct WifiSlot
  {
    char ssid[33];
    char pass[65];
  };

  Preferences prefs_;
  AsyncWebServer server_{80};

  WifiSlot wifiSlots_[kMaxWifiSlots]{};
  bool apMode_ = false;

  const char* fallbackApSsid_ = "ESP32-Setup";
  const char* fallbackApPass_ = "12345678";

  uint8_t customStaMac_[6] = {0x02, 0x10, 0x00, 0x00, 0x00, 0x01};
  bool restartScheduled_ = false;
  unsigned long restartAtMs_ = 0;
  bool otaUploadOk_ = false;
  bool webPrevToggleOn_ = false;
  bool webNextToggleOn_ = false;
  unsigned long nextWebToggleInjectAtMs_ = 0;
  bool hasSavedRuntimeMode_ = false;
  uint8_t savedRuntimeMode_ = 0;
  EspnowLinkConfig espnowConfig_{};

  void loadSettings();
  void saveSettings();
  void saveRuntimeModeSetting(uint8_t mode);
  void setDefaultSettings();

  bool connectToSavedWifiList();
  bool waitForWifiConnected(unsigned long timeoutMs);
  bool applyCustomStaMac();
  void startFallbackAp();

  void registerRoutes();

  void handleRoot(AsyncWebServerRequest* request);
  void handleSaveWifi(AsyncWebServerRequest* request);
  void handleSaveControl(AsyncWebServerRequest* request);
  void handleLedControl(AsyncWebServerRequest* request);
  void handleSceneControl(AsyncWebServerRequest* request);
  void handleRuntimeMode(AsyncWebServerRequest* request);
  void handlePressButton(AsyncWebServerRequest* request);
  void handleSetToggle(AsyncWebServerRequest* request);
  void handleReboot(AsyncWebServerRequest* request);
  void handleOtaPage(AsyncWebServerRequest* request);
  void handleOtaUpload(AsyncWebServerRequest* request, const String& filename,
                       size_t index, uint8_t* data, size_t len, bool final);
  void handleOtaDone(AsyncWebServerRequest* request);
  void handleNotFound(AsyncWebServerRequest* request);

  String renderControllerSettingsSection() const;
  String makeHtml() const;
  String htmlEscape(const String& s) const;
  String currentModeText() const;
  String currentIpText() const;
  String customMacText() const;
  String espnowPeerMacText() const;
  static bool parseMacText(const String& text, uint8_t out_mac[6]);
  bool parseIntParam(AsyncWebServerRequest* request, const String& key, int32_t* out_value) const;
  bool parseSceneParam(const String& key, SceneId* out_scene) const;
  bool parseRemoteButtonParam(const String& key, uint8_t* out_button_code) const;
  void sendEspnowWebEcho(AsyncWebServerRequest* request, const String& payload) const;
};
