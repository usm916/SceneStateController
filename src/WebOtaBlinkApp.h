#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>

class WebOtaBlinkApp
{
public:
  void begin();
  void loop();

private:
  static constexpr int kMaxWifiSlots = 3;
  static constexpr int kWifiRetryPerSsid = 3;
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

  void loadSettings();
  void saveSettings();
  void setDefaultSettings();

  bool connectToSavedWifiList();
  bool waitForWifiConnected(unsigned long timeoutMs);
  bool applyCustomStaMac();
  void startFallbackAp();

  void registerRoutes();

  void handleRoot(AsyncWebServerRequest* request);
  void handleSaveWifi(AsyncWebServerRequest* request);
  void handleReboot(AsyncWebServerRequest* request);
  void handleNotFound(AsyncWebServerRequest* request);

  String makeHtml() const;
  String htmlEscape(const String& s) const;
  String currentModeText() const;
  String currentIpText() const;
  String customMacText() const;
};
