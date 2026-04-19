#include "WebOtaBlinkApp.h"

static const char kControllerSettingsSectionTemplate[] =
#include "web_controller_settings_section.h"
;

#include <Update.h>
#include <esp_wifi.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "button_position_store.h"
#include "config.h"
#include "elevator_module.h"
#include "ir_module.h"
#include "led_module.h"
#include "tmc2209_module.h"

// ------------------------------------------------------------
// public
// ------------------------------------------------------------
void WebOtaBlinkApp::begin()
{
  // Serial.begin(115200);
  // delay(200);
  Serial.println();
  Serial.println("Booting...");

  loadSettings();

  if (!connectToSavedWifiList())
  {
    Serial.println("[WIFI] All candidates failed. Starting fallback AP.");
    startFallbackAp();
  }
  else
  {
    apMode_ = false;
  }

  registerRoutes();
  server_.begin();

  Serial.println("[HTTP] Server started");
  Serial.print("[HTTP] Open: http://");
  Serial.println(currentIpText());
  Serial.print("[HTTP] OTA : http://");
  Serial.print(currentIpText());
  Serial.println("/update");
}

void WebOtaBlinkApp::loop()
{
  const unsigned long nowMs = millis();
  if ((webPrevToggleOn_ || webNextToggleOn_) && static_cast<long>(nowMs - nextWebToggleInjectAtMs_) >= 0)
  {
    if (webPrevToggleOn_ && !webNextToggleOn_)
    {
      ir_inject_button(BTN_PREV, 260);
    }
    else if (webNextToggleOn_ && !webPrevToggleOn_)
    {
      ir_inject_button(BTN_NEXT, 260);
    }

    nextWebToggleInjectAtMs_ = nowMs + 120;
  }

  if (restartScheduled_ && static_cast<long>(millis() - restartAtMs_) >= 0)
  {
    ESP.restart();
  }
}

// ------------------------------------------------------------
// settings
// ------------------------------------------------------------
void WebOtaBlinkApp::setDefaultSettings()
{
  memset(wifiSlots_, 0, sizeof(wifiSlots_));

  strncpy(wifiSlots_[0].ssid, "SSID_1", sizeof(wifiSlots_[0].ssid) - 1);
  strncpy(wifiSlots_[0].pass, "PASSWORD_1", sizeof(wifiSlots_[0].pass) - 1);

  strncpy(wifiSlots_[1].ssid, "SSID_2", sizeof(wifiSlots_[1].ssid) - 1);
  strncpy(wifiSlots_[1].pass, "PASSWORD_2", sizeof(wifiSlots_[1].pass) - 1);

  strncpy(wifiSlots_[2].ssid, "SSID_3", sizeof(wifiSlots_[2].ssid) - 1);
  strncpy(wifiSlots_[2].pass, "PASSWORD_3", sizeof(wifiSlots_[2].pass) - 1);
}

void WebOtaBlinkApp::loadSettings()
{
  prefs_.begin("app", true);

  for (int i = 0; i < kMaxWifiSlots; ++i)
  {
    const String ssidKey = "ssid" + String(i);
    const String passKey = "pass" + String(i);

    const String ssid = prefs_.getString(ssidKey.c_str(), "");
    const String pass = prefs_.getString(passKey.c_str(), "");

    memset(wifiSlots_[i].ssid, 0, sizeof(wifiSlots_[i].ssid));
    memset(wifiSlots_[i].pass, 0, sizeof(wifiSlots_[i].pass));

    ssid.toCharArray(wifiSlots_[i].ssid, sizeof(wifiSlots_[i].ssid));
    pass.toCharArray(wifiSlots_[i].pass, sizeof(wifiSlots_[i].pass));
  }

  prefs_.end();

  bool allEmpty = true;
  for (int i = 0; i < kMaxWifiSlots; ++i)
  {
    if (strlen(wifiSlots_[i].ssid) > 0)
    {
      allEmpty = false;
      break;
    }
  }

  if (allEmpty)
  {
    setDefaultSettings();
  }
}

void WebOtaBlinkApp::saveSettings()
{
  prefs_.begin("app", false);

  for (int i = 0; i < kMaxWifiSlots; ++i)
  {
    const String ssidKey = "ssid" + String(i);
    const String passKey = "pass" + String(i);

    prefs_.putString(ssidKey.c_str(), wifiSlots_[i].ssid);
    prefs_.putString(passKey.c_str(), wifiSlots_[i].pass);
  }

  prefs_.end();
}

// ------------------------------------------------------------
// wifi
// ------------------------------------------------------------
bool WebOtaBlinkApp::waitForWifiConnected(unsigned long timeoutMs)
{
  const unsigned long startMs = millis();
  while (millis() - startMs < timeoutMs)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      return true;
    }
    delay(200);
  }
  return false;
}

bool WebOtaBlinkApp::applyCustomStaMac()
{
  WiFi.mode(WIFI_STA);
  delay(100);

  esp_err_t err = esp_wifi_stop();
  if (err != ESP_OK && err != ESP_ERR_WIFI_NOT_STOPPED)
  {
    Serial.print("[WIFI] esp_wifi_stop err=");
    Serial.println((int)err);
  }
  delay(50);

  err = esp_wifi_set_mac(WIFI_IF_STA, customStaMac_);
  if (err != ESP_OK)
  {
    Serial.print("[WIFI] Failed to set STA MAC, err=");
    Serial.println((int)err);
    esp_wifi_start();
    delay(50);
    return false;
  }

  err = esp_wifi_start();
  if (err != ESP_OK)
  {
    Serial.print("[WIFI] esp_wifi_start err=");
    Serial.println((int)err);
    return false;
  }

  delay(50);

  uint8_t mac[6] = {0};
  if (esp_wifi_get_mac(WIFI_IF_STA, mac) == ESP_OK)
  {
    char buf[18];
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.print("[WIFI] STA MAC: ");
    Serial.println(buf);
  }

  return true;
}

bool WebOtaBlinkApp::connectToSavedWifiList()
{
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);

  applyCustomStaMac();

  for (int i = 0; i < kMaxWifiSlots; ++i)
  {
    if (strlen(wifiSlots_[i].ssid) == 0)
    {
      continue;
    }

    for (int attempt = 1; attempt <= kWifiRetryPerSsid; ++attempt)
    {
      Serial.println();
      Serial.print("[WIFI] Trying SSID: ");
      Serial.print(wifiSlots_[i].ssid);
      Serial.print("  attempt ");
      Serial.print(attempt);
      Serial.print("/");
      Serial.println(kWifiRetryPerSsid);

      WiFi.disconnect();
      delay(200);

      WiFi.begin(wifiSlots_[i].ssid, wifiSlots_[i].pass);

      if (waitForWifiConnected(kWifiConnectTimeoutMs))
      {
        Serial.println("[WIFI] Connected");
        Serial.print("[WIFI] SSID: ");
        Serial.println(WiFi.SSID());
        Serial.print("[WIFI] IP: ");
        Serial.println(WiFi.localIP());
        Serial.print("[WIFI] MAC: ");
        Serial.println(WiFi.macAddress());
        return true;
      }

      Serial.println("[WIFI] Failed");
    }
  }

  return false;
}

void WebOtaBlinkApp::startFallbackAp()
{
  WiFi.disconnect(true);
  delay(100);

  WiFi.mode(WIFI_AP);
  apMode_ = true;

  const bool ok = WiFi.softAP(fallbackApSsid_, fallbackApPass_);
  if (ok)
  {
    Serial.println("[AP] Fallback AP started");
    Serial.print("[AP] SSID: ");
    Serial.println(fallbackApSsid_);
    Serial.print("[AP] PASS: ");
    Serial.println(fallbackApPass_);
    Serial.print("[AP] IP: ");
    Serial.println(WiFi.softAPIP());
  }
  else
  {
    Serial.println("[AP] Failed to start AP");
  }
}

// ------------------------------------------------------------
// web
// ------------------------------------------------------------
void WebOtaBlinkApp::registerRoutes()
{
  server_.on("/", HTTP_GET, [this](AsyncWebServerRequest* request) { handleRoot(request); });
  server_.on("/save-wifi", HTTP_POST, [this](AsyncWebServerRequest* request) { handleSaveWifi(request); });
  server_.on("/save-control", HTTP_POST, [this](AsyncWebServerRequest* request) { handleSaveControl(request); });
  server_.on("/led-control", HTTP_POST, [this](AsyncWebServerRequest* request) { handleLedControl(request); });
  server_.on("/press-btn", HTTP_POST, [this](AsyncWebServerRequest* request) { handlePressButton(request); });
  server_.on("/set-toggle", HTTP_POST, [this](AsyncWebServerRequest* request) { handleSetToggle(request); });
  server_.on("/reboot", HTTP_GET, [this](AsyncWebServerRequest* request) { handleReboot(request); });
  server_.on("/update", HTTP_GET, [this](AsyncWebServerRequest* request) { handleOtaPage(request); });
  server_.on("/update", HTTP_POST,
             [this](AsyncWebServerRequest* request) { handleOtaDone(request); },
             [this](AsyncWebServerRequest* request, const String& filename,
                    size_t index, uint8_t* data, size_t len, bool final)
             {
               handleOtaUpload(request, filename, index, data, len, final);
             });
  server_.onNotFound([this](AsyncWebServerRequest* request) { handleNotFound(request); });
}

void WebOtaBlinkApp::handleRoot(AsyncWebServerRequest* request)
{
  request->send(200, "text/html; charset=utf-8", makeHtml());
}

void WebOtaBlinkApp::handleSaveWifi(AsyncWebServerRequest* request)
{
  for (int i = 0; i < kMaxWifiSlots; ++i)
  {
    const String ssidKey = "ssid" + String(i);
    const String passKey = "pass" + String(i);

    const AsyncWebParameter* ssidParam = request->getParam(ssidKey, true);
    const AsyncWebParameter* passParam = request->getParam(passKey, true);

    const String ssid = (ssidParam != nullptr) ? ssidParam->value() : "";
    const String pass = (passParam != nullptr) ? passParam->value() : "";

    memset(wifiSlots_[i].ssid, 0, sizeof(wifiSlots_[i].ssid));
    memset(wifiSlots_[i].pass, 0, sizeof(wifiSlots_[i].pass));

    ssid.toCharArray(wifiSlots_[i].ssid, sizeof(wifiSlots_[i].ssid));
    pass.toCharArray(wifiSlots_[i].pass, sizeof(wifiSlots_[i].pass));
  }

  saveSettings();

  String html;
  html += "<!DOCTYPE html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no'>";
  html += "<title>Saved</title></head><body style='font-family:Arial,sans-serif;max-width:720px;margin:24px auto;padding:0 16px;'>";
  html += "<h1>Wi-Fi settings saved</h1>";
  html += "<p>Device will reboot and retry the Wi-Fi list.</p>";
  html += "</body></html>";

  request->send(200, "text/html; charset=utf-8", html);
  restartScheduled_ = true;
  restartAtMs_ = millis() + 800;
}

void WebOtaBlinkApp::handleReboot(AsyncWebServerRequest* request)
{
  request->send(200, "text/html; charset=utf-8",
                "<!DOCTYPE html><html><body><h1>Rebooting...</h1></body></html>");
  restartScheduled_ = true;
  restartAtMs_ = millis() + 500;
}

void WebOtaBlinkApp::handleSaveControl(AsyncWebServerRequest* request)
{
  bool updated = false;

  int32_t runCurrentMa = 0;
  if (parseIntParam(request, "tmc_run_current_ma", &runCurrentMa))
  {
    if (runCurrentMa >= 1 && runCurrentMa <= 2000)
    {
      tmc2209_set_run_current_ma((uint16_t)runCurrentMa);
      updated = true;
    }
  }

  int32_t holdCurrentPct = 0;
  if (parseIntParam(request, "tmc_hold_current_pct", &holdCurrentPct))
  {
    if (holdCurrentPct >= 0 && holdCurrentPct <= 100)
    {
      tmc2209_set_hold_current_pct((uint8_t)holdCurrentPct);
      updated = true;
    }
  }

  int32_t speedStepsPerSec = 0;
  if (parseIntParam(request, "move_max_speed_steps_per_sec", &speedStepsPerSec))
  {
    if (speedStepsPerSec > 0 && elevator_set_move_max_speed((float)speedStepsPerSec))
    {
      updated = true;
    }
  }

  int32_t accelStepsPerSec2 = 0;
  if (parseIntParam(request, "move_accel_steps_per_sec2", &accelStepsPerSec2))
  {
    if (accelStepsPerSec2 > 0 && elevator_set_move_acceleration((float)accelStepsPerSec2))
    {
      updated = true;
    }
  }

  int32_t zeroSteps = 0;
  if (parseIntParam(request, "btn_zero_steps", &zeroSteps))
  {
    button_position_store_set_zero(zeroSteps);
    updated = true;
  }

  int32_t ledGlobalBrightnessPct = 0;
  if (parseIntParam(request, "led_global_brightness_pct", &ledGlobalBrightnessPct))
  {
    if (ledGlobalBrightnessPct >= 0 && ledGlobalBrightnessPct <= 100)
    {
      if (led_set_global_brightness_pct((uint8_t)ledGlobalBrightnessPct))
      {
        updated = true;
      }
    }
  }

  for (int i = 0; i <= 9; ++i)
  {
    const String key = "btn_" + String(i) + "_relative_steps";
    int32_t relativeSteps = 0;
    if (parseIntParam(request, key, &relativeSteps))
    {
      if (button_position_store_record_relative((uint8_t)i, relativeSteps))
      {
        updated = true;
      }
    }
  }

  if (updated)
  {
    (void)tmc2209_save_current_settings();
    (void)elevator_save_motion_profile();
    (void)button_position_store_save();
    (void)led_save_global_brightness_pct();
  }

  request->redirect("/");
}

void WebOtaBlinkApp::handlePressButton(AsyncWebServerRequest* request)
{
  if (request == nullptr || !request->hasParam("btn", true))
  {
    request->send(400, "text/plain", "Missing btn");
    return;
  }

  const AsyncWebParameter* btnParam = request->getParam("btn", true);
  if (btnParam == nullptr)
  {
    request->send(400, "text/plain", "Invalid btn");
    return;
  }

  uint8_t buttonCode = 0;
  if (!parseRemoteButtonParam(btnParam->value(), &buttonCode))
  {
    request->send(400, "text/plain", "Unknown btn");
    return;
  }

  ir_inject_button((RemoteButton)buttonCode, 300);
  request->send(200, "text/plain; charset=utf-8", String("Sent: ") + btnParam->value());
}

void WebOtaBlinkApp::handleLedControl(AsyncWebServerRequest* request)
{
  if (request == nullptr) return;

  int32_t brightnessPct = -1;
  if (parseIntParam(request, "brightness_pct", &brightnessPct)) {
    if (brightnessPct >= 0 && brightnessPct <= 100 &&
        led_set_global_brightness_pct((uint8_t)brightnessPct)) {
      request->send(200, "text/plain; charset=utf-8", "brightness updated");
      return;
    }
    request->send(400, "text/plain; charset=utf-8", "brightness_pct must be 0..100");
    return;
  }

  int32_t pattern = -1;
  if (parseIntParam(request, "pattern", &pattern)) {
    if (pattern >= 0 && pattern <= 3) {
      led_set_pattern((LedPattern)pattern);
      request->send(200, "text/plain; charset=utf-8", "pattern updated");
      return;
    }
    request->send(400, "text/plain; charset=utf-8", "pattern must be 0..3");
    return;
  }

  if (!request->hasParam("strip", true)) {
    request->send(400, "text/plain; charset=utf-8", "missing strip");
    return;
  }
  const AsyncWebParameter* stripParam = request->getParam("strip", true);
  if (stripParam == nullptr) {
    request->send(400, "text/plain; charset=utf-8", "invalid strip");
    return;
  }

  const String stripValue = stripParam->value();
  const bool applyAllStrips = (stripValue == "ALL" || stripValue == "all");
  int32_t stripIndex = -1;
  if (!applyAllStrips) {
    if (!parseIntParam(request, "strip", &stripIndex)) {
      request->send(400, "text/plain; charset=utf-8", "strip must be 0..5 or ALL");
      return;
    }
    if (stripIndex < 0 || stripIndex >= SSC_LED_STRIP_COUNT) {
      request->send(400, "text/plain; charset=utf-8", "strip out of range");
      return;
    }
  }

  if (!request->hasParam("scene", true)) {
    request->send(400, "text/plain; charset=utf-8", "missing scene");
    return;
  }
  const AsyncWebParameter* sceneParam = request->getParam("scene", true);
  if (sceneParam == nullptr) {
    request->send(400, "text/plain; charset=utf-8", "invalid scene");
    return;
  }

  LedStripScene scene = LEDSCENE_SOLID;
  const String value = sceneParam->value();
  if (value == "SOLID") {
    scene = LEDSCENE_SOLID;
  } else if (value == "CHASE") {
    scene = LEDSCENE_CHASE;
  } else if (value == "BLINK") {
    scene = LEDSCENE_BLINK;
  } else if (value == "RANDOM") {
    scene = LEDSCENE_RANDOM_LONG_BLINK_THEN_ON;
  } else if (value == "CRASH") {
    scene = LEDSCENE_CRASH;
  } else if (value == "EMERGENCY") {
    scene = LEDSCENE_EMERGENCY_RED;
  } else if (value == "BLACKOUT") {
    scene = LEDSCENE_BLACKOUT;
  } else if (value == "FADEIN3S") {
    scene = LEDSCENE_FADE_IN_3S;
  } else if (value == "FADEOUT3S") {
    scene = LEDSCENE_FADE_OUT_3S;
  } else {
    request->send(400, "text/plain; charset=utf-8",
                  "scene must be SOLID/CHASE/BLINK/RANDOM/CRASH/EMERGENCY/BLACKOUT/FADEIN3S/FADEOUT3S");
    return;
  }

  if (applyAllStrips) {
    for (uint8_t strip = 0; strip < SSC_LED_STRIP_COUNT; ++strip) {
      led_set_strip_scene(strip, scene);
    }
    request->send(200, "text/plain; charset=utf-8", "all strips scene updated");
    return;
  }

  led_set_strip_scene((uint8_t)stripIndex, scene);
  request->send(200, "text/plain; charset=utf-8", "strip scene updated");
}

void WebOtaBlinkApp::handleSetToggle(AsyncWebServerRequest* request)
{
  if (request == nullptr || !request->hasParam("btn", true) || !request->hasParam("on", true))
  {
    request->send(400, "text/plain", "Missing btn/on");
    return;
  }

  const AsyncWebParameter* btnParam = request->getParam("btn", true);
  const AsyncWebParameter* onParam = request->getParam("on", true);
  if (btnParam == nullptr || onParam == nullptr)
  {
    request->send(400, "text/plain", "Invalid btn/on");
    return;
  }

  const bool enable = onParam->value() == "1";
  const String btnName = btnParam->value();

  if (btnName == "BTN_PREV")
  {
    webPrevToggleOn_ = enable;
    if (enable) webNextToggleOn_ = false;
  }
  else if (btnName == "BTN_NEXT")
  {
    webNextToggleOn_ = enable;
    if (enable) webPrevToggleOn_ = false;
  }
  else
  {
    request->send(400, "text/plain", "Toggle target must be BTN_PREV/BTN_NEXT");
    return;
  }

  if (webPrevToggleOn_ || webNextToggleOn_)
  {
    nextWebToggleInjectAtMs_ = millis();
  }

  String message = "Toggle ";
  message += btnName;
  message += enable ? " ON" : " OFF";
  request->send(200, "text/plain; charset=utf-8", message);
}

void WebOtaBlinkApp::handleOtaPage(AsyncWebServerRequest* request)
{
  String html;
  html += "<!DOCTYPE html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no'>";
  html += "<title>Web OTA</title></head><body style='font-family:Arial,sans-serif;max-width:720px;margin:24px auto;padding:0 16px;'>";
  html += "<h1>Firmware Update</h1>";
  html += "<form method='POST' action='/update' enctype='multipart/form-data'>";
  html += "<input type='file' name='firmware' accept='.bin' required>";
  html += "<button type='submit'>Upload</button>";
  html += "</form>";
  html += "</body></html>";
  request->send(200, "text/html; charset=utf-8", html);
}

void WebOtaBlinkApp::handleOtaUpload(AsyncWebServerRequest* request, const String& filename,
                                     size_t index, uint8_t* data, size_t len, bool final)
{
  (void)request;
  (void)filename;

  if (index == 0)
  {
    otaUploadOk_ = Update.begin(UPDATE_SIZE_UNKNOWN);
  }

  if (otaUploadOk_ && len > 0)
  {
    if (Update.write(data, len) != len)
    {
      otaUploadOk_ = false;
    }
  }

  if (final && otaUploadOk_)
  {
    otaUploadOk_ = Update.end(true);
  }
}

void WebOtaBlinkApp::handleOtaDone(AsyncWebServerRequest* request)
{
  String html;
  html += "<!DOCTYPE html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no'>";
  html += "<title>OTA Result</title></head><body style='font-family:Arial,sans-serif;max-width:720px;margin:24px auto;padding:0 16px;'>";

  if (otaUploadOk_)
  {
    html += "<h1>Update successful</h1><p>Rebooting...</p>";
    restartScheduled_ = true;
    restartAtMs_ = millis() + 1200;
  }
  else
  {
    html += "<h1>Update failed</h1><p>Check firmware binary and try again.</p>";
  }

  html += "</body></html>";
  request->send(200, "text/html; charset=utf-8", html);
}

void WebOtaBlinkApp::handleNotFound(AsyncWebServerRequest* request)
{
  request->send(404, "text/plain", "Not found");
}

// ------------------------------------------------------------
// html helpers
// ------------------------------------------------------------
String WebOtaBlinkApp::htmlEscape(const String& s) const
{
  String out;
  out.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); ++i)
  {
    const char c = s[i];
    switch (c)
    {
      case '&': out += "&amp;"; break;
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      case '"': out += "&quot;"; break;
      case '\'': out += "&#39;"; break;
      default: out += c; break;
    }
  }
  return out;
}

String WebOtaBlinkApp::currentModeText() const
{
  return apMode_ ? "AP mode" : "STA mode";
}

String WebOtaBlinkApp::currentIpText() const
{
  if (apMode_)
  {
    return WiFi.softAPIP().toString();
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    return WiFi.localIP().toString();
  }
  return "-";
}

String WebOtaBlinkApp::customMacText() const
{
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           customStaMac_[0], customStaMac_[1], customStaMac_[2],
           customStaMac_[3], customStaMac_[4], customStaMac_[5]);
  return String(buf);
}

bool WebOtaBlinkApp::parseIntParam(AsyncWebServerRequest* request, const String& key, int32_t* out_value) const
{
  if (request == nullptr || out_value == nullptr) return false;
  if (!request->hasParam(key, true)) return false;

  const AsyncWebParameter* p = request->getParam(key, true);
  if (p == nullptr) return false;

  const String v = p->value();
  if (v.length() == 0) return false;

  char* endp = nullptr;
  const long parsed = strtol(v.c_str(), &endp, 10);
  if (endp == v.c_str() || *endp != '\0') return false;

  *out_value = (int32_t)parsed;
  return true;
}

bool WebOtaBlinkApp::parseRemoteButtonParam(const String& key, uint8_t* out_button_code) const
{
  if (out_button_code == nullptr) return false;

  struct BtnMap
  {
    const char* name;
    RemoteButton btn;
  };

  static const BtnMap kBtnMap[] = {
    {"BTN_POWER", BTN_POWER},
    {"BTN_MODE", BTN_MODE},
    {"BTN_MUTE", BTN_MUTE},
    {"BTN_PLAYPAUSE", BTN_PLAYPAUSE},
    {"BTN_PREV", BTN_PREV},
    {"BTN_NEXT", BTN_NEXT},
    {"BTN_EQ", BTN_EQ},
    {"BTN_VOL_DOWN", BTN_VOL_DOWN},
    {"BTN_VOL_UP", BTN_VOL_UP},
    {"BTN_0", BTN_0},
    {"BTN_RPT", BTN_RPT},
    {"BTN_CLOCK", BTN_CLOCK},
    {"BTN_1", BTN_1},
    {"BTN_2", BTN_2},
    {"BTN_3", BTN_3},
    {"BTN_4", BTN_4},
    {"BTN_5", BTN_5},
    {"BTN_6", BTN_6},
    {"BTN_7", BTN_7},
    {"BTN_8", BTN_8},
    {"BTN_9", BTN_9},
  };

  for (const auto& entry : kBtnMap)
  {
    if (key == entry.name)
    {
      *out_button_code = (uint8_t)entry.btn;
      return true;
    }
  }

  return false;
}

String WebOtaBlinkApp::renderControllerSettingsSection() const
{
  String section = kControllerSettingsSectionTemplate;
  section.replace("{{TMC_RUN_CURRENT_MA}}", String(tmc2209_run_current_ma()));
  section.replace("{{TMC_HOLD_CURRENT_PCT}}", String(tmc2209_hold_current_pct()));
  section.replace("{{MOVE_MAX_SPEED_STEPS_PER_SEC}}", String((int32_t)elevator_move_max_speed()));
  section.replace("{{MOVE_ACCEL_STEPS_PER_SEC2}}", String((int32_t)elevator_move_acceleration()));
  section.replace("{{BTN_ZERO_STEPS}}", String(button_position_store_zero_steps()));
  section.replace("{{LED_GLOBAL_BRIGHTNESS_PCT}}", String(led_global_brightness_pct()));

  String relativeRows;
  for (int i = 0; i <= 9; ++i)
  {
    int32_t relativeSteps = 0;
    const bool hasValue = button_position_store_relative((uint8_t)i, &relativeSteps);
    relativeRows += "<label>BTN_";
    relativeRows += String(i);
    relativeRows += " relative steps</label>";
    relativeRows += "<input name='btn_";
    relativeRows += String(i);
    relativeRows += "_relative_steps' type='number' value='";
    relativeRows += hasValue ? String(relativeSteps) : "";
    relativeRows += "' placeholder='Not set'>";
  }

  section.replace("{{BTN_RELATIVE_ROWS}}", relativeRows);
  return section;
}

String WebOtaBlinkApp::makeHtml() const
{
  auto isToggleKey = [](const char* key) -> bool
  {
    if (key == nullptr) return false;
    return strcmp(key, "BTN_PREV") == 0 || strcmp(key, "BTN_NEXT") == 0;
  };


  auto redFromWebColor = [](uint32_t webColor) -> uint8_t
  {
    return (uint8_t)((webColor >> 16) & 0xFF);
  };
  auto greenFromWebColor = [](uint32_t webColor) -> uint8_t
  {
    return (uint8_t)((webColor >> 8) & 0xFF);
  };
  auto blueFromWebColor = [](uint32_t webColor) -> uint8_t
  {
    return (uint8_t)(webColor & 0xFF);
  };
  auto webColorCssText = [](uint32_t webColor) -> String
  {
    char buf[8] = {0};
    snprintf(buf, sizeof(buf), "#%06lX", (unsigned long)(webColor & 0xFFFFFFUL));
    return String(buf);
  };
  auto textColorForBg = [&](uint32_t webColor) -> const char*
  {
    const uint8_t r = redFromWebColor(webColor);
    const uint8_t g = greenFromWebColor(webColor);
    const uint8_t b = blueFromWebColor(webColor);
    return (r < 128 || g < 128 || b < 128) ? "rgb(255,255,255)" : "rgb(48,48,48)";
  };

  String html;
  html += "<!DOCTYPE html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no'>";
  html += "<title>ESP32 Setup</title>";
  html += "<style>";
  html += "body{font-family:Arial,sans-serif;max-width:820px;margin:24px auto;padding:0 16px;line-height:1.5;touch-action:manipulation;}";
  html += ".box{border:1px solid #ccc;border-radius:10px;padding:16px;margin-bottom:16px;}";
  html += "input{width:100%;padding:10px;font-size:16px;box-sizing:border-box;margin-top:4px;}";
  html += "button,a.btn{display:inline-block;padding:10px 14px;margin-top:12px;text-decoration:none;border:1px solid #333;border-radius:8px;background:#f5f5f5;color:#000;}";
  html += ".mono{font-family:monospace;}";
  html += ".grid{display:grid;grid-template-columns:160px 1fr;gap:8px 12px;align-items:center;}";
  html += ".remote-grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px;}";
  html += ".remote-grid button{width:100%;margin-top:0;padding:20px 14px;}";
  html += ".remote-grid .empty{visibility:hidden;}";
  html += ".toggle-on{filter:brightness(0.55);}";
  html += "#remote-status{min-height:1.4em;font-weight:bold;}";
  html += ".small{font-size:0.9em;color:#555;}";
  html += "</style></head><body>";

  html += "<h1>ESP32 Web Setup</h1>";

  html += "<div class='box'><h2>Remote Buttons</h2>";
  html += "<div id='remote-status' class='small'>Ready</div>";
  html += "<div class='remote-grid'>";

  for (size_t i = 0; i < SSC_WEB_REMOTE_BUTTON_COUNT; ++i)
  {
    const char* key = SSC_WEB_REMOTE_BUTTON_KEYS[i];
    const char* label = SSC_WEB_REMOTE_BUTTON_LABELS[i];
    const uint32_t webColor = SSC_WEB_REMOTE_BUTTON_COLORS[i].webColor;
    const bool toggleButton = isToggleKey(key);

    html += "<button type='button'";
    if (toggleButton)
    {
      if (strcmp(key, "BTN_PREV") == 0)
      {
        html += " id='toggle-prev' class='";
        html += webPrevToggleOn_ ? "toggle-on" : "";
        html += "'";
      }
      else if (strcmp(key, "BTN_NEXT") == 0)
      {
        html += " id='toggle-next' class='";
        html += webNextToggleOn_ ? "toggle-on" : "";
        html += "'";
      }
    }

    html += " style='background:";
    html += webColorCssText(webColor);
    html += ";color:";
    html += textColorForBg(webColor);
    html += ";'";

    html += " onclick=\"";
    html += toggleButton ? "toggleBtn('" : "sendBtn('";
    html += key;
    html += toggleButton ? "', this)\"" : "')\"";
    html += ">";
    html += htmlEscape(label ? label : key);
    html += "</button>";
  }

  html += "</div>";
  html += "<p class='small'>PREV/NEXTはトグル動作です。ON中は継続送信してエレベーターを上下動させます。</p>";
  html += "</div>";

  html += "<div class='box'><h2>Status</h2>";
  html += "<div class='grid'>";
  html += "<div>Mode</div><div>" + currentModeText() + "</div>";

  html += "<div>Connected SSID</div><div>";
  html += (WiFi.status() == WL_CONNECTED) ? htmlEscape(WiFi.SSID()) : "-";
  html += "</div>";

  html += "<div>IP</div><div class='mono'>";
  html += currentIpText();
  html += "</div>";

  html += "<div>STA MAC</div><div class='mono'>";
  html += customMacText();
  html += "</div>";
  html += "</div></div>";

  html += "<div class='box'><h2>Wi-Fi Candidates</h2>";
  html += "<form method='POST' action='/save-wifi'>";

  for (int i = 0; i < kMaxWifiSlots; ++i)
  {
    html += "<h3>Slot ";
    html += String(i + 1);
    html += "</h3>";

    html += "<label>SSID</label>";
    html += "<input name='ssid";
    html += String(i);
    html += "' value='";
    html += htmlEscape(wifiSlots_[i].ssid);
    html += "'>";

    html += "<label>Password</label>";
    html += "<input type='password' name='pass";
    html += String(i);
    html += "' value='";
    html += htmlEscape(wifiSlots_[i].pass);
    html += "'>";
  }

  html += "<p class='small'>保存後に再起動し、各SSIDを3回ずつ順番に試します。DHCPで接続します。</p>";
  html += "<button type='submit'>Save Wi-Fi Settings</button>";
  html += "</form></div>";

  html += "<div class='box'><h2>OTA</h2>";
  html += "<a class='btn' href='/update'>Open Web OTA</a>";
  html += "</div>";

  html += renderControllerSettingsSection();

  html += "<div class='box'><h2>LED Scene Control</h2>";
  html += "<div id='led-status' class='small'>Ready</div>";
  html += "<div class='grid'>";
  html += "<div>Scene pattern</div><div>";
  html += "<button type='button' onclick='setLedPattern(0)'>IDLE</button> ";
  html += "<button type='button' onclick='setLedPattern(1)'>MOVING</button> ";
  html += "<button type='button' onclick='setLedPattern(2)'>ARRIVED</button> ";
  html += "<button type='button' onclick='setLedPattern(3)'>ERROR</button>";
  html += "</div>";
  html += "<div>Strip scene (ALL)</div><div>";
  html += "<button type='button' onclick=\"setStripSceneAll('SOLID')\">SOLID</button> ";
  html += "<button type='button' onclick=\"setStripSceneAll('CHASE')\">CHASE</button> ";
  html += "<button type='button' onclick=\"setStripSceneAll('BLINK')\">BLINK</button> ";
  html += "<button type='button' onclick=\"setStripSceneAll('RANDOM')\">RANDOM</button> ";
  html += "<button type='button' onclick=\"setStripSceneAll('CRASH')\">CRASH</button> ";
  html += "<button type='button' onclick=\"setStripSceneAll('EMERGENCY')\">EMERGENCY</button> ";
  html += "<button type='button' onclick=\"setStripSceneAll('BLACKOUT')\">BLACKOUT</button> ";
  html += "<button type='button' onclick=\"setStripSceneAll('FADEIN3S')\">FADEIN3S</button> ";
  html += "<button type='button' onclick=\"setStripSceneAll('FADEOUT3S')\">FADEOUT3S</button>";
  html += "</div>";
  html += "</div>";
  html += "<div style='margin-top:10px;display:flex;gap:8px;align-items:center;flex-wrap:wrap;'>";
  html += "<label for='led-strip'>Strip</label>";
  html += "<select id='led-strip'>";
  html += "<option value='ALL'>ALL</option>";
  for (uint8_t strip = 0; strip < SSC_LED_STRIP_COUNT; ++strip) {
    html += "<option value='";
    html += String(strip);
    html += "'>";
    html += String(strip);
    html += "</option>";
  }
  html += "</select>";
  html += "<label for='led-scene'>Scene</label>";
  html += "<select id='led-scene'>";
  html += "<option value='SOLID'>SOLID</option>";
  html += "<option value='CHASE'>CHASE</option>";
  html += "<option value='BLINK'>BLINK</option>";
  html += "<option value='RANDOM'>RANDOM</option>";
  html += "<option value='CRASH'>CRASH</option>";
  html += "<option value='EMERGENCY'>EMERGENCY</option>";
  html += "<option value='BLACKOUT'>BLACKOUT</option>";
  html += "<option value='FADEIN3S'>FADEIN3S</option>";
  html += "<option value='FADEOUT3S'>FADEOUT3S</option>";
  html += "</select>";
  html += "<button type='button' onclick='setStripScene()'>Apply strip scene</button>";
  html += "</div>";
  html += "<p class='small'>Serial command: LEDSCENE &lt;0..5|ALL&gt; &lt;SOLID|CHASE|BLINK|RANDOM|CRASH|EMERGENCY|BLACKOUT|FADEIN3S|FADEOUT3S&gt;</p>";
  html += "<p class='small'>Serial brightness command: brightness_&lt;0..100&gt;</p>";
  html += "</div>";

  html += "<div class='box'><h2>Screen Awake (Experimental)</h2>";
  html += "<div id='wake-status' class='small'>Checking support...</div>";
  html += "<button id='wake-toggle' type='button' onclick='toggleWakeLock()' disabled>Keep screen awake</button>";
  html += "<p class='small'>対応ブラウザではこのページ表示中の画面スリープを抑止します。タブ非表示時やOS状態によって解除される場合があります。</p>";
  html += "</div>";

  html += "<div class='box'><h2>Actions</h2>";
  html += "<a class='btn' href='/reboot'>Reboot</a>";
  html += "</div>";

  if (apMode_)
  {
    html += "<div class='box'><h2>AP mode</h2>";
    html += "<p>Connect to <b>";
    html += htmlEscape(fallbackApSsid_);
    html += "</b> and open <span class='mono'>http://192.168.4.1/</span></p>";
    html += "</div>";
  }

  html += "<script>";
  html += "function setStatus(msg,isErr){const s=document.getElementById('remote-status');if(!s)return;s.textContent=msg;s.style.color=isErr?'#b00020':'#0b6b2f';}";
  html += "function setLedStatus(msg,isErr){const s=document.getElementById('led-status');if(!s)return;s.textContent=msg;s.style.color=isErr?'#b00020':'#0b6b2f';}";
  html += "function setWakeStatus(msg,isErr){const s=document.getElementById('wake-status');if(!s)return;s.textContent=msg;s.style.color=isErr?'#b00020':'#0b6b2f';}";
  html += "function postForm(url,data){return fetch(url,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded;charset=UTF-8'},body:new URLSearchParams(data)});}";
  html += "function syncToggleUi(active){const p=document.getElementById('toggle-prev');const n=document.getElementById('toggle-next');if(p)p.classList.toggle('toggle-on',active==='BTN_PREV');if(n)n.classList.toggle('toggle-on',active==='BTN_NEXT');}";
  html += "let wakeLockSentinel=null;";
  html += "let wakeLockRequested=false;";
  html += "function isWakeLockSupported(){return typeof navigator!=='undefined'&&'wakeLock' in navigator;}";
  html += "function updateWakeToggleUi(){const btn=document.getElementById('wake-toggle');if(!btn)return;const active=!!wakeLockSentinel;btn.disabled=!isWakeLockSupported();btn.textContent=active?'Allow screen sleep':'Keep screen awake';}";
  html += "async function requestWakeLock(){if(!isWakeLockSupported()){setWakeStatus('Wake Lock API is not supported on this browser.',true);updateWakeToggleUi();return false;}try{wakeLockSentinel=await navigator.wakeLock.request('screen');wakeLockSentinel.addEventListener('release',()=>{wakeLockSentinel=null;updateWakeToggleUi();if(wakeLockRequested&&document.visibilityState==='visible'){setWakeStatus('Wake lock was released. Retrying...',true);requestWakeLock();return;}setWakeStatus('Screen may sleep now.',false);});setWakeStatus('Screen wake lock is active.',false);updateWakeToggleUi();return true;}catch(e){wakeLockSentinel=null;setWakeStatus('Wake lock failed: '+e,true);updateWakeToggleUi();return false;}}";
  html += "async function releaseWakeLock(){wakeLockRequested=false;if(wakeLockSentinel){try{await wakeLockSentinel.release();}catch(e){setWakeStatus('Release failed: '+e,true);return;}wakeLockSentinel=null;}setWakeStatus('Screen may sleep now.',false);updateWakeToggleUi();}";
  html += "async function toggleWakeLock(){if(wakeLockSentinel){await releaseWakeLock();return;}wakeLockRequested=true;await requestWakeLock();}";
  html += "document.addEventListener('visibilitychange',async()=>{if(document.visibilityState!=='visible')return;if(wakeLockRequested&&!wakeLockSentinel){await requestWakeLock();}});";
  html += "function initWakeLockUi(){if(!isWakeLockSupported()){setWakeStatus('Wake Lock API is not supported on this browser. 画面を常時表示したい場合は端末設定の自動ロック時間を調整してください。',true);updateWakeToggleUi();return;}setWakeStatus('Wake lock available. Tap the button to keep screen awake.',false);updateWakeToggleUi();}";
  html += "async function sendBtn(btn){try{const r=await postForm('/press-btn',{btn:btn});const t=await r.text();setStatus((r.ok?t:('Send failed: '+t)),!r.ok);}catch(e){setStatus('Send failed: '+e,true);}}";
  html += "async function toggleBtn(btn,el){const on=!el.classList.contains('toggle-on');try{const r=await postForm('/set-toggle',{btn:btn,on:on?'1':'0'});const t=await r.text();if(r.ok){syncToggleUi(on?btn:'');setStatus(t,false);}else{setStatus('Toggle failed: '+t,true);}}catch(e){setStatus('Toggle failed: '+e,true);}}";
  html += "async function setLedPattern(pattern){try{const r=await postForm('/led-control',{pattern:String(pattern)});const t=await r.text();setLedStatus((r.ok?('Pattern set: '+pattern):('Pattern failed: '+t)),!r.ok);}catch(e){setLedStatus('Pattern failed: '+e,true);}}";
  html += "async function setStripSceneAll(scene){try{const r=await postForm('/led-control',{strip:'ALL',scene:scene});const t=await r.text();setLedStatus((r.ok?('All strips -> '+scene):('Scene failed: '+t)),!r.ok);}catch(e){setLedStatus('Scene failed: '+e,true);}}";
  html += "async function setStripScene(){const strip=document.getElementById('led-strip').value;const scene=document.getElementById('led-scene').value;try{const r=await postForm('/led-control',{strip:strip,scene:scene});const t=await r.text();setLedStatus((r.ok?('Strip '+strip+' -> '+scene):('Scene failed: '+t)),!r.ok);}catch(e){setLedStatus('Scene failed: '+e,true);}}";
  html += "let ledBrightnessApplyTimer=0;";
  html += "let ledBrightnessReqSeq=0;";
  html += "let ledBrightnessAckSeq=0;";
  html += "async function applyRealtimeBrightness(v){const seq=++ledBrightnessReqSeq;try{const r=await postForm('/led-control',{brightness_pct:String(v)});const t=await r.text();if(seq<ledBrightnessAckSeq)return;ledBrightnessAckSeq=seq;setLedStatus((r.ok?('Brightness: '+v+'%'):('Brightness failed: '+t)),!r.ok);}catch(e){if(seq<ledBrightnessAckSeq)return;ledBrightnessAckSeq=seq;setLedStatus('Brightness failed: '+e,true);}}";
  html += "function queueRealtimeBrightness(v){if(ledBrightnessApplyTimer)clearTimeout(ledBrightnessApplyTimer);ledBrightnessApplyTimer=setTimeout(()=>applyRealtimeBrightness(v),80);}";
  html += "function initBrightnessControl(){const num=document.getElementById('led-brightness-input');const slider=document.getElementById('led-brightness-slider');if(!num||!slider)return;const clamp=(raw)=>{const n=parseInt(raw,10);if(Number.isNaN(n))return 0;return Math.max(0,Math.min(100,n));};const syncFrom=(src,dst)=>{const v=clamp(src.value);src.value=String(v);dst.value=String(v);queueRealtimeBrightness(v);};num.addEventListener('input',()=>syncFrom(num,slider));slider.addEventListener('input',()=>syncFrom(slider,num));}";
  html += "syncToggleUi('";
  if (webPrevToggleOn_)
  {
    html += "BTN_PREV";
  }
  else if (webNextToggleOn_)
  {
    html += "BTN_NEXT";
  }
  html += "');";
  html += "initWakeLockUi();";
  html += "initBrightnessControl();";
  html += "</script>";

  html += "</body></html>";
  return html;
}
