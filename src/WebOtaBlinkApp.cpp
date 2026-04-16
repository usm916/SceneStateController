#include "WebOtaBlinkApp.h"

#include <Update.h>
#include <esp_wifi.h>
#include <stdlib.h>

#include "button_position_store.h"
#include "elevator_module.h"
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
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
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
  }

  request->redirect("/");
}

void WebOtaBlinkApp::handleOtaPage(AsyncWebServerRequest* request)
{
  String html;
  html += "<!DOCTYPE html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
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
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
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

String WebOtaBlinkApp::makeHtml() const
{
  String html;
  html += "<!DOCTYPE html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>ESP32 Setup</title>";
  html += "<style>";
  html += "body{font-family:Arial,sans-serif;max-width:820px;margin:24px auto;padding:0 16px;line-height:1.5;}";
  html += ".box{border:1px solid #ccc;border-radius:10px;padding:16px;margin-bottom:16px;}";
  html += "input{width:100%;padding:10px;font-size:16px;box-sizing:border-box;margin-top:4px;}";
  html += "button,a.btn{display:inline-block;padding:10px 14px;margin-top:12px;text-decoration:none;border:1px solid #333;border-radius:8px;background:#f5f5f5;color:#000;}";
  html += ".mono{font-family:monospace;}";
  html += ".grid{display:grid;grid-template-columns:160px 1fr;gap:8px 12px;align-items:center;}";
  html += ".small{font-size:0.9em;color:#555;}";
  html += "</style></head><body>";

  html += "<h1>ESP32 Web Setup</h1>";

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

  html += "<div class='box'><h2>Controller Settings</h2>";
  html += "<form method='POST' action='/save-control'>";
  html += "<label>TMC run current (mA)</label>";
  html += "<input name='tmc_run_current_ma' type='number' min='1' max='2000' value='";
  html += String(tmc2209_run_current_ma());
  html += "'>";
  html += "<label>TMC hold current (%)</label>";
  html += "<input name='tmc_hold_current_pct' type='number' min='0' max='100' value='";
  html += String(tmc2209_hold_current_pct());
  html += "'>";
  html += "<label>Move max speed (steps/sec)</label>";
  html += "<input name='move_max_speed_steps_per_sec' type='number' min='1' value='";
  html += String((int32_t)elevator_move_max_speed());
  html += "'>";
  html += "<label>Move acceleration (steps/sec²)</label>";
  html += "<input name='move_accel_steps_per_sec2' type='number' min='1' value='";
  html += String((int32_t)elevator_move_acceleration());
  html += "'>";
  html += "<label>Button zero steps (mute equivalent)</label>";
  html += "<input name='btn_zero_steps' type='number' value='";
  html += String(button_position_store_zero_steps());
  html += "'>";

  for (int i = 0; i <= 9; ++i)
  {
    int32_t relativeSteps = 0;
    const bool hasValue = button_position_store_relative((uint8_t)i, &relativeSteps);
    html += "<label>BTN_";
    html += String(i);
    html += " relative steps</label>";
    html += "<input name='btn_";
    html += String(i);
    html += "_relative_steps' type='number' value='";
    html += hasValue ? String(relativeSteps) : "";
    html += "' placeholder='Not set'>";
  }

  html += "<p class='small'>INFOやserial/IRで更新可能な値をここから書き換えできます。保存時にNVSへ永続化します。</p>";
  html += "<button type='submit'>Save Controller Settings</button>";
  html += "</form></div>";

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

  html += "</body></html>";
  return html;
}
