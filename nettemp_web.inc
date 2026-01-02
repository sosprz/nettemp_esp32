#if NETTEMP_ENABLE_BLE
// Forward declaration for BLE scan trigger (defined in .ino)
static void bleTriggerManualScan();
#endif

static String dsRomLinuxId(const std::array<uint8_t, 8>& rom, uint8_t bus, bool includeBus) {
  char out[32]{};
  char serial[13]{};
  // Linux w1 serial is bytes 1..6 reversed (no CRC).
  snprintf(serial, sizeof(serial), "%02X%02X%02X%02X%02X%02X",
           rom[6], rom[5], rom[4], rom[3], rom[2], rom[1]);
  if (includeBus) {
    snprintf(out, sizeof(out), "%02x-ds_%02X_%s", (unsigned)bus, (unsigned)rom[0], serial);
  } else {
    snprintf(out, sizeof(out), "ds_%02X_%s", (unsigned)rom[0], serial);
  }
  String s(out);
  s.toLowerCase();
  return s;
}

#if NETTEMP_ENABLE_PORTAL
enum class WebMode {
  None,
  ApSetup,
  StaConfig,
};

WebMode g_webMode = WebMode::None;

static String portalSsid() {
  String ssid = "nettemp-setup-";
  ssid += String((uint32_t)(ESP.getEfuseMac() & 0xFFFFFFFFu), HEX);
  ssid.toLowerCase();
  return ssid;
}

static String htmlEscape(const String& s) {
  String out;
  out.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    const char c = s[i];
    if (c == '&') out += "&amp;";
    else if (c == '<') out += "&lt;";
    else if (c == '>') out += "&gt;";
    else if (c == '"') out += "&quot;";
    else out += c;
  }
  return out;
}

static void appendCheckbox(String& body, const String& name, bool checked, const char* label) {
  body += "<label><input type='checkbox' name='";
  body += name;
  body += "' value='1' ";
  body += checked ? "checked" : "";
  body += "> ";
  body += label;
  body += "</label>";
}

static void appendBleFieldCheckboxes(String& body, const char* prefix, uint32_t mask) {
  appendCheckbox(body, String(prefix) + "tempc", (mask & SRV_BLE_TEMPC) != 0, "tempc");
  appendCheckbox(body, String(prefix) + "tempf", (mask & SRV_BLE_TEMPF) != 0, "tempf");
  appendCheckbox(body, String(prefix) + "hum", (mask & SRV_BLE_HUM) != 0, "hum");
  appendCheckbox(body, String(prefix) + "batt", (mask & SRV_BLE_BATT) != 0, "batt");
  appendCheckbox(body, String(prefix) + "volt", (mask & SRV_BLE_VOLT) != 0, "volt");
  appendCheckbox(body, String(prefix) + "rssi", (mask & SRV_BLE_RSSI) != 0, "rssi");
}

#if NETTEMP_ENABLE_I2C
static void appendI2cFieldCheckboxes(String& body, const char* prefix, uint32_t mask) {
  appendCheckbox(body, String(prefix) + "tempc", (mask & SRV_I2C_TEMPC) != 0, "tempc");
  appendCheckbox(body, String(prefix) + "hum", (mask & SRV_I2C_HUM) != 0, "hum");
  appendCheckbox(body, String(prefix) + "press", (mask & SRV_I2C_PRESS) != 0, "press_hpa");
  appendCheckbox(body, String(prefix) + "lux", (mask & SRV_I2C_LUX) != 0, "lux");
  appendCheckbox(body, String(prefix) + "dist", (mask & SRV_I2C_DIST) != 0, "dist");
}
#endif

static void appendGpioFieldCheckboxes(String& body, const char* prefix, uint32_t mask) {
  appendCheckbox(body, String(prefix) + "tempc", (mask & SRV_GPIO_TEMPC) != 0, "tempc");
  appendCheckbox(body, String(prefix) + "tempf", (mask & SRV_GPIO_TEMPF) != 0, "tempf");
  appendCheckbox(body, String(prefix) + "hum", (mask & SRV_GPIO_HUM) != 0, "hum");
  appendCheckbox(body, String(prefix) + "batt", (mask & SRV_GPIO_BATT) != 0, "batt");
  appendCheckbox(body, String(prefix) + "volt", (mask & SRV_GPIO_VOLT) != 0, "volt");
  appendCheckbox(body, String(prefix) + "soil_raw", (mask & SRV_GPIO_SOIL_RAW) != 0, "soil_raw");
  appendCheckbox(body, String(prefix) + "soil_pct", (mask & SRV_GPIO_SOIL_PCT) != 0, "soil_pct");
  appendCheckbox(body, String(prefix) + "dist", (mask & SRV_GPIO_DIST) != 0, "dist");
}

static uint32_t parseBleMask(const char* prefix, uint32_t defaultMask) {
  (void)defaultMask;  // Unused - user has full control via checkboxes
  uint32_t mask = 0;
  if (g_webServer.hasArg(String(prefix) + "tempc")) mask |= SRV_BLE_TEMPC;
  if (g_webServer.hasArg(String(prefix) + "tempf")) mask |= SRV_BLE_TEMPF;
  if (g_webServer.hasArg(String(prefix) + "hum")) mask |= SRV_BLE_HUM;
  if (g_webServer.hasArg(String(prefix) + "batt")) mask |= SRV_BLE_BATT;
  if (g_webServer.hasArg(String(prefix) + "volt")) mask |= SRV_BLE_VOLT;
  if (g_webServer.hasArg(String(prefix) + "rssi")) mask |= SRV_BLE_RSSI;
  return mask;
}

#if NETTEMP_ENABLE_I2C
static uint32_t parseI2cMask(const char* prefix, uint32_t defaultMask) {
  (void)defaultMask;  // Unused - user has full control via checkboxes
  uint32_t mask = 0;
  if (g_webServer.hasArg(String(prefix) + "tempc")) mask |= SRV_I2C_TEMPC;
  if (g_webServer.hasArg(String(prefix) + "hum")) mask |= SRV_I2C_HUM;
  if (g_webServer.hasArg(String(prefix) + "press")) mask |= SRV_I2C_PRESS;
  if (g_webServer.hasArg(String(prefix) + "lux")) mask |= SRV_I2C_LUX;
  if (g_webServer.hasArg(String(prefix) + "dist")) mask |= SRV_I2C_DIST;
  return mask;
}
#endif

static uint32_t parseGpioMask(const char* prefix, uint32_t defaultMask) {
  (void)defaultMask;  // Unused - user has full control via checkboxes
  uint32_t mask = 0;
  if (g_webServer.hasArg(String(prefix) + "tempc")) mask |= SRV_GPIO_TEMPC;
  if (g_webServer.hasArg(String(prefix) + "tempf")) mask |= SRV_GPIO_TEMPF;
  if (g_webServer.hasArg(String(prefix) + "hum")) mask |= SRV_GPIO_HUM;
  if (g_webServer.hasArg(String(prefix) + "batt")) mask |= SRV_GPIO_BATT;
  if (g_webServer.hasArg(String(prefix) + "volt")) mask |= SRV_GPIO_VOLT;
  if (g_webServer.hasArg(String(prefix) + "soil_raw")) mask |= SRV_GPIO_SOIL_RAW;
  if (g_webServer.hasArg(String(prefix) + "soil_pct")) mask |= SRV_GPIO_SOIL_PCT;
  if (g_webServer.hasArg(String(prefix) + "dist")) mask |= SRV_GPIO_DIST;
  return mask;
}

static bool webRequireAuth();
static void bleConfigureScan();

static void resetMqttSendState() {
  for (auto& s : g_sensors) s.lastMqttSentMs = 0;
#if NETTEMP_ENABLE_I2C
  for (auto& s : g_i2cSensors) s.last_mqtt_sent_ms = 0;
#endif
}

// REMOVED: Old blocking handleSendNow - replaced with async handlers below

static String formatSendStatus(uint32_t lastSendMs, uint32_t intervalMs) {
  if (lastSendMs == 0) return String("Sent never");
  const uint32_t secAgo = (millis() - lastSendMs) / 1000UL;
  if (intervalMs == 0) return String("Sent ") + String(secAgo) + " sec ago";
  const uint32_t intervalS = intervalMs / 1000UL;
  if (intervalS == 0) return String("Sent ") + String(secAgo) + " sec ago";
  if (secAgo < intervalS) {
    const uint32_t nextIn = intervalS - secAgo;
    return String("Sent ") + String(secAgo) + " sec ago Next in " + String(nextIn) + " sec";
  }
  return String("Sent ") + String(secAgo) + " sec ago Ready";
}

static uint32_t parseU32Or(const String& s, uint32_t fallback) {
  if (!s.length()) return fallback;
  long v = s.toInt();
  if (v < 0) return fallback;
  return (uint32_t)v;
}

static void portalStop() {
  if (!g_portalRunning) return;
  g_dnsServer.stop();
  g_webServer.stop();
  WiFi.softAPdisconnect(true);
  g_portalRunning = false;
  g_webMode = WebMode::None;
}

static bool webRequireAuth() {
  if (g_webMode != WebMode::StaConfig) return true;
  if (g_cfg.webUser.length() == 0 || g_cfg.webPass.length() == 0) {
    g_webServer.send(500, "text/plain", "Web auth not configured");
    return false;
  }
  if (g_webServer.authenticate(g_cfg.webUser.c_str(), g_cfg.webPass.c_str())) return true;
  g_webServer.requestAuthentication();
  return false;
}

static void portalHandleRoot() {
  // AP captive portal: WiFi + admin credentials only
  const String ssid = htmlEscape(g_cfg.wifiSsid);
  const String body =
    String("<!doctype html><html><head><meta charset='utf-8'/>"
           "<meta name='viewport' content='width=device-width,initial-scale=1'/>"
           "<title>Nettemp Setup (WiFi)</title>"
           "<style>"
           "*,*:before,*:after{box-sizing:border-box}"
           ":root{--bg:#0b1220;--card:#111b2e;--muted:#9fb0cc;--text:#e8eefc;--accent:#2f6bff;--border:#243251}"
           "body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial;margin:0;background:var(--bg);color:var(--text)}"
           ".wrap{max-width:860px;margin:0 auto;padding:16px}"
           ".top{display:flex;align-items:center;gap:10px;margin:6px 0 14px}"
           "h2{margin:0;font-size:20px}"
           "p{color:var(--muted);line-height:1.35}"
           ".card{background:var(--card);border:1px solid var(--border);border-radius:14px;padding:14px;margin:12px 0}"
           "label{display:block;margin:10px 0 6px;color:var(--muted);font-size:13px}"
           "input{max-width:100%;padding:8px 10px;border-radius:10px;border:1px solid var(--border);background:#0d1628;color:var(--text)}"
           "input[type=checkbox]{width:auto}"
           ".row{display:flex;gap:10px} .row>div{flex:1}"
           "code{background:#0d1628;border:1px solid var(--border);padding:2px 6px;border-radius:8px;color:var(--text)}"
           "button{background:var(--accent);border:none;color:white;padding:10px 14px;border-radius:10px;font-weight:600}"
           "</style>"
           "</head><body>"
           "<div class='wrap'>"
           "<div class='top'><div>"
           "<h2>Nettemp Setup</h2><p>Step 1/2: WiFi + Web Admin (Basic Auth)</p>"
           "</div></div>"
           "<div class='card'>"
           "<form method='POST' action='/save'>"
           "<h3 style='margin:0 0 10px'>WiFi</h3>"
           "<label>SSID</label><input name='wifiSsid' value='") + ssid + "'/>"
           "<label>Password</label><input name='wifiPass' type='password' value='' placeholder='(leave empty to keep existing)'/>"
           "<h3 style='margin:16px 0 10px'>Web Admin</h3>"
           "<label>Username</label><input name='webUser' value='" + htmlEscape(g_cfg.webUser) + "'/>"
           "<label>Password</label><input name='webPass' type='password' value='' placeholder='(required for web config server)'/>"
           "<label style='margin-top:14px'><input type='checkbox' name='apAuto' value='1' " + String(g_portalAuto ? "checked" : "") + "> Auto-start portal if WiFi not configured</label>"
           "<p style='margin:14px 0 0'><button type='submit'>Save & Reboot</button></p>"
           "</form>"
           "</div>"
           "<p>Portal SSID: <code>" + htmlEscape(portalSsid()) + "</code> (open network)</p>"
           "<p>Step 2/2: after WiFi connects, open <code>http://&lt;device-ip&gt;/</code> to configure MQTT/Server (Basic Auth).</p>"
           "</div></body></html>";
  g_webServer.send(200, "text/html; charset=utf-8", body);
}

static void portalHandleSave() {
  const String wifiSsid = g_webServer.arg("wifiSsid");
  const String wifiPass = g_webServer.arg("wifiPass");
  if (wifiSsid.length()) g_cfg.wifiSsid = wifiSsid;
  if (wifiPass.length()) g_cfg.wifiPass = wifiPass;

  const String webUser = g_webServer.arg("webUser");
  const String webPass = g_webServer.arg("webPass");
  if (webUser.length()) g_cfg.webUser = webUser;
  if (webPass.length()) g_cfg.webPass = webPass;

  g_portalAuto = g_webServer.hasArg("apAuto");

  prefsSave();
  g_webServer.send(200, "text/html; charset=utf-8",
                   "<!doctype html><html><head><meta charset='utf-8'/>"
                   "<meta name='viewport' content='width=device-width,initial-scale=1'/>"
                   "<title>Saved</title></head><body><h3>Saved.</h3><p>Rebooting...</p></body></html>");
  delay(400);
  ESP.restart();
}

static void portalStart() {
  if (g_portalRunning) return;
  WiFi.mode(WIFI_AP_STA);
  const IPAddress ip(192, 168, 4, 1);
  const IPAddress mask(255, 255, 255, 0);
  WiFi.softAPConfig(ip, ip, mask);
  WiFi.softAP(portalSsid().c_str()); // open network

  g_dnsServer.start(53, "*", ip);
  g_webServer.on("/", HTTP_GET, portalHandleRoot);
  g_webServer.on("/save", HTTP_POST, portalHandleSave);
  g_webServer.onNotFound([]() {
    g_webServer.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
    g_webServer.send(302, "text/plain", "");
  });
  g_webServer.begin();
  g_portalRunning = true;
  g_webMode = WebMode::ApSetup;
}

static void portalTick() {
  if (!g_portalRunning) return;
  if (g_webMode == WebMode::ApSetup) {
    g_dnsServer.processNextRequest();
  }
  g_webServer.handleClient();
}

#include "nettemp_web_ui.inc"

static void configHandleRoot() {
  if (!webRequireAuth()) return;
  g_prefs.begin("nettemp", true);
  WebPrefsView prefs{&g_prefs};
  const String body = buildConfigPage(prefs);
  g_prefs.end();
  g_webServer.send(200, "text/html; charset=utf-8", body);
}

#if NETTEMP_ENABLE_BLE
static void configHandleSaveSensors() {
  if (!webRequireAuth()) return;
  // Apply per-device manual overrides (or clear them).
  g_prefs.begin("nettemp", false);
  for (const auto& s : g_sensors) {
    const bool hasValues = !isnan(s.temperatureC) || !isnan(s.humidityPct) || s.batteryPct >= 0;
    if (!hasValues) continue;
    const String macNo = macNoColonsUpper(s.mac);
    const String selKey = "sel_" + macNo;
    const String nameKey = "bleName_" + macNo;
    if (g_webServer.hasArg(nameKey)) {
      const String prefKey = prefKeyBleName(macNo);
      g_prefs.putString(prefKey.c_str(), g_webServer.arg(nameKey));
    }
    bleManualSet(macNo, g_webServer.hasArg(selKey));
  }
  g_prefs.end();
  prefsSave();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}
#endif  // NETTEMP_ENABLE_BLE

#if NETTEMP_ENABLE_I2C
static void configHandleSaveI2c() {
  if (!webRequireAuth()) return;
  g_i2cSelDefined = true;
  g_i2cSelAddrs.clear();

  g_prefs.begin("nettemp", false);
  for (auto& s : g_i2cSensors) {
    const String addrHex = (s.address < 16 ? String("0") : String("")) + String(s.address, HEX);
    const String key = "i2c_" + addrHex;
    const String nameKey = "i2cName_" + addrHex;
    if (g_webServer.hasArg(nameKey)) {
      const String prefKey = prefKeyI2cName(addrHex);
      g_prefs.putString(prefKey.c_str(), g_webServer.arg(nameKey));
    }
    const bool selected = g_webServer.hasArg(key);
    s.selected = selected;
    if (selected) g_i2cSelAddrs.push_back(s.address);
  }
  g_prefs.end();

  prefsSave();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

static void configHandleI2cRescan() {
  if (!webRequireAuth()) return;

  // If there is no saved selection yet, treat current selection as the baseline and persist it.
  if (!g_i2cSelDefined) {
    g_i2cSelAddrs.clear();
    for (const auto& s : g_i2cSensors) {
      if (s.selected) g_i2cSelAddrs.push_back(s.address);
    }
    g_i2cSelDefined = true;
  }

  i2cInitBus();
  g_i2cDetectedAddrs = i2cScanAllAddresses(*g_i2cBus);
  g_i2cSensors = i2cDetectKnownSensors(*g_i2cBus);
  if (!g_i2cSensors.empty()) i2cUpdateReadings(*g_i2cBus, g_i2cSensors);
  i2cApplySelectionToDetected();
  i2cPersistLastDetected();

  prefsSave();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

static void configHandleI2cPins() {
  if (!webRequireAuth()) return;

  const String sdaStr = g_webServer.arg("i2cSda");
  const String sclStr = g_webServer.arg("i2cScl");
  const int sda = sdaStr.length() ? sdaStr.toInt() : g_cfg.i2cSdaPin;
  const int scl = sclStr.length() ? sclStr.toInt() : g_cfg.i2cSclPin;

  // Allow -1 as "board default". Basic sanity range otherwise.
  g_cfg.i2cSdaPin = (sda < 0) ? -1 : std::min(48, sda);
  g_cfg.i2cSclPin = (scl < 0) ? -1 : std::min(48, scl);
  prefsSave();

  i2cInitBus();
  g_i2cDetectedAddrs = i2cScanAllAddresses(*g_i2cBus);
  g_i2cSensors = i2cDetectKnownSensors(*g_i2cBus);
  if (!g_i2cSensors.empty()) i2cUpdateReadings(*g_i2cBus, g_i2cSensors);
  if (g_i2cSelDefined) i2cApplySelectionToDetected();
  i2cPersistLastDetected();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}
#endif

#if NETTEMP_ENABLE_BLE
static void configHandleSaveBle() {
  if (!webRequireAuth()) return;
  g_bleAutoScan = g_webServer.hasArg("bleAuto");

  prefsSave();
  if (g_bleAutoScan) {
    bleConfigureScan();
  }
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

static void configHandleSaveBleMax() {
  if (!webRequireAuth()) return;

  const String maxStr = g_webServer.arg("max");
  if (maxStr.length()) {
    uint32_t maxSensors = maxStr.toInt();
    if (maxSensors < 5) maxSensors = 5;
    if (maxSensors > 200) maxSensors = 200;
    g_bleMaxResults = (uint8_t)maxSensors;

    // Prune if current count exceeds new limit (keeping selected sensors)
    if (g_sensors.size() > maxSensors) {
      std::vector<SensorRow> selected;
      std::vector<SensorRow> unselected;

      for (const auto& s : g_sensors) {
        if (s.selected || s.selectionLocked) {
          selected.push_back(s);
        } else {
          unselected.push_back(s);
        }
      }

      std::sort(unselected.begin(), unselected.end(), [](const SensorRow& a, const SensorRow& b) {
        return a.lastSeenMs > b.lastSeenMs;
      });

      g_sensors.clear();
      g_sensors = selected;

      const uint32_t remainingSlots = (maxSensors > selected.size()) ? (maxSensors - selected.size()) : 0;
      for (uint32_t i = 0; i < remainingSlots && i < unselected.size(); i++) {
        g_sensors.push_back(unselected[i]);
      }

      g_sensors.shrink_to_fit();
    }

    prefsSave();
  }

  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

static void configHandleBleScanNow() {
  if (!webRequireAuth()) return;
  bleTriggerManualScan();  // Trigger BLE scan (defined in .ino where BLE types are available)
  g_webServer.send(200, "text/plain", "OK");
}
#endif  // NETTEMP_ENABLE_BLE

static void configHandleSaveMqtt() {
  if (!webRequireAuth()) return;

  g_cfg.mqttEnabled = g_webServer.hasArg("mqttEn");
  const String mqttHost = g_webServer.arg("mqttHost");
  if (mqttHost.length()) g_cfg.mqttHost = mqttHost;
  const uint32_t mqttPort = parseU32Or(g_webServer.arg("mqttPort"), g_cfg.mqttPort);
  if (mqttPort > 0 && mqttPort <= 65535) g_cfg.mqttPort = (uint16_t)mqttPort;
  g_cfg.mqttUser = g_webServer.arg("mqttUser");
  const String mqttPass = g_webServer.arg("mqttPass");
  if (mqttPass.length()) g_cfg.mqttPass = mqttPass;
  const uint32_t mqttIntS = parseU32Or(g_webServer.arg("mqttInt"), (g_cfg.mqttIntervalMs / 1000UL));
  if (mqttIntS >= 5 && mqttIntS <= 3600) g_cfg.mqttIntervalMs = mqttIntS * 1000UL;
  g_cfg.bleSendMqtt = g_webServer.hasArg("mqttBle");
  g_cfg.gpioSendMqtt = g_webServer.hasArg("mqttGpio");
  g_cfg.i2cSendMqtt = g_webServer.hasArg("mqttI2c");
  g_mqttBleFields = parseBleMask("m_f_", (SRV_BLE_TEMPC | SRV_BLE_HUM | SRV_BLE_BATT));
  g_mqttGpioFields = parseGpioMask("m_f_gpio_", SRV_GPIO_DEFAULT);

#if NETTEMP_ENABLE_I2C
  g_mqttI2cFields = parseI2cMask("m_f_i2c_", (SRV_I2C_TEMPC | SRV_I2C_HUM | SRV_I2C_PRESS | SRV_I2C_LUX | SRV_I2C_DIST));
#endif

  prefsSave();
  mqttEnsureConnected();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

static void configHandleSaveServer() {
  if (!webRequireAuth()) return;

  g_cfg.serverEnabled = g_webServer.hasArg("srvEn");
  const String srvUrl = g_webServer.arg("srvUrl");
  if (srvUrl.length()) g_cfg.serverBaseUrl = srvUrl;
  const String srvKey = g_webServer.arg("srvKey");
  if (srvKey.length()) g_cfg.serverApiKey = srvKey;
  const uint32_t srvIntS = parseU32Or(g_webServer.arg("srvInt"), (g_cfg.serverIntervalMs / 1000UL));
  if (srvIntS >= 5 && srvIntS <= 3600) g_cfg.serverIntervalMs = srvIntS * 1000UL;
  g_cfg.bleSendServer = g_webServer.hasArg("srvBle");
  g_cfg.gpioSendServer = g_webServer.hasArg("srvGpio");
  g_cfg.i2cSendServer = g_webServer.hasArg("srvI2c");
  g_srvBleFields = parseBleMask("f_", (SRV_BLE_TEMPC | SRV_BLE_HUM | SRV_BLE_BATT));
  g_srvGpioFields = parseGpioMask("f_gpio_", SRV_GPIO_DEFAULT);

#if NETTEMP_ENABLE_I2C
  g_srvI2cFields = parseI2cMask("f_i2c_", (SRV_I2C_TEMPC | SRV_I2C_HUM | SRV_I2C_PRESS | SRV_I2C_LUX | SRV_I2C_DIST));
#endif

  prefsSave();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

static void configHandleSaveLocalServer() {
  if (!webRequireAuth()) return;

  g_cfg.localServerEnabled = g_webServer.hasArg("lsEn");
  const String lsUrl = g_webServer.arg("lsUrl");
  if (lsUrl.length()) g_cfg.localServerUrl = lsUrl;
  const String lsKey = g_webServer.arg("lsKey");
  if (lsKey.length()) g_cfg.localServerApiKey = lsKey;
  const uint32_t lsIntS = parseU32Or(g_webServer.arg("lsInt"), (g_cfg.localServerIntervalMs / 1000UL));
  if (lsIntS >= 5 && lsIntS <= 3600) g_cfg.localServerIntervalMs = lsIntS * 1000UL;

  g_cfg.bleSendLocalServer = g_webServer.hasArg("lsBle");
  g_cfg.gpioSendLocalServer = g_webServer.hasArg("lsGpio");
  g_cfg.i2cSendLocalServer = g_webServer.hasArg("lsI2c");

  g_localBleFields = parseBleMask("ls_f_", (SRV_BLE_TEMPC | SRV_BLE_HUM | SRV_BLE_BATT));
  g_localGpioFields = parseGpioMask("ls_f_gpio_", SRV_GPIO_DEFAULT_LOCAL);

#if NETTEMP_ENABLE_I2C
  g_localI2cFields = parseI2cMask("ls_f_i2c_", (SRV_I2C_TEMPC | SRV_I2C_HUM | SRV_I2C_PRESS | SRV_I2C_LUX | SRV_I2C_DIST));
#endif

  prefsSave();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

static void configHandleSaveDevice() {
  if (!webRequireAuth()) return;

  const String deviceId = g_webServer.arg("deviceId");
  if (deviceId.length()) g_cfg.deviceId = deviceId;

  prefsSave();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

static void configHandleSaveWebhook() {
  if (!webRequireAuth()) return;

  g_cfg.webhookEnabled = g_webServer.hasArg("whEn");
  const String whUrl = g_webServer.arg("whUrl");
  if (whUrl.length()) g_cfg.webhookUrl = whUrl;
  const uint32_t whIntS = parseU32Or(g_webServer.arg("whInt"), (g_cfg.webhookIntervalMs / 1000UL));
  if (whIntS >= 5 && whIntS <= 3600) g_cfg.webhookIntervalMs = whIntS * 1000UL;
  g_webhookBleFields = parseBleMask("w_f_", (SRV_BLE_TEMPC | SRV_BLE_HUM | SRV_BLE_BATT));
#if NETTEMP_ENABLE_I2C
  g_webhookI2cFields = parseI2cMask("w_f_i2c_", (SRV_I2C_TEMPC | SRV_I2C_HUM | SRV_I2C_PRESS | SRV_I2C_LUX | SRV_I2C_DIST));
#endif
  g_webhookGpioFields = parseGpioMask("w_f_gpio_", SRV_GPIO_DEFAULT);
  g_cfg.bleSendWebhook = g_webServer.hasArg("whBle");
  g_cfg.gpioSendWebhook = g_webServer.hasArg("whGpio");
  g_cfg.i2cSendWebhook = g_webServer.hasArg("whI2c");

  prefsSave();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

static void configHandleSendMqtt() {
  if (!webRequireAuth()) return;
  resetMqttSendState();
  g_lastMqttLocalSentMs = 0;
  g_sendStatusMqtt = 1;  // Set status to "sending"
  g_pendingSendMqtt = true;  // Set async flag instead of blocking
  g_webServer.send(200, "text/plain", "OK");
}

static void configHandleSendServer() {
  if (!webRequireAuth()) return;
  g_lastServerSendMs = 0;
  g_sendStatusServer = 1;  // Set status to "sending"
  g_pendingSendServer = true;  // Set async flag instead of blocking
  g_webServer.send(200, "text/plain", "OK");
}

static void configHandleSendLocalServer() {
  if (!webRequireAuth()) return;
  g_lastLocalServerSendMs = 0;
  g_sendStatusLocalServer = 1;  // Set status to "sending"
  g_pendingSendLocalServer = true;  // Set async flag instead of blocking
  g_webServer.send(200, "text/plain", "OK");
}

#if NETTEMP_ENABLE_SERVER
static void configHandleSendWebhook() {
  if (!webRequireAuth()) return;
  g_lastWebhookSendMs = 0;
  g_sendStatusWebhook = 1;  // Set status to "sending"
  g_pendingSendWebhook = true;  // Set async flag instead of blocking
  g_webServer.send(200, "text/plain", "OK");
}
#endif

// Status endpoints for AJAX polling
static void configHandleSendStatusMqtt() {
  if (!webRequireAuth()) return;
  g_webServer.send(200, "text/plain", String(g_sendStatusMqtt));
}

static void configHandleSendStatusServer() {
  if (!webRequireAuth()) return;
  g_webServer.send(200, "text/plain", String(g_sendStatusServer));
}

static void configHandleSendStatusLocalServer() {
  if (!webRequireAuth()) return;
  g_webServer.send(200, "text/plain", String(g_sendStatusLocalServer));
}

#if NETTEMP_ENABLE_SERVER
static void configHandleSendStatusWebhook() {
  if (!webRequireAuth()) return;
  g_webServer.send(200, "text/plain", String(g_sendStatusWebhook));
}
#endif

static bool g_otaFailed = false;
static String g_otaStatus;

static void configHandleOtaFinish() {
  if (!webRequireAuth()) return;
  const bool ok = !Update.hasError() && !g_otaFailed;
  String msg = ok ? "Update OK. Rebooting..." : "Update failed.";
  if (!ok && g_otaStatus.length()) {
    msg = g_otaStatus;
  }
  g_webServer.send(ok ? 200 : 500, "text/plain", msg);
  if (ok) {
    delay(300);
    ESP.restart();
  }
}

static void configHandleOtaUpload() {
  if (!webRequireAuth()) return;
  HTTPUpload& upload = g_webServer.upload();
  if (upload.status == UPLOAD_FILE_START) {
    g_otaFailed = false;
    g_otaStatus = "";
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      g_otaFailed = true;
      g_otaStatus = String("Update failed: ") + Update.errorString() + " (check OTA partition scheme)";
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (!g_otaFailed) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
        g_otaFailed = true;
        g_otaStatus = String("Update failed: ") + Update.errorString() + " (check OTA partition scheme)";
      }
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (!g_otaFailed) {
      if (!Update.end(true)) {
        g_otaFailed = true;
        g_otaStatus = String("Update failed: ") + Update.errorString() + " (check OTA partition scheme)";
      }
    }
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    Update.end();
    g_otaFailed = true;
    g_otaStatus = "Update aborted.";
  }
}

static void configHandleSaveGpio() {
  if (!webRequireAuth()) return;

  // Detect which sensor is being saved
  const String sensor = g_webServer.arg("sensor");

  // Handle scan button - update DS18B20 settings, rescan, then redirect
  if (g_webServer.hasArg("dsScan") && sensor == "ds18b20") {
    g_cfg.dsEnabled = g_webServer.hasArg("dsEn");
    g_cfg.dsPin = (int)parseU32Or(g_webServer.arg("dsPin"), (uint32_t)g_cfg.dsPin);
    dsEnsureBus();
    prefsSave();
    dsRescan();
    g_webServer.sendHeader("Location", "/", true);
    g_webServer.send(302, "text/plain", "");
    return;
  }

  // Update only the sensor being saved
  if (sensor == "ds18b20") {
    g_cfg.dsEnabled = g_webServer.hasArg("dsEn");
    g_cfg.dsPin = (int)parseU32Or(g_webServer.arg("dsPin"), (uint32_t)g_cfg.dsPin);
    dsEnsureBus();
  } else if (sensor == "dht") {
    g_cfg.dhtEnabled = g_webServer.hasArg("dhtEn");
    g_cfg.dhtSelected = g_webServer.hasArg("dhtSel");
    g_cfg.dhtPin = (int)parseU32Or(g_webServer.arg("dhtPin"), (uint32_t)g_cfg.dhtPin);
    g_cfg.dhtType = (int)parseU32Or(g_webServer.arg("dhtType"), (uint32_t)g_cfg.dhtType);
    if (g_cfg.dhtType != 11 && g_cfg.dhtType != 22) g_cfg.dhtType = 22;
    g_dhtTempC = NAN;
    g_dhtHumPct = NAN;
    g_dhtLastSeenMs = 0;
    g_dhtLastReadMs = 0;
  } else if (sensor == "soil") {
    g_cfg.soilEnabled = g_webServer.hasArg("soilEn");
    g_cfg.soilSelected = g_webServer.hasArg("soilSel");
    g_cfg.soilAdcPin = (int)parseU32Or(g_webServer.arg("soilPin"), (uint32_t)g_cfg.soilAdcPin);
    g_cfg.soilDryRaw = (int)parseU32Or(g_webServer.arg("soilDry"), (uint32_t)g_cfg.soilDryRaw);
    g_cfg.soilWetRaw = (int)parseU32Or(g_webServer.arg("soilWet"), (uint32_t)g_cfg.soilWetRaw);
  } else if (sensor == "hcsr04") {
    g_cfg.hcsr04Enabled = g_webServer.hasArg("hcEn");
    g_cfg.hcsr04Selected = g_webServer.hasArg("hcSel");
    g_cfg.hcsr04TrigPin = (int)parseU32Or(g_webServer.arg("hcTrig"), (uint32_t)g_cfg.hcsr04TrigPin);
    g_cfg.hcsr04EchoPin = (int)parseU32Or(g_webServer.arg("hcEcho"), (uint32_t)g_cfg.hcsr04EchoPin);
    g_hcsr04Cm = NAN;
    g_hcsr04LastSeenMs = 0;
    g_hcsr04LastReadMs = 0;
  } else if (sensor == "vbat") {
    const bool vbatEnabled = g_webServer.hasArg("vbEn");
    g_cfg.vbatSelected = g_webServer.hasArg("vbSel");
    if (!vbatEnabled) {
      g_cfg.vbatMode = 0;
    } else {
      const uint32_t mode = parseU32Or(g_webServer.arg("vbMode"), (uint32_t)g_cfg.vbatMode);
      g_cfg.vbatMode = (mode == 2) ? 2 : 1;
    }
    g_cfg.vbatAdcPin = (int)parseU32Or(g_webServer.arg("vbPin"), (uint32_t)g_cfg.vbatAdcPin);
    g_cfg.vbatDividerX1000 = (uint16_t)parseU32Or(g_webServer.arg("vbDiv"), (uint32_t)g_cfg.vbatDividerX1000);
    g_cfg.vbatFullMv = (uint16_t)parseU32Or(g_webServer.arg("vbFull"), (uint32_t)g_cfg.vbatFullMv);
    g_cfg.vbatEmptyMv = (uint16_t)parseU32Or(g_webServer.arg("vbEmp"), (uint32_t)g_cfg.vbatEmptyMv);
    g_cfg.vbatSendVolt = g_webServer.hasArg("vbSV");
    g_vbatVolts = NAN;
    g_vbatPct = -1;
    g_vbatLastSeenMs = 0;
  }

  g_lastMqttLocalSentMs = 0;

  // Save sensor names
  g_prefs.begin("nettemp", false);
  if (sensor == "ds18b20") {
    for (size_t i = 0; i < g_dsRoms.size(); i++) {
      char romHex[17]{};
      for (int b = 0; b < 8; b++) sprintf(romHex + b * 2, "%02X", g_dsRoms[i][b]);
      const String key = prefKeyDsName(romHex);
      const String argKey = String("dsName_") + romHex;
      if (g_webServer.hasArg(argKey)) {
        const String nameVal = g_webServer.arg(argKey);
        if (nameVal.length()) {
          g_prefs.putString(key.c_str(), nameVal);
        } else {
        }
      } else {
      }
      // Save selection state
      const String selKey = prefKeyDsSel(romHex);
      const bool selected = g_webServer.hasArg(selKey);
      g_prefs.putBool(selKey.c_str(), selected);
    }
  } else if (sensor == "dht") {
    g_prefs.putString("name_dht", g_webServer.arg("name_dht"));
  } else if (sensor == "soil") {
    g_prefs.putString("name_soil", g_webServer.arg("name_soil"));
  } else if (sensor == "hcsr04") {
    g_prefs.putString("name_hcsr04", g_webServer.arg("name_hcsr04"));
  } else if (sensor == "vbat") {
    g_prefs.putString("name_vbat", g_webServer.arg("name_vbat"));
  }
  g_prefs.end();

  prefsSave();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

static void configHandleSaveOled() {
  if (!webRequireAuth()) return;

  g_cfg.oledEnabled = g_webServer.hasArg("oledEn");
  g_cfg.oledTextSize = (uint8_t)parseU32Or(g_webServer.arg("oledSz"), g_cfg.oledTextSize);
  if (g_cfg.oledTextSize < 1 || g_cfg.oledTextSize > 3) g_cfg.oledTextSize = 1;
  g_prefs.begin("nettemp", false);
  for (const auto& s : g_sensors) {
    const String macNo = macNoColonsUpper(s.mac);
    const String selKey = prefKeyBleOledSel(macNo);
    const String tcKey = prefKeyBleOledTempc(macNo);
    const String humKey = prefKeyBleOledHum(macNo);
    const String voltKey = prefKeyBleOledVolt(macNo);
    const String battKey = prefKeyBleOledBatt(macNo);
    g_prefs.putBool(selKey.c_str(), g_webServer.hasArg(selKey));
    g_prefs.putBool(tcKey.c_str(), g_webServer.hasArg(tcKey));
    g_prefs.putBool(humKey.c_str(), g_webServer.hasArg(humKey));
    g_prefs.putBool(voltKey.c_str(), g_webServer.hasArg(voltKey));
    g_prefs.putBool(battKey.c_str(), g_webServer.hasArg(battKey));
  }
#if NETTEMP_ENABLE_I2C
  for (const auto& s : g_i2cSensors) {
    const String addrHex = (s.address < 16 ? String("0") : String("")) + String(s.address, HEX);
    const String selKey = prefKeyI2cOledSel(addrHex);
    const String tcKey = prefKeyI2cOledTempc(addrHex);
    const String humKey = prefKeyI2cOledHum(addrHex);
    const String pressKey = prefKeyI2cOledPress(addrHex);
    g_prefs.putBool(selKey.c_str(), g_webServer.hasArg(selKey));
    g_prefs.putBool(tcKey.c_str(), g_webServer.hasArg(tcKey));
    g_prefs.putBool(humKey.c_str(), g_webServer.hasArg(humKey));
    g_prefs.putBool(pressKey.c_str(), g_webServer.hasArg(pressKey));
  }
#endif
  g_prefs.putBool(PREF_OLED_DHT, g_webServer.hasArg(PREF_OLED_DHT));
  g_prefs.putBool(PREF_OLED_DHT_T, g_webServer.hasArg(PREF_OLED_DHT_T));
  g_prefs.putBool(PREF_OLED_DHT_F, g_webServer.hasArg(PREF_OLED_DHT_F));
  g_prefs.putBool(PREF_OLED_DHT_H, g_webServer.hasArg(PREF_OLED_DHT_H));
  for (size_t i = 0; i < g_dsRoms.size(); i++) {
    char romHex[17]{};
    for (int b = 0; b < 8; b++) sprintf(romHex + b * 2, "%02X", g_dsRoms[i][b]);
    const String selKey = prefKeyDsOledSel(romHex);
    const String tcKey = prefKeyDsOledTempc(romHex);
    const String tfKey = prefKeyDsOledTempf(romHex);
    g_prefs.putBool(selKey.c_str(), g_webServer.hasArg(selKey));
    g_prefs.putBool(tcKey.c_str(), g_webServer.hasArg(tcKey));
    g_prefs.putBool(tfKey.c_str(), g_webServer.hasArg(tfKey));
  }
  g_prefs.putBool(PREF_OLED_SOIL, g_webServer.hasArg(PREF_OLED_SOIL));
  g_prefs.putBool(PREF_OLED_SOIL_RAW, g_webServer.hasArg(PREF_OLED_SOIL_RAW));
  g_prefs.putBool(PREF_OLED_SOIL_PCT, g_webServer.hasArg(PREF_OLED_SOIL_PCT));
  g_prefs.putBool(PREF_OLED_HC, g_webServer.hasArg(PREF_OLED_HC));
  g_prefs.putBool(PREF_OLED_HC_DIST, g_webServer.hasArg(PREF_OLED_HC_DIST));
  g_prefs.putBool(PREF_OLED_VBAT, g_webServer.hasArg(PREF_OLED_VBAT));
  g_prefs.putBool(PREF_OLED_VBAT_V, g_webServer.hasArg(PREF_OLED_VBAT_V));
  g_prefs.putBool(PREF_OLED_VBAT_B, g_webServer.hasArg(PREF_OLED_VBAT_B));
  g_prefs.end();

  prefsSave();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

static void configHandleWatchdog() {
  if (!webRequireAuth()) return;

  g_cfg.wdtTimeoutSeconds = parseU32Or(g_webServer.arg("wdtTO"), g_cfg.wdtTimeoutSeconds);
  if (g_cfg.wdtTimeoutSeconds < 10) g_cfg.wdtTimeoutSeconds = 10;
  if (g_cfg.wdtTimeoutSeconds > 300) g_cfg.wdtTimeoutSeconds = 300;

  prefsSave();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

static void configHandleCrashLog() {
  if (!webRequireAuth()) return;

  const String log = crashGetLog();
  if (log.length() == 0) {
    g_webServer.send(404, "text/plain", "No crash log available");
    return;
  }

  g_webServer.sendHeader("Content-Disposition", "attachment; filename=crash.txt");
  g_webServer.send(200, "text/plain", log);
}

static void configHandleCrashClear() {
  if (!webRequireAuth()) return;

  crashClear();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

#if NETTEMP_ENABLE_BLE
static void configHandleCleanSensors() {
  if (!webRequireAuth()) return;

  // Remove ALL unselected sensors (keep only selected/manual ones)
  std::vector<SensorRow> selected;
  for (const auto& s : g_sensors) {
    if (s.selected || s.selectionLocked) {
      selected.push_back(s);
    }
  }

  g_sensors = selected;
  g_sensors.shrink_to_fit();

  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}
#endif  // NETTEMP_ENABLE_BLE

#if NETTEMP_ENABLE_BLE
static void configHandleFreeMemory() {
  if (!webRequireAuth()) return;

  // Prune BLE sensors to free memory (but NEVER prune selected sensors!)
  const uint32_t oldSize = g_sensors.size();
  if (g_sensors.size() > 10) {
    // Separate selected (important) sensors from unselected ones
    std::vector<SensorRow> selected;
    std::vector<SensorRow> unselected;

    for (const auto& s : g_sensors) {
      if (s.selected || s.selectionLocked) {
        selected.push_back(s);
      } else {
        unselected.push_back(s);
      }
    }

    // Sort unselected by most recently seen
    std::sort(unselected.begin(), unselected.end(), [](const SensorRow& a, const SensorRow& b) {
      return a.lastSeenMs > b.lastSeenMs;
    });

    // Rebuild: keep ALL selected + up to 10 recent unselected
    g_sensors.clear();
    g_sensors = selected;  // Keep all selected sensors

    const uint32_t remainingSlots = (10 > selected.size()) ? (10 - selected.size()) : 0;
    for (uint32_t i = 0; i < remainingSlots && i < unselected.size(); i++) {
      g_sensors.push_back(unselected[i]);
    }

    g_sensors.shrink_to_fit();
  }

  // Clear lastAdvHex strings to free memory
  for (auto& s : g_sensors) {
    s.lastAdvHex = "";
    s.lastAdvSrc = "";
    s.lastAdvUuid = "";
  }

  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}
#endif  // NETTEMP_ENABLE_BLE

static void configHandleSavePower() {
  if (!webRequireAuth()) return;

  g_cfg.powerSleepEnabled = g_webServer.hasArg("pwrSlp");
  g_cfg.powerSleepSeconds = parseU32Or(g_webServer.arg("pwrSlpS"), g_cfg.powerSleepSeconds);
  g_cfg.powerBleScanSeconds = parseU32Or(g_webServer.arg("pwrBleS"), g_cfg.powerBleScanSeconds);
  g_cfg.powerWifiWaitSeconds = parseU32Or(g_webServer.arg("pwrWiS"), g_cfg.powerWifiWaitSeconds);
  g_cfg.powerBootGraceSeconds = parseU32Or(g_webServer.arg("pwrBootS"), g_cfg.powerBootGraceSeconds);
  g_cfg.powerAwakeHoldSeconds = parseU32Or(g_webServer.arg("pwrHoldS"), g_cfg.powerAwakeHoldSeconds);
  if (g_cfg.powerSleepSeconds < 10) g_cfg.powerSleepSeconds = 10;
  if (g_cfg.powerSleepSeconds > 86400) g_cfg.powerSleepSeconds = 86400;
  if (g_cfg.powerBleScanSeconds < 1) g_cfg.powerBleScanSeconds = 1;
  if (g_cfg.powerBleScanSeconds > 30) g_cfg.powerBleScanSeconds = 30;
  if (g_cfg.powerWifiWaitSeconds < 2) g_cfg.powerWifiWaitSeconds = 2;
  if (g_cfg.powerWifiWaitSeconds > 30) g_cfg.powerWifiWaitSeconds = 30;
  if (g_cfg.powerBootGraceSeconds > 600) g_cfg.powerBootGraceSeconds = 600;
  if (g_cfg.powerAwakeHoldSeconds > 120) g_cfg.powerAwakeHoldSeconds = 120;

  g_cfg.vbatMode = (uint8_t)parseU32Or(g_webServer.arg("vbMode"), g_cfg.vbatMode);
  if (g_cfg.vbatMode > 2) g_cfg.vbatMode = 0;
  g_cfg.vbatAdcPin = (int)parseU32Or(g_webServer.arg("vbPin"), (uint32_t)g_cfg.vbatAdcPin);
  g_cfg.vbatDividerX1000 = (uint16_t)parseU32Or(g_webServer.arg("vbDiv"), (uint32_t)g_cfg.vbatDividerX1000);
  g_cfg.vbatFullMv = (uint16_t)parseU32Or(g_webServer.arg("vbFull"), (uint32_t)g_cfg.vbatFullMv);
  g_cfg.vbatEmptyMv = (uint16_t)parseU32Or(g_webServer.arg("vbEmp"), (uint32_t)g_cfg.vbatEmptyMv);
  g_cfg.vbatSendVolt = g_webServer.hasArg("vbSV");
  if (g_cfg.vbatDividerX1000 < 500) g_cfg.vbatDividerX1000 = 500;
  if (g_cfg.vbatDividerX1000 > 5000) g_cfg.vbatDividerX1000 = 5000;
  if (g_cfg.vbatFullMv < 3600) g_cfg.vbatFullMv = 4200;
  if (g_cfg.vbatFullMv > 5000) g_cfg.vbatFullMv = 4200;
  if (g_cfg.vbatEmptyMv < 2500) g_cfg.vbatEmptyMv = 3200;
  if (g_cfg.vbatEmptyMv > 4200) g_cfg.vbatEmptyMv = 3200;
  g_vbatVolts = NAN;
  g_vbatPct = -1;
  g_vbatLastSeenMs = 0;
  g_powerBootCycleDone = false;
  g_powerCycleDoneThisBoot = false;

  g_prefs.begin("nettemp", false);
  g_prefs.putString("name_vbat", g_webServer.arg("name_vbat"));
  g_prefs.end();

  prefsSave();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

static void configHandleSaveAdmin() {
  if (!webRequireAuth()) return;

  const String webUser = g_webServer.arg("webUser");
  if (webUser.length()) g_cfg.webUser = webUser;
  const String webPass = g_webServer.arg("webPass");
  if (webPass.length()) g_cfg.webPass = webPass;

  prefsSave();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

static void configHandleSave() {
  if (!webRequireAuth()) return;

  // This handler is orphaned - no form uses /save_config anymore
  // All settings are now handled by dedicated handlers:
  // - /save_gpio, /save_mqtt, /save_server, /save_power, /save_admin, etc.
  // Just redirect to home without modifying anything to avoid breaking other settings
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

static void doFactoryReset() {
  g_prefs.begin("nettemp", false);
  g_prefs.clear();
  g_prefs.end();
}

static void configHandleReset() {
  if (!webRequireAuth()) return;
  doFactoryReset();
  g_webServer.send(200, "text/plain", "Reset OK. Rebooting...");
  delay(300);
  ESP.restart();
}

static void configHandleReboot() {
  if (!webRequireAuth()) return;
  g_webServer.send(200, "text/plain", "Rebooting...");
  delay(300);
  ESP.restart();
}

static void configServerStart() {
  if (g_portalRunning && g_webMode == WebMode::StaConfig) return;
  if (!wifiConnected()) return;
  if (g_cfg.webUser.length() == 0 || g_cfg.webPass.length() == 0) return;
  if (g_portalRunning) {
    portalStop();
  }
  g_webServer.close();
  g_webServer.on("/", HTTP_GET, configHandleRoot);
  g_webServer.on("/save_config", HTTP_POST, configHandleSave);
#if NETTEMP_ENABLE_BLE
  g_webServer.on("/save_ble", HTTP_POST, configHandleSaveBle);
  g_webServer.on("/save_ble_max", HTTP_POST, configHandleSaveBleMax);
  g_webServer.on("/ble_scan_now", HTTP_POST, configHandleBleScanNow);
#endif
  g_webServer.on("/save_mqtt", HTTP_POST, configHandleSaveMqtt);
  g_webServer.on("/save_server", HTTP_POST, configHandleSaveServer);
  g_webServer.on("/save_local_server", HTTP_POST, configHandleSaveLocalServer);
  g_webServer.on("/save_webhook", HTTP_POST, configHandleSaveWebhook);
  g_webServer.on("/send_mqtt", HTTP_POST, configHandleSendMqtt);
  g_webServer.on("/send_server", HTTP_POST, configHandleSendServer);
  g_webServer.on("/send_local_server", HTTP_POST, configHandleSendLocalServer);
#if NETTEMP_ENABLE_SERVER
  g_webServer.on("/send_webhook", HTTP_POST, configHandleSendWebhook);
#endif
  g_webServer.on("/send_status_mqtt", HTTP_GET, configHandleSendStatusMqtt);
  g_webServer.on("/send_status_server", HTTP_GET, configHandleSendStatusServer);
  g_webServer.on("/send_status_local_server", HTTP_GET, configHandleSendStatusLocalServer);
#if NETTEMP_ENABLE_SERVER
  g_webServer.on("/send_status_webhook", HTTP_GET, configHandleSendStatusWebhook);
#endif
  g_webServer.on("/ota", HTTP_POST, configHandleOtaFinish, configHandleOtaUpload);
  g_webServer.on("/save_device", HTTP_POST, configHandleSaveDevice);
  g_webServer.on("/save_oled", HTTP_POST, configHandleSaveOled);
  g_webServer.on("/save_gpio", HTTP_POST, configHandleSaveGpio);
  g_webServer.on("/save_power", HTTP_POST, configHandleSavePower);
  g_webServer.on("/watchdog", HTTP_POST, configHandleWatchdog);
  g_webServer.on("/crash.txt", HTTP_GET, configHandleCrashLog);
  g_webServer.on("/crash/clear", HTTP_POST, configHandleCrashClear);
#if NETTEMP_ENABLE_BLE
  g_webServer.on("/free_memory", HTTP_POST, configHandleFreeMemory);
  g_webServer.on("/clean_sensors", HTTP_POST, configHandleCleanSensors);
#endif
  g_webServer.on("/save_admin", HTTP_POST, configHandleSaveAdmin);
#if NETTEMP_ENABLE_BLE
  g_webServer.on("/save_sensors", HTTP_POST, configHandleSaveSensors);
#endif
#if NETTEMP_ENABLE_I2C
  g_webServer.on("/save_i2c", HTTP_POST, configHandleSaveI2c);
  g_webServer.on("/i2c_rescan", HTTP_POST, configHandleI2cRescan);
  g_webServer.on("/i2c_pins", HTTP_POST, configHandleI2cPins);
#endif
  g_webServer.on("/reset", HTTP_POST, configHandleReset);
  g_webServer.on("/reboot", HTTP_POST, configHandleReboot);
  g_webServer.onNotFound([]() {
    g_webServer.sendHeader("Location", "/", true);
    g_webServer.send(302, "text/plain", "");
  });
  g_webServer.begin();
  g_portalRunning = true;
  g_webMode = WebMode::StaConfig;
}

static void webTickMaybeStartConfig() {
  if (g_webMode == WebMode::StaConfig) return;
  if (!wifiConnected()) return;
  configServerStart();
}
#endif // NETTEMP_ENABLE_PORTAL
