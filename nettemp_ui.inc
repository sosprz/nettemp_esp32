#if NETTEMP_CARDPUTER_UI
static String keyboardReadWord() {
  if (!M5Cardputer.Keyboard.isChange()) return "";
  if (!M5Cardputer.Keyboard.isPressed()) return "";
  auto state = M5Cardputer.Keyboard.keysState();
  if (!state.word.empty()) {
    String result;
    for (char c : state.word) result += c;
    return result;
  }
  return "";
}

static bool keyboardPressedEnter() {
  if (!M5Cardputer.Keyboard.isChange()) return false;
  auto state = M5Cardputer.Keyboard.keysState();
  return state.enter;
}

static bool keyboardPressedEsc() {
  if (!M5Cardputer.Keyboard.isChange()) return false;
  auto state = M5Cardputer.Keyboard.keysState();
  return state.fn;  // Use fn key as ESC/back
}

static bool keyboardPressedDel() {
  if (!M5Cardputer.Keyboard.isChange()) return false;
  auto state = M5Cardputer.Keyboard.keysState();
  return state.del;
}

static bool keyboardPressedSpace() {
  if (!M5Cardputer.Keyboard.isChange()) return false;
  auto state = M5Cardputer.Keyboard.keysState();
  return state.space;
}

static bool editText(const char* title, String& value, bool secret = false) {
  String buf = value;
  uint32_t lastDraw = 0;

  while (true) {
    M5Cardputer.update();

    if (keyboardPressedEsc()) return false;
    if (keyboardPressedEnter()) {
      value = buf;
      return true;
    }
    if (keyboardPressedDel()) {
      if (buf.length() > 0) buf.remove(buf.length() - 1);
    }
    const String w = keyboardReadWord();
    if (w.length()) {
      buf += w;
      if (buf.length() > 128) buf = buf.substring(0, 128);
    }

    const uint32_t now = millis();
    if (now - lastDraw >= 50) {
      lastDraw = now;
      M5Cardputer.Display.fillScreen(COLOR_BG);
      M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
      M5Cardputer.Display.setTextSize(1);
      M5Cardputer.Display.setCursor(0, 0);
      M5Cardputer.Display.printf("%s\n", title);
      M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
      M5Cardputer.Display.printf("Enter=OK  Fn=Back  Del=Backspace\n\n");

      M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
      String shown = buf;
      if (secret) {
        shown = "";
        shown.reserve(buf.length());
        for (size_t i = 0; i < buf.length(); i++) shown += '*';
      }
      M5Cardputer.Display.printf("> %s_\n", shown.c_str());
    }
    delay(10);
  }
}

static void showSplash() {
  M5Cardputer.Display.fillScreen(COLOR_BG);
  const int w = M5Cardputer.Display.width();
  const int h = M5Cardputer.Display.height();

  M5Cardputer.Display.fillRect(0, 0, w, 10, COLOR_ACCENT);
  M5Cardputer.Display.fillRect(0, h - 10, w, 10, COLOR_ACCENT);

  const int markSize = 46;
  const int markX = 12;
  const int markY = (h / 2) - (markSize / 2) - 8;
  M5Cardputer.Display.fillRoundRect(markX, markY, markSize, markSize, 10, COLOR_ACCENT);
  M5Cardputer.Display.setTextColor(TFT_WHITE, COLOR_ACCENT);
  M5Cardputer.Display.setTextSize(2);
  M5Cardputer.Display.setCursor(markX + 10, markY + 14);
  M5Cardputer.Display.print("nt");

  M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
  M5Cardputer.Display.setTextSize(3);
  M5Cardputer.Display.setCursor(markX + markSize + 12, markY + 6);
  M5Cardputer.Display.print("Nettemp");

  M5Cardputer.Display.setTextSize(1);
  M5Cardputer.Display.setTextColor(TFT_LIGHTGREY, COLOR_BG);
  M5Cardputer.Display.setCursor(markX + markSize + 14, markY + 36);
  M5Cardputer.Display.print("Cardputer BLE");

  delay(SPLASH_MS);
}
#else
static String keyboardReadWord() { return ""; }
static bool keyboardPressedEnter() { return false; }
static bool keyboardPressedEsc() { return false; }
static bool keyboardPressedDel() { return false; }
static bool keyboardPressedSpace() { return false; }
#endif

#if !NETTEMP_CARDPUTER_UI
static bool editText(const char* title, String& value, bool secret = false) {
  (void)title;
  (void)value;
  (void)secret;
  return false;
}

static void showSplash() {
}
#endif

#if NETTEMP_CARDPUTER_UI
static void drawHeader(const char* title) {
  M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
  M5Cardputer.Display.setTextSize(1);
  M5Cardputer.Display.setCursor(0, 0);
  M5Cardputer.Display.printf("%s\n", title);
  M5Cardputer.Display.drawFastHLine(0, 12, M5Cardputer.Display.width(), COLOR_MUTED);
}

static void drawFooter(const char* text) {
  const int y = M5Cardputer.Display.height() - 12;
  M5Cardputer.Display.fillRect(0, y, M5Cardputer.Display.width(), 12, COLOR_BG);
  M5Cardputer.Display.setCursor(0, y);
  M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
  M5Cardputer.Display.print(text);
}

static void drawMenu(const char* title, const std::vector<String>& items, int index) {
  M5Cardputer.Display.fillScreen(COLOR_BG);
  drawHeader(title);

  const int startY = 16;
  const int lineH = 14;
  for (size_t i = 0; i < items.size(); i++) {
    const int y = startY + (int)i * lineH;
    if ((int)i == index) {
      M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_ACCENT);
      M5Cardputer.Display.fillRect(0, y - 1, M5Cardputer.Display.width(), lineH, COLOR_ACCENT);
      M5Cardputer.Display.setCursor(2, y);
      M5Cardputer.Display.print(items[i]);
      M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
    } else {
      M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
      M5Cardputer.Display.setCursor(2, y);
      M5Cardputer.Display.print(items[i]);
    }
  }
}

static void viewMainMenu() {
  const std::vector<String> items = {
    "Scan: BLE devices",
    String("Auto scan: ") + (g_bleAutoScan ? "on" : "off"),
    String("Scan: I2C addresses") + (NETTEMP_ENABLE_I2C ? "" : " (disabled)"),
    String("WiFi: ") + (wifiConnected() ? "connected" : "not connected"),
    String("Send: MQTT ") + (NETTEMP_ENABLE_MQTT ? (g_cfg.mqttEnabled ? "[on]" : "[off]") : "(disabled)"),
    String("Send: Server ") + (NETTEMP_ENABLE_SERVER ? (g_cfg.serverEnabled ? "[on]" : "[off]") : "(disabled)"),
    "Pair: QR import token",
    "About"
  };
  drawMenu("NETTEMP (Cardputer)", items, g_menuIndex);

  drawFooter("BtnA=next  Enter=select  Fn=back");
}
#endif

static String buildImportUrl(uint32_t nonce) {
#if NETTEMP_ENABLE_SERVER
  ensureServerApiKey();
  String url = String(NETTEMP_APP_URL);
  url += "/import-token?token=";
  url += g_cfg.serverApiKey;
  url += "&name=";
  url += g_cfg.deviceId;
  url += "&n=";
  url += String(nonce);
  return url;
#else
  (void)nonce;
  return String(NETTEMP_APP_URL);
#endif
}

#if NETTEMP_CARDPUTER_UI
static void viewWifiSetup() {
  M5Cardputer.Display.fillScreen(COLOR_BG);
  drawHeader("WiFi");
  M5Cardputer.Display.setCursor(0, 14);
  M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
  M5Cardputer.Display.print("Enter=scan/select  Esc=back\n\n");

  M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
  M5Cardputer.Display.printf("Saved SSID: %s\n", g_cfg.wifiSsid.length() ? g_cfg.wifiSsid.c_str() : "(none)");
  M5Cardputer.Display.printf("Status: %s\n", wifiConnected() ? "connected" : "not connected");
  if (wifiConnected()) {
    M5Cardputer.Display.printf("IP: %s\n", WiFi.localIP().toString().c_str());
  }
}

static void wifiScanAndSelect() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  int n = WiFi.scanNetworks(/*async=*/false, /*hidden=*/true);
  if (n < 0) n = 0;
  int idx = 0;

  while (true) {
    M5Cardputer.update();
    if (keyboardPressedEsc()) {
      WiFi.scanDelete();
      return;
    }
    if (M5Cardputer.BtnA.wasPressed()) idx = (idx + 1) % (n == 0 ? 1 : n);

    if (keyboardPressedEnter()) {
      if (n == 0) return;
      const String ssid = WiFi.SSID(idx);
      String pass = g_cfg.wifiPass;
      if (!editText(("WiFi password for: " + ssid).c_str(), pass, true)) return;
      g_cfg.wifiSsid = ssid;
      g_cfg.wifiPass = pass;
      prefsSave();
      WiFi.begin(g_cfg.wifiSsid.c_str(), g_cfg.wifiPass.c_str());
      WiFi.scanDelete();
      return;
    }

    M5Cardputer.Display.fillScreen(COLOR_BG);
    drawHeader("WiFi: select network");
    M5Cardputer.Display.setCursor(0, 14);
    M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
    M5Cardputer.Display.print("BtnA=next  Enter=select  Fn=back\n");
    M5Cardputer.Display.drawFastHLine(0, 28, M5Cardputer.Display.width(), COLOR_MUTED);

    if (n == 0) {
      M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
      M5Cardputer.Display.setCursor(0, 40);
      M5Cardputer.Display.print("(no networks found)");
    } else {
      const int startY = 34;
      const int lineH = 14;
      const int maxLines = (M5Cardputer.Display.height() - startY - 10) / lineH;
      const int first = std::max(0, idx - (maxLines / 2));

      for (int row = 0; row < maxLines; row++) {
        const int i = first + row;
        if (i >= n) break;
        const int y = startY + row * lineH;

        const bool isSel = i == idx;
        if (isSel) {
          M5Cardputer.Display.fillRect(0, y - 1, M5Cardputer.Display.width(), lineH, COLOR_ACCENT);
          M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_ACCENT);
        } else {
          M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
        }

        const String s = WiFi.SSID(i);
        const int rssi = WiFi.RSSI(i);
        M5Cardputer.Display.setCursor(2, y);
        M5Cardputer.Display.printf("%s (%ddBm)", s.c_str(), rssi);
      }
    }

  delay(20);
  }
}
#endif

#if NETTEMP_CARDPUTER_UI
static void viewMqttSetup() {
#if !NETTEMP_ENABLE_MQTT
  M5Cardputer.Display.fillScreen(COLOR_BG);
  drawHeader("MQTT");
  M5Cardputer.Display.setCursor(0, 14);
  M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
  M5Cardputer.Display.print("MQTT disabled at compile time.\n");
  drawFooter("Esc=back");
  return;
#else
  M5Cardputer.Display.fillScreen(COLOR_BG);
  drawHeader("MQTT");
  M5Cardputer.Display.setCursor(0, 14);
  M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
  M5Cardputer.Display.print("Enter=edit  Space=toggle enable  Esc=back\n\n");

  M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
  M5Cardputer.Display.printf("Enabled: %s\n", g_cfg.mqttEnabled ? "yes" : "no");
  M5Cardputer.Display.printf("Broker: %s:%u\n", g_cfg.mqttHost.c_str(), g_cfg.mqttPort);
  M5Cardputer.Display.printf("Auth: %s\n", (g_cfg.mqttUser.length() || g_cfg.mqttPass.length()) ? "set" : "(none)");
  M5Cardputer.Display.printf("Interval: %lus\n", (unsigned long)(g_cfg.mqttIntervalMs / 1000));
#endif
  drawFooter("Fn=back");
}
#endif

#if NETTEMP_CARDPUTER_UI
static void editMqttConfig() {
#if !NETTEMP_ENABLE_MQTT
  return;
#else
  if (keyboardPressedSpace()) {
    g_cfg.mqttEnabled = !g_cfg.mqttEnabled;
    prefsSave();
    return;
  }
  String host = g_cfg.mqttHost;
  if (!editText("MQTT broker host/IP", host, false)) return;
  g_cfg.mqttHost = host;

  String portStr = String(g_cfg.mqttPort);
  if (!editText("MQTT port", portStr, false)) return;
  const int port = portStr.toInt();
  if (port > 0 && port <= 65535) g_cfg.mqttPort = (uint16_t)port;

  String user = g_cfg.mqttUser;
  if (!editText("MQTT username (optional)", user, false)) return;
  g_cfg.mqttUser = user;

  String pass = g_cfg.mqttPass;
  if (!editText("MQTT password (optional)", pass, true)) return;
  g_cfg.mqttPass = pass;

  String intStr = String(g_cfg.mqttIntervalMs / 1000);
  if (!editText("MQTT interval (seconds)", intStr, false)) return;
  const int sec = intStr.toInt();
  if (sec >= 5 && sec <= 3600) g_cfg.mqttIntervalMs = (uint32_t)sec * 1000U;

  prefsSave();
#endif
}

static void viewServerSetup() {
  M5Cardputer.Display.fillScreen(COLOR_BG);
  drawHeader("Server (Nettemp Cloud/API)");
  M5Cardputer.Display.setCursor(0, 14);
  M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
  M5Cardputer.Display.print("Enter=edit  Space=toggle enable  Esc=back\n\n");

  M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
  M5Cardputer.Display.printf("Enabled: %s\n", g_cfg.serverEnabled ? "yes" : "no");
  M5Cardputer.Display.printf("URL: %s\n", g_cfg.serverBaseUrl.c_str());
  M5Cardputer.Display.printf("API key: %s\n", g_cfg.serverApiKey.length() ? "set" : "(not set)");
  M5Cardputer.Display.printf("Interval: %lus\n", (unsigned long)(g_cfg.serverIntervalMs / 1000));
  M5Cardputer.Display.printf("I2C device id: %s\n", g_cfg.deviceId.c_str());
  drawFooter("Fn=back");
}

static void editServerConfig() {
  if (keyboardPressedSpace()) {
    g_cfg.serverEnabled = !g_cfg.serverEnabled;
    prefsSave();
    return;
  }

  String url = g_cfg.serverBaseUrl;
  if (!editText("Server base URL", url, false)) return;
  g_cfg.serverBaseUrl = url;

  String key = g_cfg.serverApiKey;
  if (!editText("API key (ntk_...)", key, true)) return;
  g_cfg.serverApiKey = key;

  String intStr = String(g_cfg.serverIntervalMs / 1000);
  if (!editText("Server interval (seconds)", intStr, false)) return;
  const int sec = intStr.toInt();
  if (sec >= 5 && sec <= 3600) g_cfg.serverIntervalMs = (uint32_t)sec * 1000U;

  String dev = g_cfg.deviceId;
  if (!editText("Device ID (I2C only)", dev, false)) return;
  g_cfg.deviceId = dev;

  prefsSave();
}

static void viewPairQr() {
#if !NETTEMP_ENABLE_SERVER
  M5Cardputer.Display.fillScreen(COLOR_BG);
  drawHeader("Pair: import token (QR)");
  M5Cardputer.Display.setCursor(0, 14);
  M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
  M5Cardputer.Display.print("Disabled at compile time.\nSet NETTEMP_ENABLE_SERVER=1.\n");
  drawFooter("Esc=back");
  return;
#else
  ensureServerApiKey();

  const uint32_t nonce = (uint32_t)(millis() / QR_ROTATE_MS);
  const String url = buildImportUrl(nonce);

  M5Cardputer.Display.fillScreen(COLOR_BG);
  drawHeader("Pair: import token (QR)");
  M5Cardputer.Display.setCursor(0, 14);
  M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
  M5Cardputer.Display.print("Scan QR while logged in\nEsc=back\n");

  // Draw QR (M5GFX helper)
  const int size = 140;
  const int x = (M5Cardputer.Display.width() - size) / 2;
  const int y = 40;
  // version=6 is a reasonable default for short URLs; library picks if 0 on some builds.
  M5Cardputer.Display.qrcode(url.c_str(), x, y, size, 6);

  M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
  M5Cardputer.Display.setCursor(0, y + size + 6);
  M5Cardputer.Display.printf("name: %s\n", g_cfg.deviceId.c_str());
  M5Cardputer.Display.printf("token: %s...\n", g_cfg.serverApiKey.substring(0, 10).c_str());
  drawFooter("Fn=back");
#endif
}

static void viewScanBle() {
  M5Cardputer.Display.fillScreen(COLOR_BG);
  drawHeader("BLE Scan (LYWSD03MMC)");
  M5Cardputer.Display.setCursor(0, 14);
  M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
  M5Cardputer.Display.printf("BtnA=next  Space=select  Fn=back\n");
  M5Cardputer.Display.drawFastHLine(0, 28, M5Cardputer.Display.width(), COLOR_MUTED);

  const auto idx = buildBleSortedIndex();
  if (!idx.empty()) {
    if (g_bleCursor < 0) g_bleCursor = 0;
    if (g_bleCursor >= (int)idx.size()) g_bleCursor = (int)idx.size() - 1;
  } else {
    g_bleCursor = 0;
  }

  const int startY = 32;
  const int lineH = 14;
  const int maxLines = (M5Cardputer.Display.height() - startY - 12) / lineH;

  for (int row = 0; row < maxLines; row++) {
    const int y = startY + row * lineH;
    if ((size_t)row >= idx.size()) break;
    const auto& s = g_sensors[idx[row]];

    const uint32_t age = millis() - s.lastSeenMs;
    const bool stale = (s.lastSeenMs > 0) && (age > STALE_AFTER_MS);
    const bool focused = row == g_bleCursor;
    const uint16_t fg = stale ? COLOR_WARN : TFT_CYAN;
    const uint16_t bg = focused ? COLOR_ACCENT : COLOR_BG;

    if (focused) {
      M5Cardputer.Display.fillRect(0, y - 1, M5Cardputer.Display.width(), lineH, COLOR_ACCENT);
      M5Cardputer.Display.setTextColor(COLOR_FG, bg);
    } else {
      M5Cardputer.Display.setTextColor(fg, bg);
    }
    M5Cardputer.Display.setCursor(0, y);

    const char sel = s.selected ? '*' : ' ';
    const String ms = macShort(s.mac);
    if (!isnan(s.temperatureC) && !isnan(s.humidityPct)) {
      M5Cardputer.Display.printf("%c %s %5.2fC %5.1f%% b%3d r%4d", sel, ms.c_str(), s.temperatureC, s.humidityPct, s.batteryPct, s.rssi);
    } else {
      M5Cardputer.Display.printf("%c %s (no decode) r%4d", sel, ms.c_str(), s.rssi);
    }
  }
}

static void viewScanI2c() {
#if !NETTEMP_ENABLE_I2C
  M5Cardputer.Display.fillScreen(COLOR_BG);
  drawHeader("I2C Sensors");
  M5Cardputer.Display.setCursor(0, 14);
  M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
  M5Cardputer.Display.print("Disabled at compile time.\nSet NETTEMP_ENABLE_I2C=1.\n");
  drawFooter("Fn=back");
  return;
#else
  M5Cardputer.Display.fillScreen(COLOR_BG);
  drawHeader("I2C Sensors (BMP/TMP/SHT/AHT)");
  M5Cardputer.Display.setCursor(0, 14);
  M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
  M5Cardputer.Display.print("BtnA=next  Space=select  Enter=rescan  Fn=back\n");

  if (g_i2cSensors.empty()) {
    M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
    M5Cardputer.Display.setCursor(0, 34);
    M5Cardputer.Display.print("(no known I2C sensors found)");
    M5Cardputer.Display.setCursor(0, 48);
    M5Cardputer.Display.print("Try: BMP280 0x76/0x77, AHTxx 0x38, SHT3x 0x44/0x45");
    drawFooter("Fn=back");
    return;
  }

  if (g_i2cCursor < 0) g_i2cCursor = 0;
  if (g_i2cCursor >= (int)g_i2cSensors.size()) g_i2cCursor = (int)g_i2cSensors.size() - 1;

  const int startY = 34;
  const int lineH = 14;
  const int maxLines = (M5Cardputer.Display.height() - startY - 12) / lineH;
  const int first = std::max(0, g_i2cCursor - (maxLines / 2));

  for (int row = 0; row < maxLines; row++) {
    const int i = first + row;
    if (i >= (int)g_i2cSensors.size()) break;
    const int y = startY + row * lineH;
    const auto& s = g_i2cSensors[i];

    const bool focused = i == g_i2cCursor;
    if (focused) {
      M5Cardputer.Display.fillRect(0, y - 1, M5Cardputer.Display.width(), lineH, COLOR_ACCENT);
      M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_ACCENT);
    } else {
      M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
    }

    const char sel = s.selected ? '*' : ' ';
    const char* name = i2cSensorTypeName(s.type);

    if (s.reading.ok) {
      if (!isnan(s.reading.pressure_hpa)) {
        M5Cardputer.Display.setCursor(0, y);
        M5Cardputer.Display.printf("%c 0x%02X %-5s %5.2fC %6.1fhPa", sel, s.address, name, s.reading.temperature_c, s.reading.pressure_hpa);
      } else if (!isnan(s.reading.humidity_pct)) {
        M5Cardputer.Display.setCursor(0, y);
        M5Cardputer.Display.printf("%c 0x%02X %-5s %5.2fC %5.1f%%", sel, s.address, name, s.reading.temperature_c, s.reading.humidity_pct);
      } else {
        M5Cardputer.Display.setCursor(0, y);
        M5Cardputer.Display.printf("%c 0x%02X %-5s %5.2fC", sel, s.address, name, s.reading.temperature_c);
      }
    } else {
      M5Cardputer.Display.setCursor(0, y);
      M5Cardputer.Display.printf("%c 0x%02X %-5s (no read)", sel, s.address, name);
    }
  }
  drawFooter("Fn=back");
#endif
}

static void viewAbout() {
  M5Cardputer.Display.fillScreen(COLOR_BG);
  drawHeader("About");
  M5Cardputer.Display.setCursor(0, 16);
  M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
  M5Cardputer.Display.print("Nettemp Cardputer\n");
  M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
  M5Cardputer.Display.print("BLE (ATC/PVVX) -> Screen/MQTT/Cloud\n\n");
  M5Cardputer.Display.printf("WiFi: %s\n", wifiConnected() ? WiFi.localIP().toString().c_str() : "(not connected)");
  drawFooter("Fn=back");
}

static void handleKeys() {
  if (keyboardPressedEsc()) {
    g_view = View::MainMenu;
    return;
  }

  if (g_view == View::MainMenu) {
    const int menuCount = 8;
    if (M5Cardputer.BtnA.wasPressed()) g_menuIndex = (g_menuIndex + 1) % menuCount;
    if (keyboardPressedEnter()) {
      switch (g_menuIndex) {
        case 0: g_view = View::ScanBle; break;
        case 1:
          g_bleAutoScan = !g_bleAutoScan;
          prefsSave();
          bleEnsureAutoScan();
          break;
        case 2: g_view = View::ScanI2c; break;
        case 3: g_view = View::WifiSetup; break;
        case 4: g_view = View::MqttSetup; break;
        case 5: g_view = View::ServerSetup; break;
        case 6: g_view = View::PairQr; break;
        case 7: g_view = View::About; break;
        default: break;
      }
    }
    return;
  }

  if (g_view == View::ScanBle) {
    const int count = (int)buildBleSortedIndex().size();
    if (M5Cardputer.BtnA.wasPressed() && count > 0) g_bleCursor = (g_bleCursor + 1) % count;
    if (keyboardPressedSpace()) {
      const auto idx = buildBleSortedIndex();
      if (!idx.empty()) {
        int cursor = g_bleCursor;
        if (cursor < 0) cursor = 0;
        if (cursor >= (int)idx.size()) cursor = (int)idx.size() - 1;
        g_sensors[idx[cursor]].selected = !g_sensors[idx[cursor]].selected;
      }
    }
    return;
  }

  if (g_view == View::ScanI2c) {
    const int count = (int)g_i2cSensors.size();
    if (M5Cardputer.BtnA.wasPressed() && count > 0) g_i2cCursor = (g_i2cCursor + 1) % count;
    if (keyboardPressedSpace() && count > 0) {
      g_i2cSensors[g_i2cCursor].selected = !g_i2cSensors[g_i2cCursor].selected;
    }
    if (keyboardPressedEnter()) {
      g_i2cDetectedAddrs = i2cScanAllAddresses(Wire);
      g_i2cSensors = i2cDetectKnownSensors(Wire);
      if (!g_i2cSensors.empty()) i2cUpdateReadings(Wire, g_i2cSensors);
      i2cApplySelectionToDetected();
      g_i2cCursor = 0;
    }
    return;
  }

  if (g_view == View::WifiSetup) {
    if (keyboardPressedEnter()) wifiScanAndSelect();
    return;
  }

  if (g_view == View::MqttSetup) {
    if (keyboardPressedSpace()) {
      g_cfg.mqttEnabled = !g_cfg.mqttEnabled;
      prefsSave();
      return;
    }
    if (keyboardPressedEnter()) editMqttConfig();
    return;
  }

  if (g_view == View::ServerSetup) {
    if (keyboardPressedSpace()) {
      g_cfg.serverEnabled = !g_cfg.serverEnabled;
      prefsSave();
      return;
    }
    if (keyboardPressedEnter()) editServerConfig();
    return;
  }
}
#endif

#if NETTEMP_CARDPUTER_UI
static void uiSetup() {
  auto cfg = M5.config();
  M5Cardputer.begin(cfg);
  M5Cardputer.Display.setRotation(1);
  M5Cardputer.Display.fillScreen(COLOR_BG);
}

static void uiLoopTick() {
  const uint32_t now = millis();
  if (now - g_lastUiMs < UI_REFRESH_MS) return;
  g_lastUiMs = now;
  switch (g_view) {
    case View::MainMenu: viewMainMenu(); break;
    case View::ScanBle: viewScanBle(); break;
    case View::ScanI2c: viewScanI2c(); break;
    case View::WifiSetup: viewWifiSetup(); break;
    case View::MqttSetup: viewMqttSetup(); break;
    case View::ServerSetup: viewServerSetup(); break;
    case View::PairQr: viewPairQr(); break;
    case View::About: viewAbout(); break;
    default: break;
  }
}
#endif
