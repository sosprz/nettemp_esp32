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
  if (!M5Cardputer.Keyboard.isPressed()) return false;  // Only on press, not release
  auto state = M5Cardputer.Keyboard.keysState();
  // M5Cardputer Advanced: backtick '`' is ESC
  if (!state.word.empty() && state.word[0] == '`') {
    return true;
  }
  return false;  // No Fn key fallback - Advanced uses backtick only
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

// M5Cardputer Advanced arrow keys (mapped to punctuation)
static bool keyboardPressedUp() {
  if (!M5Cardputer.Keyboard.isChange()) return false;
  if (!M5Cardputer.Keyboard.isPressed()) return false;  // Only on press, not release
  auto state = M5Cardputer.Keyboard.keysState();
  if (!state.word.empty()) {
    if (state.word[0] == ';') return true;
  }
  return false;
}

static bool keyboardPressedDown() {
  if (!M5Cardputer.Keyboard.isChange()) return false;
  if (!M5Cardputer.Keyboard.isPressed()) return false;  // Only on press, not release
  auto state = M5Cardputer.Keyboard.keysState();
  if (!state.word.empty()) {
    if (state.word[0] == '.') return true;
  }
  return false;
}

static bool keyboardPressedLeft() {
  if (!M5Cardputer.Keyboard.isChange()) return false;
  if (!M5Cardputer.Keyboard.isPressed()) return false;  // Only on press, not release
  auto state = M5Cardputer.Keyboard.keysState();
  if (!state.word.empty()) {
    if (state.word[0] == ',') return true;
  }
  return false;
}

static bool keyboardPressedRight() {
  if (!M5Cardputer.Keyboard.isChange()) return false;
  if (!M5Cardputer.Keyboard.isPressed()) return false;  // Only on press, not release
  auto state = M5Cardputer.Keyboard.keysState();
  if (!state.word.empty()) {
    if (state.word[0] == '/') return true;
  }
  return false;
}

static uint32_t g_showSensorsLastMs = 0;
static size_t g_showSensorsIndex = 0;
static uint32_t g_lastUiInputMs = 0;
static bool g_displaySleeping = false;
static bool g_wifiEnablePending = false;
static uint32_t g_wifiEnableMs = 0;
static uint32_t g_lastKeyMs = 0;
static const bool kDisableUiSleep = false;  // Enable screen saver after 15s
static const bool kDisableUiKeyboard = false;  // ENABLE KEYBOARD!

// Frame layout: 2px empty + 1px blue border + 2px padding = 5px total
static const int OUTER_MARGIN = 2;      // 2px empty space from edge
static const int FRAME_WIDTH = 1;       // 1px blue border
static const int INNER_PADDING = 2;     // 2px padding inside frame
static const int FRAME_PADDING = OUTER_MARGIN + FRAME_WIDTH + INNER_PADDING;  // 5px total
static const int CONTENT_X = FRAME_PADDING;  // Content starts at x=5
static const int CONTENT_Y = FRAME_PADDING;  // Content starts at y=5
static const int UI_TEXT_SIZE = 1;

static int uiFontH() {
  M5Cardputer.Display.setTextSize(UI_TEXT_SIZE);
  return M5Cardputer.Display.fontHeight();
}

static int uiBodyStartY() {
  return CONTENT_Y + uiFontH() + 6;
}

// Scrolling text animation for long strings
static String scrollText(const String& text, int maxWidth, uint32_t updateIntervalMs = 200) {
  static String s_lastText = "";
  static int s_scrollOffset = 0;
  static uint32_t s_lastScrollMs = 0;

  M5Cardputer.Display.setTextSize(UI_TEXT_SIZE);
  const int textWidth = M5Cardputer.Display.textWidth(text);

  // Reset scroll if text changed
  if (text != s_lastText) {
    s_lastText = text;
    s_scrollOffset = 0;
    s_lastScrollMs = millis();
  }

  // If text fits, no scrolling needed
  if (textWidth <= maxWidth) {
    s_scrollOffset = 0;
    return text;
  }

  // Update scroll offset
  const uint32_t now = millis();
  if (now - s_lastScrollMs >= updateIntervalMs) {
    s_lastScrollMs = now;
    s_scrollOffset++;
    // Reset to start after full scroll (add some padding)
    if (s_scrollOffset > text.length() + 5) {
      s_scrollOffset = 0;
    }
  }

  // Create scrolling effect by rotating the string
  String scrolled = text + "     " + text;  // Add spacing between loops
  if (s_scrollOffset < scrolled.length()) {
    scrolled = scrolled.substring(s_scrollOffset);
  }

  return scrolled;
}

// Forward declaration
static void drawFrame();

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
      M5Cardputer.Display.setTextSize(UI_TEXT_SIZE);
      M5Cardputer.Display.setCursor(CONTENT_X, CONTENT_Y);
      M5Cardputer.Display.printf("%s\n", title);
      const int fontH = M5Cardputer.Display.fontHeight();
      M5Cardputer.Display.setCursor(CONTENT_X, CONTENT_Y + fontH + 2);
      M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
      M5Cardputer.Display.printf("Enter=OK  Fn=Back  Del=Backspace\n\n");

      M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
      String shown = buf;
      if (secret) {
        shown = "";
        shown.reserve(buf.length());
        for (size_t i = 0; i < buf.length(); i++) shown += '*';
      }
      M5Cardputer.Display.setCursor(CONTENT_X, CONTENT_Y + (fontH * 2) + 6);
      M5Cardputer.Display.printf("> %s_\n", shown.c_str());
      drawFrame();
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

  drawFrame();
  delay(SPLASH_MS);
}
#else
static String keyboardReadWord() { return ""; }
static bool keyboardPressedEnter() { return false; }
static bool keyboardPressedEsc() { return false; }
static bool keyboardPressedDel() { return false; }
static bool keyboardPressedSpace() { return false; }
static bool keyboardPressedUp() { return false; }
static bool keyboardPressedDown() { return false; }
static bool keyboardPressedLeft() { return false; }
static bool keyboardPressedRight() { return false; }
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
// Draw battery icon with animated fill level
static void drawBatteryIcon(int x, int y, int pct) {
  // Battery dimensions
  const int battW = 18;
  const int battH = 10;
  const int termW = 2;
  const int termH = 4;

  // Battery body outline (gray)
  M5Cardputer.Display.drawRect(x, y, battW, battH, COLOR_MUTED);

  // Battery terminal (small rectangle on right)
  M5Cardputer.Display.fillRect(x + battW, y + (battH - termH) / 2, termW, termH, COLOR_MUTED);

  // Fill level based on percentage
  if (pct >= 0 && pct <= 100) {
    const int fillW = ((battW - 4) * pct) / 100;  // -4 for 2px margin on each side
    uint16_t fillColor = COLOR_FG;

    // Change color based on battery level
    if (pct <= 20) {
      fillColor = TFT_RED;      // Red for low battery
    } else if (pct <= 40) {
      fillColor = TFT_ORANGE;   // Orange for medium-low
    } else {
      fillColor = TFT_GREEN;    // Green for good battery
    }

    if (fillW > 0) {
      M5Cardputer.Display.fillRect(x + 2, y + 2, fillW, battH - 4, fillColor);
    }
  }
}

static void drawHeader(const char* title, bool centerTitle = false) {
  M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
  M5Cardputer.Display.setTextSize(UI_TEXT_SIZE);
  const int fontH = M5Cardputer.Display.fontHeight();
  const int contentWidth = M5Cardputer.Display.width() - (FRAME_PADDING * 2);
  if (centerTitle) {
    const int w = M5Cardputer.Display.textWidth(title);
    const int x = CONTENT_X + (contentWidth - w) / 2;
    M5Cardputer.Display.setCursor(x < CONTENT_X ? CONTENT_X : x, CONTENT_Y);
  } else {
    M5Cardputer.Display.setCursor(CONTENT_X, CONTENT_Y);
  }
  M5Cardputer.Display.printf("%s\n", title);

  // Draw battery icon and percentage
  const String batt = (g_vbatPct >= 0) ? (String(g_vbatPct) + "%") : String("--%");
  const int battW = M5Cardputer.Display.textWidth(batt.c_str());
  const int iconW = 22;  // Battery icon width (18 + 2 for terminal + 2 spacing)
  const int totalW = iconW + battW;
  const int battX = M5Cardputer.Display.width() - totalW - FRAME_PADDING;

  // Draw battery icon
  const int iconY = CONTENT_Y + (fontH - 10) / 2;  // Center vertically with text
  drawBatteryIcon(battX, iconY, g_vbatPct);

  // Draw percentage text
  M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
  M5Cardputer.Display.setCursor(battX + iconW, CONTENT_Y);
  M5Cardputer.Display.print(batt);

  M5Cardputer.Display.drawFastHLine(CONTENT_X, CONTENT_Y + fontH + 2, contentWidth, COLOR_MUTED);
}

static void drawFooter(const char* text) {
  M5Cardputer.Display.setTextSize(UI_TEXT_SIZE);
  const int fontH = M5Cardputer.Display.fontHeight();
  const int y = M5Cardputer.Display.height() - fontH - FRAME_PADDING;
  const int contentWidth = M5Cardputer.Display.width() - (FRAME_PADDING * 2);
  M5Cardputer.Display.fillRect(CONTENT_X, y, contentWidth, fontH, COLOR_BG);
  M5Cardputer.Display.setCursor(CONTENT_X, y);
  M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
  M5Cardputer.Display.print(text);
}

static void drawFrame() {
  const int w = M5Cardputer.Display.width();
  const int h = M5Cardputer.Display.height();
  // Draw blue frame at OUTER_MARGIN (2px from edges)
  const int frameX = OUTER_MARGIN;
  const int frameY = OUTER_MARGIN;
  const int frameW = w - (OUTER_MARGIN * 2);
  const int frameH = h - (OUTER_MARGIN * 2);

  // Top
  M5Cardputer.Display.drawFastHLine(frameX, frameY, frameW, TFT_BLUE);
  // Bottom
  M5Cardputer.Display.drawFastHLine(frameX, frameY + frameH - 1, frameW, TFT_BLUE);
  // Left
  M5Cardputer.Display.drawFastVLine(frameX, frameY, frameH, TFT_BLUE);
  // Right
  M5Cardputer.Display.drawFastVLine(frameX + frameW - 1, frameY, frameH, TFT_BLUE);
}

static void drawMenu(const char* title, const std::vector<String>& items, int index, bool centerTitle = false) {
  drawHeader(title, centerTitle);

  const int contentWidth = M5Cardputer.Display.width() - (FRAME_PADDING * 2);
  M5Cardputer.Display.setTextSize(UI_TEXT_SIZE);
  const int fontH = M5Cardputer.Display.fontHeight();
  const int startY = CONTENT_Y + fontH + 6;
  const int footerY = M5Cardputer.Display.height() - fontH - FRAME_PADDING;
  const int availableHeight = footerY - startY;

  // Calculate line height to fit maximum items (reduce gap from 2px to 1px)
  const int lineH = fontH + 1;
  const int maxLines = std::max(1, availableHeight / lineH);

  const int first = std::max(0, std::min((int)items.size() - maxLines, index - (maxLines / 2)));
  for (int row = 0; row < maxLines; row++) {
    const int i = first + row;
    if (i >= (int)items.size()) break;
    const int y = startY + row * lineH;
    const bool focused = i == index;
    const uint16_t bg = focused ? COLOR_ACCENT : COLOR_BG;
    M5Cardputer.Display.fillRect(CONTENT_X, y - 1, contentWidth, lineH, bg);
    if (focused) {
      M5Cardputer.Display.setTextColor(COLOR_FG, bg);
      M5Cardputer.Display.setCursor(CONTENT_X + 1, y);
      M5Cardputer.Display.print(items[i]);
    } else {
      M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
      M5Cardputer.Display.setCursor(CONTENT_X + 1, y);
      M5Cardputer.Display.print(items[i]);
    }
  }
}

static void viewMainMenu() {
  // Build WiFi status line with scrolling for long text
  String wifiStatus;
  if (WiFi.getMode() == WIFI_OFF) {
    wifiStatus = "WiFi: Connecting";
  } else if (g_wifiEnablePending && !wifiConnected()) {
    wifiStatus = "WiFi: Connecting";
  } else if (wifiConnected()) {
    wifiStatus = "Connected " + WiFi.SSID() + " " + WiFi.localIP().toString();
  } else {
#if NETTEMP_ENABLE_PORTAL
    wifiStatus = String("AP SSID: ") + (g_portalRunning && g_webMode == WebMode::ApSetup ? portalSsid() : "nettemp-setup");
#else
    wifiStatus = "WiFi: not connected";
#endif
  }

  // Apply scrolling if WiFi status is too long
  const int contentWidth = M5Cardputer.Display.width() - (FRAME_PADDING * 2);
  wifiStatus = scrollText(wifiStatus, contentWidth - 10);  // -10 for some margin

  const std::vector<String> items = {
    "Show sensors",
    "BLE devices",
    String("I2C devices") + (NETTEMP_ENABLE_I2C ? "" : " (disabled)"),
    "GPIO devices",
    wifiStatus,
    "Channels",
    "Reset to factory",
  };
  if (g_menuIndex < 0 || g_menuIndex >= (int)items.size()) g_menuIndex = 0;
  drawMenu("nettemp.pl", items, g_menuIndex, true);

  drawFooter("Up/Down=nav  Enter=select  Esc=back");
}
#endif

#if NETTEMP_CARDPUTER_UI
static void dsRomToHex(const std::array<uint8_t, 8>& rom, char out[17]) {
  for (int b = 0; b < 8; b++) {
    snprintf(out + (b * 2), 3, "%02X", rom[b]);
  }
  out[16] = '\0';
}

static void viewShowSensors() {
  drawHeader("Sensors", true);
  M5Cardputer.Display.setTextSize(UI_TEXT_SIZE);

  // Collect BLE sensors
  const auto idxAll = buildBleSortedIndex();
  std::vector<size_t> idx;
  idx.reserve(idxAll.size());
  for (const auto i : idxAll) {
    const auto& s = g_sensors[i];
    if (!bleIsSelected(s)) continue;
    if (isnan(s.temperatureC)) continue;
    if (isnan(s.humidityPct)) continue;
    if (s.batteryPct < 0) continue;
    idx.push_back(i);
  }

  // Count selected I2C sensors
  int i2cCount = 0;
#if NETTEMP_ENABLE_I2C
  for (const auto& s : g_i2cSensors) {
    if (s.selected) i2cCount++;
  }
#endif

  // Collect selected DS18B20 sensors
  std::vector<size_t> dsIdx;
  if (g_cfg.dsEnabled && !g_dsRoms.empty()) {
    g_prefs.begin("nettemp", true);
    for (size_t i = 0; i < g_dsRoms.size(); i++) {
      if (i >= g_dsTempsC.size() || isnan(g_dsTempsC[i])) continue;
      char romHex[17]{};
      dsRomToHex(g_dsRoms[i], romHex);
      const String selKey = prefKeyDsSel(romHex);
      const bool selected = g_prefs.getBool(selKey.c_str(), true);
      if (selected) dsIdx.push_back(i);
    }
    g_prefs.end();
  }

  // Count enabled GPIO sensors
  int gpioCount = 0;
  if (g_cfg.dhtEnabled && g_cfg.dhtSelected && (!isnan(g_dhtTempC) || !isnan(g_dhtHumPct))) gpioCount++;
  if (g_cfg.hcsr04Enabled && g_cfg.hcsr04Selected && !isnan(g_hcsr04Cm)) gpioCount++;

  if (idx.empty() && i2cCount == 0 && dsIdx.empty() && gpioCount == 0) {
    M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
    const int fontH = M5Cardputer.Display.fontHeight();
    M5Cardputer.Display.setCursor(CONTENT_X, CONTENT_Y + fontH + 6);
    M5Cardputer.Display.print("(no sensors selected)");
    drawFooter("Esc=back");
    return;
  }

  const uint32_t now = millis();
  const int totalSensors = idx.size() + i2cCount + (int)dsIdx.size() + gpioCount;
  static size_t s_lastShownIndex = 999;  // Track last shown sensor
  if (now - g_showSensorsLastMs >= 2000) {
    g_showSensorsLastMs = now;
    g_showSensorsIndex = (g_showSensorsIndex + 1) % totalSensors;
  }
  if (g_showSensorsIndex >= (size_t)totalSensors) g_showSensorsIndex = 0;

  const int contentWidth = M5Cardputer.Display.width() - (FRAME_PADDING * 2);
  const int fontH = uiFontH();
  const int footerY = M5Cardputer.Display.height() - fontH - FRAME_PADDING;
  const int contentHeight = footerY - uiBodyStartY();
  const int valueX = CONTENT_X + (contentWidth / 2);

  // Clear content area when switching sensors to avoid text overlap
  if (s_lastShownIndex != g_showSensorsIndex) {
    M5Cardputer.Display.fillRect(CONTENT_X, uiBodyStartY(), contentWidth, contentHeight, COLOR_BG);
    s_lastShownIndex = g_showSensorsIndex;
  }

  const int lineH = fontH + 2;
  int y = uiBodyStartY();
  M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
  M5Cardputer.Display.setTextSize(UI_TEXT_SIZE);

  auto drawTwoCol = [&](const char* label, const String& value) {
    M5Cardputer.Display.setCursor(CONTENT_X, y);
    M5Cardputer.Display.print(label);
    M5Cardputer.Display.setCursor(valueX, y);
    M5Cardputer.Display.print(value);
    y += lineH;
  };

  // Check if showing BLE sensor or I2C sensor
  if (g_showSensorsIndex < idx.size()) {
    // Show BLE sensor
    const auto& s = g_sensors[idx[g_showSensorsIndex]];
    const String macFull = macWithColonsUpper(s.mac);

    M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
    M5Cardputer.Display.setCursor(CONTENT_X, y);
    M5Cardputer.Display.printf("MAC: %s", macFull.c_str());
    y += lineH;
    M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
    drawTwoCol("Temperature", String(s.temperatureC, 1) + "\xB0C");
    drawTwoCol("Humidity", String(s.humidityPct, 0) + "%");
    drawTwoCol("Battery", String(s.batteryPct) + "%");
  }
#if NETTEMP_ENABLE_I2C
  else if (g_showSensorsIndex < idx.size() + i2cCount) {
    // Show I2C sensor
    int i2cIdx = g_showSensorsIndex - idx.size();
    int currentI2c = 0;
    for (const auto& s : g_i2cSensors) {
      if (!s.selected) continue;
      if (currentI2c == i2cIdx) {
        M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
        M5Cardputer.Display.setCursor(CONTENT_X, y);
        M5Cardputer.Display.printf("I2C 0x%02X %s", s.address, i2cSensorTypeName(s.type));
        y += lineH;
        M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
        if (!isnan(s.reading.temperature_c)) {
          drawTwoCol("Temperature", String(s.reading.temperature_c, 1) + "\xB0C");
        }
        if (!isnan(s.reading.humidity_pct)) {
          drawTwoCol("Humidity", String(s.reading.humidity_pct, 0) + "%");
        }
        if (!isnan(s.reading.pressure_hpa)) {
          drawTwoCol("Pressure", String(s.reading.pressure_hpa, 0) + " hPa");
        }
        if (!isnan(s.reading.light_lux)) {
          drawTwoCol("Light", String(s.reading.light_lux, 1) + " lx");
        }
        if (!isnan(s.reading.distance_mm)) {
          drawTwoCol("Distance", String(s.reading.distance_mm / 10.0f, 1) + " cm");
        }
        break;
      }
      currentI2c++;
    }
  }
#endif
  else if (g_showSensorsIndex < idx.size() + i2cCount + dsIdx.size()) {
    // Show DS18B20 sensor
    const size_t dsIndex = dsIdx[g_showSensorsIndex - idx.size() - i2cCount];
    char romHex[17]{};
    dsRomToHex(g_dsRoms[dsIndex], romHex);
    const float tempc = (dsIndex < g_dsTempsC.size()) ? g_dsTempsC[dsIndex] : NAN;

    M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
    M5Cardputer.Display.setCursor(CONTENT_X, y);
    M5Cardputer.Display.printf("ROM: %s", romHex);
    y += lineH;
    M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
    if (!isnan(tempc)) {
      drawTwoCol("Temperature", String(tempc, 1) + "\xB0C");
    }
  } else if (g_showSensorsIndex < idx.size() + i2cCount + dsIdx.size() + gpioCount) {
    // Show GPIO sensor (DHT or HC-SR04)
    int gpioIdx = g_showSensorsIndex - idx.size() - i2cCount - (int)dsIdx.size();
    int currentGpio = 0;

    // Check DHT sensor
    if (g_cfg.dhtEnabled && g_cfg.dhtSelected && (!isnan(g_dhtTempC) || !isnan(g_dhtHumPct))) {
      if (currentGpio == gpioIdx) {
        M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
        M5Cardputer.Display.setCursor(CONTENT_X, y);
        M5Cardputer.Display.printf("DHT%d pin:%d", g_cfg.dhtType, g_cfg.dhtPin);
        y += lineH;
        M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
        if (!isnan(g_dhtTempC)) {
          drawTwoCol("Temperature", String(g_dhtTempC, 1) + "\xB0C");
        }
        if (!isnan(g_dhtHumPct)) {
          drawTwoCol("Humidity", String(g_dhtHumPct, 0) + "%");
        }
        drawFooter("Esc=back");
        return;
      }
      currentGpio++;
    }

    // Check HC-SR04 sensor
    if (g_cfg.hcsr04Enabled && g_cfg.hcsr04Selected && !isnan(g_hcsr04Cm)) {
      if (currentGpio == gpioIdx) {
        M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
        M5Cardputer.Display.setCursor(CONTENT_X, y);
        M5Cardputer.Display.printf("HC-SR04 T:%d E:%d", g_cfg.hcsr04TrigPin, g_cfg.hcsr04EchoPin);
        y += lineH;
        M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
        drawTwoCol("Distance", String(g_hcsr04Cm, 0) + " cm");
        drawFooter("Esc=back");
        return;
      }
      currentGpio++;
    }
  }
  drawFooter("Esc=back");
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
  drawHeader("WiFi");
  M5Cardputer.Display.setTextSize(UI_TEXT_SIZE);
  const int fontH = M5Cardputer.Display.fontHeight();
  int y = uiBodyStartY();
  M5Cardputer.Display.setCursor(CONTENT_X, y);
  M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
  M5Cardputer.Display.print("Enter=scan/select  Space=disable WiFi  Esc=back\n\n");

  M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
  const bool wifiOff = WiFi.getMode() == WIFI_OFF;
  if (wifiOff) {
    M5Cardputer.Display.print("WiFi is disabled\n");
    drawFooter("Esc=back");
    return;
  }
  y += fontH * 2;
  if (wifiConnected()) {
    M5Cardputer.Display.printf("Connected: %s\n", WiFi.SSID().c_str());
  } else {
    M5Cardputer.Display.printf("Saved SSID: %s\n", g_cfg.wifiSsid.length() ? g_cfg.wifiSsid.c_str() : "(none)");
  }
  M5Cardputer.Display.printf("Status: %s\n", wifiConnected() ? "connected" : "not connected");
#if NETTEMP_ENABLE_PORTAL
  const bool apOn = g_portalRunning && g_webMode == WebMode::ApSetup;
  M5Cardputer.Display.printf("AP: %s\n", apOn ? "on" : "off");
#endif
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
    if (keyboardPressedDown()) idx = (idx + 1) % (n == 0 ? 1 : n);
    if (keyboardPressedUp()) idx = (idx - 1 + (n == 0 ? 1 : n)) % (n == 0 ? 1 : n);

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
    const int contentWidth = M5Cardputer.Display.width() - (FRAME_PADDING * 2);
    M5Cardputer.Display.setTextSize(UI_TEXT_SIZE);
    const int fontH = M5Cardputer.Display.fontHeight();
    const int startY = uiBodyStartY();
    M5Cardputer.Display.setCursor(CONTENT_X, startY);
    M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
    M5Cardputer.Display.print("Up/Down=nav  Enter=select  Esc=back\n");
    M5Cardputer.Display.drawFastHLine(CONTENT_X, CONTENT_Y + fontH + 2, contentWidth, COLOR_MUTED);

    if (n == 0) {
      M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
      M5Cardputer.Display.setCursor(CONTENT_X, startY + fontH + 2);
      M5Cardputer.Display.print("(no networks found)");
    } else {
      const int lineH = fontH + 2;
      const int footerY = M5Cardputer.Display.height() - fontH - FRAME_PADDING;
      const int maxLines = (footerY - (startY + fontH + 2)) / lineH;
      const int listY = startY + fontH + 2;
      const int first = std::max(0, idx - (maxLines / 2));

      for (int row = 0; row < maxLines; row++) {
        const int i = first + row;
        if (i >= n) break;
        const int y = listY + row * lineH;

        const bool isSel = i == idx;
        if (isSel) {
          M5Cardputer.Display.fillRect(CONTENT_X, y - 1, contentWidth, lineH, COLOR_ACCENT);
          M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_ACCENT);
        } else {
          M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
        }

        const String s = WiFi.SSID(i);
        const int rssi = WiFi.RSSI(i);
        M5Cardputer.Display.setCursor(CONTENT_X + 1, y);
        M5Cardputer.Display.printf("%s (%ddBm)", s.c_str(), rssi);
      }
    }
    drawFrame();

  delay(20);
  }
}
#endif

#if NETTEMP_CARDPUTER_UI
static void viewMqttSetup() {
#if !NETTEMP_ENABLE_MQTT
  drawHeader("MQTT");
  M5Cardputer.Display.setTextSize(UI_TEXT_SIZE);
  M5Cardputer.Display.setCursor(CONTENT_X, uiBodyStartY());
  M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
  M5Cardputer.Display.print("MQTT disabled at compile time.\n");
  drawFooter("Esc=back");
  return;
#else
  drawHeader("MQTT");
  M5Cardputer.Display.setTextSize(UI_TEXT_SIZE);
  M5Cardputer.Display.setCursor(CONTENT_X, uiBodyStartY());
  M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
  M5Cardputer.Display.print("Enter=edit  Space=toggle enable  Esc=back\n\n");

  M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
  M5Cardputer.Display.printf("Enabled: %s\n", g_cfg.mqttEnabled ? "yes" : "no");
  M5Cardputer.Display.printf("Broker: %s:%u\n", g_cfg.mqttHost.c_str(), g_cfg.mqttPort);
  M5Cardputer.Display.printf("Auth: %s\n", (g_cfg.mqttUser.length() || g_cfg.mqttPass.length()) ? "set" : "(none)");
  M5Cardputer.Display.printf("Interval: %lus\n", (unsigned long)(g_cfg.mqttIntervalMs / 1000));
#endif
  drawFooter("Esc=back");
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
  drawHeader("Server (Nettemp Cloud/API)");
  M5Cardputer.Display.setTextSize(UI_TEXT_SIZE);
  M5Cardputer.Display.setCursor(CONTENT_X, uiBodyStartY());
  M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
  M5Cardputer.Display.print("Enter=edit  Space=toggle enable  Esc=back\n\n");

  M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
  M5Cardputer.Display.printf("Enabled: %s\n", g_cfg.serverEnabled ? "yes" : "no");
  M5Cardputer.Display.printf("URL: %s\n", g_cfg.serverBaseUrl.c_str());
  M5Cardputer.Display.printf("API key: %s\n", g_cfg.serverApiKey.length() ? "set" : "(not set)");
  M5Cardputer.Display.printf("Interval: %lus\n", (unsigned long)(g_cfg.serverIntervalMs / 1000));
  M5Cardputer.Display.printf("I2C device id: %s\n", g_cfg.deviceId.c_str());
  drawFooter("Esc=back");
}

static void viewChannels() {
  drawHeader("Channels");
  M5Cardputer.Display.setTextSize(UI_TEXT_SIZE);
  const int fontH = uiFontH();
  const int lineH = fontH + 2;
  const int contentWidth = M5Cardputer.Display.width() - (FRAME_PADDING * 2);
  const int footerY = M5Cardputer.Display.height() - fontH - FRAME_PADDING;
  const int contentHeight = footerY - uiBodyStartY();
  M5Cardputer.Display.fillRect(CONTENT_X, uiBodyStartY(), contentWidth, contentHeight, COLOR_BG);
  M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);

  int y = uiBodyStartY();
  auto onOff = [](bool v) { return v ? "on" : "off"; };
  M5Cardputer.Display.setCursor(CONTENT_X, y);
  M5Cardputer.Display.printf("MQTT: %s", onOff(g_cfg.mqttEnabled));
  y += lineH;
  M5Cardputer.Display.setCursor(CONTENT_X, y);
  M5Cardputer.Display.printf("Server: %s", onOff(g_cfg.serverEnabled));
  y += lineH;
  M5Cardputer.Display.setCursor(CONTENT_X, y);
  M5Cardputer.Display.printf("Local server: %s", onOff(g_cfg.localServerEnabled));
  y += lineH;
  M5Cardputer.Display.setCursor(CONTENT_X, y);
  M5Cardputer.Display.printf("Webhook: %s", onOff(g_cfg.webhookEnabled));
  drawFooter("Esc=back");
}

static void viewResetConfirm() {
  drawHeader("Reset");
  M5Cardputer.Display.setTextSize(UI_TEXT_SIZE);
  const int fontH = uiFontH();
  const int contentWidth = M5Cardputer.Display.width() - (FRAME_PADDING * 2);
  const int footerY = M5Cardputer.Display.height() - fontH - FRAME_PADDING;
  const int contentHeight = footerY - uiBodyStartY();
  M5Cardputer.Display.fillRect(CONTENT_X, uiBodyStartY(), contentWidth, contentHeight, COLOR_BG);
  M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
  M5Cardputer.Display.setCursor(CONTENT_X, uiBodyStartY());
  M5Cardputer.Display.print("Reset to factory?");
  M5Cardputer.Display.setCursor(CONTENT_X, uiBodyStartY() + fontH + 6);
  M5Cardputer.Display.print("Enter=Yes  Esc=No");
  drawFooter("Enter=Yes  Esc=No");
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
  drawHeader("Pair: import token (QR)");
  M5Cardputer.Display.setTextSize(UI_TEXT_SIZE);
  M5Cardputer.Display.setCursor(CONTENT_X, uiBodyStartY());
  M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
  M5Cardputer.Display.print("Disabled at compile time.\nSet NETTEMP_ENABLE_SERVER=1.\n");
  drawFooter("Esc=back");
  return;
#else
  ensureServerApiKey();

  const uint32_t nonce = (uint32_t)(millis() / QR_ROTATE_MS);
  const String url = buildImportUrl(nonce);

  drawHeader("Pair: import token (QR)");
  M5Cardputer.Display.setTextSize(UI_TEXT_SIZE);
  const int fontH = M5Cardputer.Display.fontHeight();
  M5Cardputer.Display.setCursor(CONTENT_X, uiBodyStartY());
  M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
  M5Cardputer.Display.print("Scan QR while logged in\nEsc=back\n");

  // Draw QR (M5GFX helper)
  const int size = 120;  // Smaller to fit with new padding
  const int x = (M5Cardputer.Display.width() - size) / 2;
  const int y = uiBodyStartY() + (fontH * 2);
  // version=6 is a reasonable default for short URLs; library picks if 0 on some builds.
  M5Cardputer.Display.qrcode(url.c_str(), x, y, size, 6);

  M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
  M5Cardputer.Display.setCursor(CONTENT_X, y + size + 6);
  M5Cardputer.Display.printf("name: %s\n", g_cfg.deviceId.c_str());
  M5Cardputer.Display.printf("token: %s...\n", g_cfg.serverApiKey.substring(0, 10).c_str());
  drawFooter("Esc=back");
#endif
}

static const char* rssiBars(int rssi) {
  if (rssi >= -60) return "||||";
  if (rssi >= -70) return "||| ";
  if (rssi >= -80) return "||  ";
  if (rssi >= -90) return "|   ";
  return ".   ";
}

static void viewScanBle() {
  drawHeader("BLE Scan (LYWSD03MMC)");
  M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
  M5Cardputer.Display.setTextSize(UI_TEXT_SIZE);

  const auto idxAll = buildBleSortedIndex();
  std::vector<size_t> idx;
  idx.reserve(idxAll.size());
  for (const auto i : idxAll) {
    const auto& s = g_sensors[i];
    if (isnan(s.temperatureC)) continue;
    if (isnan(s.humidityPct)) continue;
    if (s.batteryPct < 0) continue;
    idx.push_back(i);
  }
  if (!idx.empty()) {
    if (g_bleCursor < 0) g_bleCursor = 0;
    if (g_bleCursor >= (int)idx.size()) g_bleCursor = (int)idx.size() - 1;
  } else {
    g_bleCursor = 0;
  }

  const int fontH = M5Cardputer.Display.fontHeight();
  const int startY = uiBodyStartY();
  const int lineH = fontH + 2;
  const int contentWidth = M5Cardputer.Display.width() - (FRAME_PADDING * 2);
  const int footerY = M5Cardputer.Display.height() - fontH - FRAME_PADDING;
  const int maxLines = (footerY - startY) / lineH;

  for (int row = 0; row < maxLines; row++) {
    const int y = startY + row * lineH;
    if ((size_t)row >= idx.size()) break;
    const auto& s = g_sensors[idx[row]];

    const uint32_t age = millis() - s.lastSeenMs;
    const bool stale = (s.lastSeenMs > 0) && (age > STALE_AFTER_MS);
    const bool focused = row == g_bleCursor;
    const uint16_t fg = stale ? COLOR_WARN : TFT_CYAN;
    const uint16_t bg = focused ? COLOR_ACCENT : COLOR_BG;

    M5Cardputer.Display.fillRect(CONTENT_X, y - 1, contentWidth, lineH, bg);
    if (focused) {
      M5Cardputer.Display.setTextColor(COLOR_FG, bg);
    } else {
      M5Cardputer.Display.setTextColor(fg, bg);
    }
    M5Cardputer.Display.setCursor(CONTENT_X, y);

    const char sel = bleIsSelected(s) ? '*' : ' ';
    const String ms = macShort(s.mac);
    M5Cardputer.Display.printf("%c %s %5.2f\xB0C %5.1f%% %3d%%V %s",
                               sel, ms.c_str(), s.temperatureC, s.humidityPct, s.batteryPct, rssiBars(s.rssi));
  }
  drawFooter("Up/Down=nav  Space=select  Esc=back");
}

static void viewScanI2c() {
#if !NETTEMP_ENABLE_I2C
  drawHeader("I2C Sensors");
  M5Cardputer.Display.setTextSize(UI_TEXT_SIZE);
  M5Cardputer.Display.setCursor(CONTENT_X, uiBodyStartY());
  M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
  M5Cardputer.Display.print("Disabled at compile time.\nSet NETTEMP_ENABLE_I2C=1.\n");
  drawFooter("Esc=back");
  return;
#else
  drawHeader("I2C Sensors");

  if (g_i2cSensors.empty()) {
    M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
    const int fontH = uiFontH();
    M5Cardputer.Display.setCursor(CONTENT_X, uiBodyStartY());
    M5Cardputer.Display.print("No I2C sensors found");
    M5Cardputer.Display.setCursor(CONTENT_X, uiBodyStartY() + fontH + 2);
    M5Cardputer.Display.print("Try: BMP280 0x76/0x77, AHTxx 0x38, SHT3x 0x44/0x45");
    drawFooter("Esc=back");
    return;
  }

  if (g_i2cCursor < 0) g_i2cCursor = 0;
  if (g_i2cCursor >= (int)g_i2cSensors.size()) g_i2cCursor = (int)g_i2cSensors.size() - 1;

  const int fontH = uiFontH();
  const int startY = uiBodyStartY();
  const int lineH = fontH + 2;
  const int contentWidth = M5Cardputer.Display.width() - (FRAME_PADDING * 2);
  const int footerY = M5Cardputer.Display.height() - fontH - FRAME_PADDING;
  const int maxLines = (footerY - startY) / lineH;
  const int first = std::max(0, g_i2cCursor - (maxLines / 2));

  for (int row = 0; row < maxLines; row++) {
    const int i = first + row;
    if (i >= (int)g_i2cSensors.size()) break;
    const int y = startY + row * lineH;
    const auto& s = g_i2cSensors[i];

    const bool focused = i == g_i2cCursor;
    const uint16_t bg = focused ? COLOR_ACCENT : COLOR_BG;
    M5Cardputer.Display.fillRect(CONTENT_X, y - 1, contentWidth, lineH, bg);
    if (focused) {
      M5Cardputer.Display.setTextColor(COLOR_FG, bg);
    } else {
      M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
    }

    const char sel = s.selected ? '*' : ' ';
    const char* typeName = i2cSensorTypeName(s.type);

    // Get custom name if configured
    String customName;
    char addrHex[8];
    snprintf(addrHex, sizeof(addrHex), "%02X", s.address);
    g_prefs.begin("nettemp", true);
    customName = g_prefs.getString(prefKeyI2cName(String(addrHex)).c_str(), "");
    g_prefs.end();

    const char* displayName = customName.length() ? customName.c_str() : typeName;

    if (s.reading.ok) {
      M5Cardputer.Display.setCursor(CONTENT_X, y);
      if (!isnan(s.reading.pressure_hpa)) {
        M5Cardputer.Display.printf("%c %-8s %5.2fC %6.1fhPa", sel, displayName, s.reading.temperature_c, s.reading.pressure_hpa);
      } else if (!isnan(s.reading.humidity_pct)) {
        M5Cardputer.Display.printf("%c %-8s %5.2fC %5.1f%%", sel, displayName, s.reading.temperature_c, s.reading.humidity_pct);
      } else if (!isnan(s.reading.light_lux)) {
        M5Cardputer.Display.printf("%c %-8s %5.1flx", sel, displayName, s.reading.light_lux);
      } else if (!isnan(s.reading.distance_mm)) {
        M5Cardputer.Display.printf("%c %-8s %4.1fcm", sel, displayName, s.reading.distance_mm / 10.0f);
      } else if (!isnan(s.reading.temperature_c)) {
        M5Cardputer.Display.printf("%c %-8s %5.2fC", sel, displayName, s.reading.temperature_c);
      } else {
        M5Cardputer.Display.printf("%c %-8s no data", sel, displayName);
      }
    } else {
      M5Cardputer.Display.setCursor(CONTENT_X, y);
      M5Cardputer.Display.printf("%c %-8s no read", sel, displayName);
    }
  }
  drawFooter("Up/Down=nav  Space=select  Enter=rescan  Esc=back");
#endif
}

static int g_gpioDevicesCursor = 0;
static int g_dsCursor = 0;

static void viewGpioDevices() {
  drawHeader("GPIO Devices");
  const std::vector<String> items = {
    String("DS18B20: ") + (g_cfg.dsEnabled ? "enabled" : "disabled"),
    String("HC-SR04: ") + (g_cfg.hcsr04Enabled ? "enabled" : "disabled"),
    String("DHT11/22: ") + (g_cfg.dhtEnabled ? "enabled" : "disabled"),
  };
  if (g_gpioDevicesCursor < 0 || g_gpioDevicesCursor >= (int)items.size()) g_gpioDevicesCursor = 0;

  const int fontH = uiFontH();
  const int startY = uiBodyStartY();
  const int lineH = fontH + 2;
  for (size_t i = 0; i < items.size(); i++) {
    const int y = startY + (int)i * lineH;
    const bool focused = (int)i == g_gpioDevicesCursor;
    const uint16_t bg = focused ? COLOR_ACCENT : COLOR_BG;
    M5Cardputer.Display.fillRect(CONTENT_X, y - 1, M5Cardputer.Display.width() - (FRAME_PADDING * 2), lineH, bg);
    M5Cardputer.Display.setTextColor(COLOR_FG, bg);
    M5Cardputer.Display.setCursor(CONTENT_X + 1, y);
    M5Cardputer.Display.print(items[i]);
  }
  drawFooter("Up/Down=nav  Enter=configure  Esc=back");
}

static int editGpioPin(const char* title, int currentPin) {
  const int validPins[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
                           13, 14, 15, 16, 17, 18, 21,
                           33, 34, 35, 36, 37, 38, 39, 40};
  const int pinCount = sizeof(validPins) / sizeof(validPins[0]);

  // Find current pin index
  int idx = 0;
  for (int i = 0; i < pinCount; i++) {
    if (validPins[i] == currentPin) {
      idx = i;
      break;
    }
  }

  int lastIdx = -1;
  bool needsRedraw = true;

  while (true) {
    M5Cardputer.update();

    // Only redraw when value changes
    if (needsRedraw || idx != lastIdx) {
      // Clear only the value area to reduce flicker
      M5Cardputer.Display.fillRect(0, 14, M5Cardputer.Display.width(), 60, COLOR_BG);
      drawHeader(title);
      M5Cardputer.Display.setCursor(CONTENT_X, uiBodyStartY() + 10);
      M5Cardputer.Display.setTextSize(UI_TEXT_SIZE);
      M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
      M5Cardputer.Display.printf("GPIO: %d", validPins[idx]);
      drawFooter(";.=nav  ,/=nav  s=SAVE  `=cancel");
      drawFrame();  // Redraw frame
      lastIdx = idx;
      needsRedraw = false;
    }

    // Check character input FIRST (handles all keys consistently)
    String word = keyboardReadWord();
    if (word.length() > 0) {
      char c = word[0];
      // PRIMARY: 's' = SAVE, '`' = cancel
      if (c == 's' || c == 'S') {
        // SAVE and exit
        delay(200);
        return validPins[idx];
      } else if (c == '`') {
        // Cancel and return original
        delay(200);
        return currentPin;
      }
      // NAVIGATION: ';' = up, '.' = down, ',' = left, '/' = right
      else if (c == ';' || c == ',') {
        idx = (idx - 1 + pinCount) % pinCount;
        delay(150);
      } else if (c == '.' || c == '/') {
        idx = (idx + 1) % pinCount;
        delay(150);
      }
    }

    // Also check Enter key for save
    if (keyboardPressedEnter()) {
      delay(200);
      return validPins[idx];
    }

    delay(50);
  }
}

static void viewGpioDs18b20() {
  drawHeader("DS18B20");
  M5Cardputer.Display.setTextSize(UI_TEXT_SIZE);
  const int fontH = uiFontH();
  const int lineH = fontH + 2;
  const int contentWidth = M5Cardputer.Display.width() - (FRAME_PADDING * 2);
  const int footerY = M5Cardputer.Display.height() - fontH - FRAME_PADDING;
  const int contentHeight = footerY - uiBodyStartY();
  M5Cardputer.Display.fillRect(CONTENT_X, uiBodyStartY(), contentWidth, contentHeight, COLOR_BG);

  int y = uiBodyStartY();
  M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
  M5Cardputer.Display.setCursor(CONTENT_X, y);
  M5Cardputer.Display.printf("En:%s GPIO:%d Det:%u",
    g_cfg.dsEnabled ? "Y" : "N",
    g_cfg.dsPin,
    (unsigned)g_dsRoms.size());
  y += lineH + 2;

  if (g_dsRoms.empty()) {
    M5Cardputer.Display.setCursor(CONTENT_X, y);
    M5Cardputer.Display.print("(no DS18B20 devices)");
    drawFooter("e=enable  p=GPIO  r=scan  `=back");
    return;
  }

  if (g_dsCursor < 0) g_dsCursor = 0;
  if (g_dsCursor >= (int)g_dsRoms.size()) g_dsCursor = (int)g_dsRoms.size() - 1;

  const int maxLines = (footerY - y) / lineH;
  const int first = std::max(0, g_dsCursor - (maxLines / 2));
  g_prefs.begin("nettemp", true);
  for (int row = 0; row < maxLines; row++) {
    const int i = first + row;
    if (i >= (int)g_dsRoms.size()) break;
    const int rowY = y + row * lineH;

    char romHex[17]{};
    dsRomToHex(g_dsRoms[i], romHex);
    const String selKey = prefKeyDsSel(romHex);
    const bool selected = g_prefs.getBool(selKey.c_str(), true);
    const float tempc = (i < (int)g_dsTempsC.size()) ? g_dsTempsC[i] : NAN;

    const bool focused = i == g_dsCursor;
    const uint16_t bg = focused ? COLOR_ACCENT : COLOR_BG;
    M5Cardputer.Display.fillRect(CONTENT_X, rowY - 1, contentWidth, lineH, bg);
    M5Cardputer.Display.setTextColor(COLOR_FG, bg);
    M5Cardputer.Display.setCursor(CONTENT_X, rowY);
    if (isnan(tempc)) {
      M5Cardputer.Display.printf("%c %s --", selected ? '*' : ' ', romHex);
    } else {
      M5Cardputer.Display.printf("%c %s %5.2f\xB0C", selected ? '*' : ' ', romHex, tempc);
    }
  }
  g_prefs.end();
  drawFooter(";.=nav  Sp=sel  e=en  p=GPIO  r=scan  `=back");
}

static void viewGpioHcsr04() {
  drawHeader("HC-SR04 Ultrasonic");
  M5Cardputer.Display.setTextSize(UI_TEXT_SIZE);
  const int fontH = uiFontH();
  const int lineH = fontH + 2;
  const int contentWidth = M5Cardputer.Display.width() - (FRAME_PADDING * 2);
  const int footerY = M5Cardputer.Display.height() - fontH - FRAME_PADDING;
  const int contentHeight = footerY - uiBodyStartY();
  M5Cardputer.Display.fillRect(CONTENT_X, uiBodyStartY(), contentWidth, contentHeight, COLOR_BG);
  M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
  int y = uiBodyStartY();
  M5Cardputer.Display.setCursor(CONTENT_X, y);
  M5Cardputer.Display.printf("Enabled: %s", g_cfg.hcsr04Enabled ? "yes" : "no");
  y += lineH;
  M5Cardputer.Display.setCursor(CONTENT_X, y);
  M5Cardputer.Display.printf("Send: %s", g_cfg.hcsr04Selected ? "yes" : "no");
  y += lineH;
  M5Cardputer.Display.setCursor(CONTENT_X, y);
  M5Cardputer.Display.printf("Trigger: GPIO %d", g_cfg.hcsr04TrigPin);
  y += lineH;
  M5Cardputer.Display.setCursor(CONTENT_X, y);
  M5Cardputer.Display.printf("Echo: GPIO %d", g_cfg.hcsr04EchoPin);
  drawFooter("t=trig  e=echo  s=send  Sp=toggle  `=back");
}

static void viewGpioDht() {
  drawHeader("DHT11/DHT22");
  M5Cardputer.Display.setTextSize(UI_TEXT_SIZE);
  const int fontH = uiFontH();
  const int lineH = fontH + 2;
  const int contentWidth = M5Cardputer.Display.width() - (FRAME_PADDING * 2);
  const int footerY = M5Cardputer.Display.height() - fontH - FRAME_PADDING;
  const int contentHeight = footerY - uiBodyStartY();
  M5Cardputer.Display.fillRect(CONTENT_X, uiBodyStartY(), contentWidth, contentHeight, COLOR_BG);
  M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
  int y = uiBodyStartY();
  M5Cardputer.Display.setCursor(CONTENT_X, y);
  M5Cardputer.Display.printf("Enabled: %s", g_cfg.dhtEnabled ? "yes" : "no");
  y += lineH;
  M5Cardputer.Display.setCursor(CONTENT_X, y);
  M5Cardputer.Display.printf("Send: %s", g_cfg.dhtSelected ? "yes" : "no");
  y += lineH;
  M5Cardputer.Display.setCursor(CONTENT_X, y);
  M5Cardputer.Display.printf("Type: DHT%d", g_cfg.dhtType);
  y += lineH;
  M5Cardputer.Display.setCursor(CONTENT_X, y);
  M5Cardputer.Display.printf("Pin: GPIO %d", g_cfg.dhtPin);
  drawFooter("p=GPIO  y=type  s=send  Sp=toggle  `=back");
}

static void viewAbout() {
  drawHeader("About");
  M5Cardputer.Display.setTextSize(UI_TEXT_SIZE);
  M5Cardputer.Display.setCursor(CONTENT_X, uiBodyStartY());
  M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
  M5Cardputer.Display.print("Nettemp Cardputer\n");
  M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
  M5Cardputer.Display.print("BLE (ATC/PVVX) -> Screen/MQTT/Cloud\n\n");
  M5Cardputer.Display.printf("WiFi: %s\n", wifiConnected() ? WiFi.localIP().toString().c_str() : "(not connected)");
  drawFooter("Esc=back");
}

static void handleKeys() {
  if (kDisableUiKeyboard) return;
  const uint32_t now = millis();
  auto state = M5Cardputer.Keyboard.keysState();
  const auto& keyList = M5Cardputer.Keyboard.keyList();
  const bool anyKey = state.enter || state.space || state.del || !state.word.empty() || !keyList.empty();

  if (g_displaySleeping) {
    if (anyKey) {
      M5Cardputer.Display.wakeup();
      g_displaySleeping = false;
      g_lastUiInputMs = now;
      g_viewNeedsRedraw = true;
      g_viewNeedsClear = true;
    }
    return;
  }
  if (!anyKey) return;
  if ((now - g_lastKeyMs) < 120) return;
  g_lastKeyMs = now;

  // Detect which key was pressed
  char keyChar = '\0';
  if (!state.word.empty()) {
    keyChar = state.word[0];
  } else if (!keyList.empty()) {
    const auto key = keyList[0];
    const auto kv = M5Cardputer.Keyboard.getKeyValue(key);
    keyChar = state.shift ? kv.value_second : kv.value_first;
  }

  bool isEnter = state.enter;
  bool isSpace = state.space || (keyChar == ' ');
  bool isEsc = (keyChar == '`');  // M5Cardputer Advanced: backtick is ESC
  bool isDown = (keyChar == '.');  // M5Cardputer Advanced arrow keys
  bool isUp = (keyChar == ';');
  bool isLeft = (keyChar == ',');
  bool isRight = (keyChar == '/');

  // Any key press triggers immediate screen redraw
  g_lastUiInputMs = now;
  g_viewNeedsRedraw = true;
  g_lastUiMs = 0;  // Force immediate UI update by resetting timer

  // ESC navigation
  if (isEsc) {
    // From GPIO config screens, go back to GPIO devices menu
    if (g_view == View::GpioDs18b20 || g_view == View::GpioHcsr04 || g_view == View::GpioDht) {
      g_view = View::GpioDevices;
      return;
    }
    // From all other views, go to main menu
    g_view = View::MainMenu;
    return;
  }

  if (g_view == View::MainMenu) {
    const int menuCount = 7;
    if (isDown) {
      g_menuIndex = (g_menuIndex + 1) % menuCount;
    }
    if (isUp) {
      g_menuIndex = (g_menuIndex - 1 + menuCount) % menuCount;
    }
    if (isEnter) {
      switch (g_menuIndex) {
        case 0: g_view = View::ShowSensors; break;
        case 1: g_view = View::ScanBle; break;
        case 2: g_view = View::ScanI2c; break;
        case 3: g_view = View::GpioDevices; break;
        case 4:
          if (WiFi.getMode() == WIFI_OFF) {
            // WiFi OFF -> enable and connect
            WiFi.mode(WIFI_STA);
            wifiConnectIfConfigured();
            g_wifiEnablePending = true;
            g_wifiEnableMs = millis();
          } else if (wifiConnected()) {
            // WiFi connected
#if NETTEMP_ENABLE_PORTAL
            if (g_portalRunning) {
              // Connected + AP on -> stop AP, stay connected
              portalStop();
            } else {
              // Connected + AP off -> disconnect WiFi
              WiFi.disconnect(true);
              WiFi.mode(WIFI_OFF);
              g_wifiEnablePending = false;
            }
#else
            WiFi.disconnect(true);
            WiFi.mode(WIFI_OFF);
            g_wifiEnablePending = false;
#endif
          } else {
            // WiFi not connected (but not OFF)
#if NETTEMP_ENABLE_PORTAL
            if (!g_portalRunning) {
              // Not connected + AP off -> start AP for configuration
              portalStart();
            } else {
              // Not connected + AP on -> stop AP and try to connect to saved WiFi
              portalStop();
              wifiConnectIfConfigured();
              g_wifiEnablePending = true;
              g_wifiEnableMs = millis();
            }
#endif
          }
          break;
        case 5:
          g_view = View::Channels;
          break;
        case 6:
          g_view = View::ResetConfirm;
          break;
        default: break;
      }
    }
    return;
  }

#if NETTEMP_ENABLE_BLE
  if (g_view == View::ScanBle) {
    const auto idxAll = buildBleSortedIndex();
    std::vector<size_t> idx;
    idx.reserve(idxAll.size());
    for (const auto i : idxAll) {
      const auto& s = g_sensors[i];
      if (isnan(s.temperatureC)) continue;
      if (isnan(s.humidityPct)) continue;
      if (s.batteryPct < 0) continue;
      idx.push_back(i);
    }
    const int count = (int)idx.size();
    if (isDown && count > 0) g_bleCursor = (g_bleCursor + 1) % count;
    if (isUp && count > 0) g_bleCursor = (g_bleCursor - 1 + count) % count;
    if (isSpace) {
      if (!idx.empty()) {
        int cursor = g_bleCursor;
        if (cursor < 0) cursor = 0;
        if (cursor >= (int)idx.size()) cursor = (int)idx.size() - 1;
        auto& s = g_sensors[idx[cursor]];
        const String macNo = macNoColonsUpper(s.mac);
        bool manualSelected = false;
        const bool manual = bleManualGetSelected(macNo, manualSelected);
        const bool current = manual ? manualSelected : s.selected;
        const bool next = !current;
        if (next) {
          bleManualSet(macNo, true);
        } else {
          bleManualClear(macNo);
        }
        s.selected = next;
        s.selectionLocked = next;
        g_bleStrictMode = bleManualHasAnySelected();
        prefsSave();
      }
    }
    return;
  }
#endif

#if NETTEMP_ENABLE_I2C
  if (g_view == View::ScanI2c) {
    const int count = (int)g_i2cSensors.size();
    if (isDown && count > 0) g_i2cCursor = (g_i2cCursor + 1) % count;
    if (isUp && count > 0) g_i2cCursor = (g_i2cCursor - 1 + count) % count;
    if (isSpace && count > 0) {
      g_i2cSensors[g_i2cCursor].selected = !g_i2cSensors[g_i2cCursor].selected;
    }
    if (isEnter) {
      g_i2cDetectedAddrs = i2cScanAllAddresses(*g_i2cBus);
      g_i2cSensors = i2cDetectKnownSensors(*g_i2cBus);
      if (!g_i2cSensors.empty()) i2cUpdateReadings(*g_i2cBus, g_i2cSensors);
      i2cApplySelectionToDetected();
      g_i2cCursor = 0;
    }
    return;
  }
#endif

  if (g_view == View::GpioDevices) {
    const int count = 3;
    if (isDown) g_gpioDevicesCursor = (g_gpioDevicesCursor + 1) % count;
    if (isUp) g_gpioDevicesCursor = (g_gpioDevicesCursor - 1 + count) % count;
    if (isEnter) {
      switch (g_gpioDevicesCursor) {
        case 0: g_view = View::GpioDs18b20; break;
        case 1: g_view = View::GpioHcsr04; break;
        case 2: g_view = View::GpioDht; break;
        default: break;
      }
    }
    return;
  }

  if (g_view == View::GpioDs18b20) {
    const int count = (int)g_dsRoms.size();
    if (isDown && count > 0) {
      g_dsCursor = (g_dsCursor + 1) % count;
      g_viewNeedsRedraw = true;
    }
    if (isUp && count > 0) {
      g_dsCursor = (g_dsCursor - 1 + count) % count;
      g_viewNeedsRedraw = true;
    }
    if ((isSpace || isEnter) && count > 0) {
      if (g_dsCursor < 0) g_dsCursor = 0;
      if (g_dsCursor >= count) g_dsCursor = count - 1;
      char romHex[17]{};
      dsRomToHex(g_dsRoms[g_dsCursor], romHex);
      const String selKey = prefKeyDsSel(romHex);
      g_prefs.begin("nettemp", false);
      const bool current = g_prefs.getBool(selKey.c_str(), true);
      g_prefs.putBool(selKey.c_str(), !current);
      g_prefs.end();
      g_viewNeedsRedraw = true;
    }
    if (keyChar == 'e' || keyChar == 'E') {
      g_cfg.dsEnabled = !g_cfg.dsEnabled;
      dsEnsureBus();
      if (g_cfg.dsEnabled) dsRescan();
      prefsSave();
      g_viewNeedsRedraw = true;
    } else if (keyChar == 'p' || keyChar == 'P') {
      int newPin = editGpioPin("DS18B20 Pin", g_cfg.dsPin);
      g_cfg.dsPin = newPin;
      dsEnsureBus();
      if (g_cfg.dsEnabled) dsRescan();
      prefsSave();
      g_viewNeedsRedraw = true;
      g_viewNeedsClear = true;
    } else if (keyChar == 'r' || keyChar == 'R') {
      if (g_cfg.dsEnabled) dsRescan();
      g_viewNeedsRedraw = true;
    }
    return;
  }

  if (g_view == View::GpioHcsr04) {
    if (isSpace) {
      g_cfg.hcsr04Enabled = !g_cfg.hcsr04Enabled;
      prefsSave();
      g_viewNeedsRedraw = true;
    }
    if (keyChar == 's' || keyChar == 'S') {
      g_cfg.hcsr04Selected = !g_cfg.hcsr04Selected;
      prefsSave();
      g_viewNeedsRedraw = true;
    }
    if (keyChar == 't' || keyChar == 'T') {
      int newPin = editGpioPin("HC-SR04 Trigger Pin", g_cfg.hcsr04TrigPin);
      g_cfg.hcsr04TrigPin = newPin;
      prefsSave();
      g_viewNeedsRedraw = true;
      g_viewNeedsClear = true;
    } else if (keyChar == 'e' || keyChar == 'E') {
      int newPin = editGpioPin("HC-SR04 Echo Pin", g_cfg.hcsr04EchoPin);
      g_cfg.hcsr04EchoPin = newPin;
      prefsSave();
      g_viewNeedsRedraw = true;
      g_viewNeedsClear = true;
    }
    return;
  }

  if (g_view == View::GpioDht) {
    if (isSpace) {
      g_cfg.dhtEnabled = !g_cfg.dhtEnabled;
      prefsSave();
      g_viewNeedsRedraw = true;
    }
    if (keyChar == 's' || keyChar == 'S') {
      g_cfg.dhtSelected = !g_cfg.dhtSelected;
      prefsSave();
      g_viewNeedsRedraw = true;
    }
    if (keyChar == 'p' || keyChar == 'P') {
      int newPin = editGpioPin("DHT Pin", g_cfg.dhtPin);
      g_cfg.dhtPin = newPin;
      prefsSave();
      g_viewNeedsRedraw = true;
      g_viewNeedsClear = true;
    } else if (keyChar == 'y' || keyChar == 'Y') {
      g_cfg.dhtType = (g_cfg.dhtType == 11) ? 22 : 11;
      prefsSave();
      g_viewNeedsRedraw = true;
    }
    return;
  }

  if (g_view == View::ResetConfirm) {
    if (isEnter || keyChar == 'y' || keyChar == 'Y') {
      g_prefs.begin("nettemp", false);
      g_prefs.clear();
      g_prefs.end();
      ESP.restart();
    }
    if (keyChar == 'n' || keyChar == 'N') {
      g_view = View::MainMenu;
    }
    return;
  }

  if (g_view == View::WifiSetup) {
    if (isSpace) {
#if NETTEMP_ENABLE_PORTAL
      if (g_portalRunning) portalStop();
#endif
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      return;
    }
    if (isEnter) wifiScanAndSelect();
    return;
  }

  if (g_view == View::MqttSetup) {
    if (isSpace) {
      g_cfg.mqttEnabled = !g_cfg.mqttEnabled;
      prefsSave();
      return;
    }
    if (isEnter) editMqttConfig();
    return;
  }

  if (g_view == View::ServerSetup) {
    if (isSpace) {
      g_cfg.serverEnabled = !g_cfg.serverEnabled;
      prefsSave();
      return;
    }
    if (isEnter) editServerConfig();
    return;
  }
}
#endif

#if NETTEMP_CARDPUTER_UI
static void uiSetup() {
  auto cfg = M5.config();
  cfg.clear_display = true;
  cfg.output_power = true;
  cfg.internal_imu = true;
  cfg.internal_rtc = true;
  cfg.internal_mic = false;  // Disable mic to save resources
  M5Cardputer.begin(cfg, true);  // true = init display
  M5Cardputer.Display.setRotation(1);
  // Use a single font and size across UI for consistency.
  M5Cardputer.Display.setFont(&fonts::Font2);
  M5Cardputer.Display.setTextSize(UI_TEXT_SIZE);
  M5Cardputer.Display.fillScreen(COLOR_BG);
  g_lastUiInputMs = millis();
  g_displaySleeping = false;
}

static void uiLoopTick() {
  const uint32_t now = millis();
  if (now - g_lastUiMs < UI_REFRESH_MS) return;
  g_lastUiMs = now;

  if (g_wifiEnablePending && (wifiConnected() || (now - g_wifiEnableMs) > 20000UL)) {
    g_wifiEnablePending = false;
    g_viewNeedsRedraw = true;
  }

  if (!kDisableUiSleep) {
    if (!g_displaySleeping && g_lastUiInputMs != 0 && (now - g_lastUiInputMs) >= 30000UL) {
      M5Cardputer.Display.sleep();
      g_displaySleeping = true;
      return;
    }
    if (g_displaySleeping) return;
  }

  if (g_view == View::ShowSensors && now - g_showSensorsLastMs >= 2000) {
    g_viewNeedsRedraw = true;
  }

  // Auto-update BLE scan view every 2 seconds to show new sensors
  if (g_view == View::ScanBle) {
    static uint32_t s_lastBleScanRefresh = 0;
    if (now - s_lastBleScanRefresh >= 2000) {
      s_lastBleScanRefresh = now;
      g_viewNeedsRedraw = true;
    }
  }

  // Check if view changed - flag for full redraw
  if (g_view != g_prevView) {
    g_viewNeedsRedraw = true;
    g_viewNeedsClear = true;
    if (g_view == View::ScanBle) {
#if NETTEMP_ENABLE_BLE
      g_bleLastScanStartMs = 0;
      bleTriggerManualScan();
#endif
    }
    g_prevView = g_view;
  }

  // Only redraw if something changed
  if (!g_viewNeedsRedraw) return;

  // Clear screen only on full view change to avoid flicker
  if (g_viewNeedsClear) {
    M5Cardputer.Display.fillScreen(COLOR_BG);
    g_viewNeedsClear = false;
  }

  // Redraw view
  switch (g_view) {
    case View::MainMenu: viewMainMenu(); break;
    case View::ShowSensors: viewShowSensors(); break;
    case View::ScanBle: viewScanBle(); break;
    case View::ScanI2c: viewScanI2c(); break;
    case View::GpioDevices: viewGpioDevices(); break;
    case View::GpioDs18b20: viewGpioDs18b20(); break;
    case View::GpioHcsr04: viewGpioHcsr04(); break;
    case View::GpioDht: viewGpioDht(); break;
    case View::WifiSetup: viewWifiSetup(); break;
    case View::MqttSetup: viewMqttSetup(); break;
    case View::ServerSetup: viewServerSetup(); break;
    case View::PairQr: viewPairQr(); break;
    case View::About: viewAbout(); break;
    case View::Channels: viewChannels(); break;
    case View::ResetConfirm: viewResetConfirm(); break;
    default: break;
  }

  // Draw blue frame around screen
  drawFrame();

  // Clear redraw flag after drawing
  g_viewNeedsRedraw = false;
}
#endif
