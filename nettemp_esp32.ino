#ifndef NETTEMP_HEADLESS
// Default behavior:
// - on M5Stack Cardputer board builds -> UI mode (needs M5Cardputer library)
// - on other ESP32 boards            -> headless mode (Serial-only)
#if defined(ARDUINO_M5STACK_CARDPUTER) || defined(ARDUINO_M5STACK_Cardputer) || defined(M5CARDPUTER)
#define NETTEMP_HEADLESS 0
#else
#define NETTEMP_HEADLESS 1
#endif
#endif

#ifndef NETTEMP_SERIAL_BAUD
#define NETTEMP_SERIAL_BAUD 115200
#endif

#ifndef NETTEMP_ENABLE_MQTT
#define NETTEMP_ENABLE_MQTT 1
#endif

#ifndef NETTEMP_ENABLE_SERVER
#define NETTEMP_ENABLE_SERVER 1
#endif

#ifndef NETTEMP_ENABLE_I2C
#define NETTEMP_ENABLE_I2C 1
#endif

#ifndef NETTEMP_ENABLE_PORTAL
#define NETTEMP_ENABLE_PORTAL 1
#endif

#ifndef NETTEMP_BLE_PARSE_MFG
// Manufacturer-data parsing is intentionally off by default because it can produce false-positives
// (random devices whose payload accidentally matches the ATC frame shape).
#define NETTEMP_BLE_PARSE_MFG 0
#endif

#if !NETTEMP_HEADLESS
#include <M5Cardputer.h>
#endif
#if NETTEMP_ENABLE_SERVER
#include <HTTPClient.h>
#endif
#if NETTEMP_ENABLE_PORTAL
#include <Update.h>
#endif

#include <Preferences.h>
#include <WiFi.h>
#include "nettemp_core.inc"



static void mqttEnsureConnected() {
#if !NETTEMP_ENABLE_MQTT
  return;
#else
  if (!g_cfg.mqttEnabled) return;
  if (!wifiConnected()) return;

  g_mqtt.setServer(g_cfg.mqttHost.c_str(), g_cfg.mqttPort);

  if (g_mqtt.connected()) return;

  String clientId = "nettemp_esp32-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  if (g_cfg.mqttUser.length() || g_cfg.mqttPass.length()) {
    g_mqtt.connect(clientId.c_str(), g_cfg.mqttUser.c_str(), g_cfg.mqttPass.c_str());
  } else {
    g_mqtt.connect(clientId.c_str());
  }
#endif
}

#if !NETTEMP_HEADLESS
static String keyboardReadWord() {
  if (!M5Cardputer.Keyboard.isChange()) return "";
  if (!M5Cardputer.Keyboard.isPressed()) return "";
  auto state = M5Cardputer.Keyboard.keysState();
  if (!state.word.empty()) return String(state.word.c_str());
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
  return state.esc;
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
#else
static String keyboardReadWord() { return ""; }
static bool keyboardPressedEnter() { return false; }
static bool keyboardPressedEsc() { return false; }
static bool keyboardPressedDel() { return false; }
static bool keyboardPressedSpace() { return false; }
#endif

static String macNoColonsUpper(const String& mac) {
  String out;
  out.reserve(mac.length());
  for (size_t i = 0; i < mac.length(); i++) {
    const char c = mac[i];
    if (c == ':') continue;
    out += (char)toupper((unsigned char)c);
  }
  return out;
}

static String macWithColonsUpper(const String& mac) {
  String out;
  out.reserve(mac.length());
  for (size_t i = 0; i < mac.length(); i++) {
    const char c = mac[i];
    out += (char)toupper((unsigned char)c);
  }
  return out;
}

static String normalizeMacToColonsUpper(String mac) {
  mac.trim();
  if (mac.length() == 12) {
    String out;
    out.reserve(17);
    for (int i = 0; i < 12; i += 2) {
      out += (char)toupper((unsigned char)mac[i]);
      out += (char)toupper((unsigned char)mac[i + 1]);
      if (i != 10) out += ':';
    }
    return out;
  }
  return macWithColonsUpper(mac);
}

static String formatMacFromBytesReversedUpper(const uint8_t mac[6]) {
  static const char* kHex = "0123456789ABCDEF";
  String out;
  out.reserve(17);
  for (int i = 5; i >= 0; i--) {
    const uint8_t b = mac[i];
    out += kHex[(b >> 4) & 0x0F];
    out += kHex[b & 0x0F];
    if (i != 0) out += ':';
  }
  return out;
}

static String formatMacFromBytesForwardUpper(const uint8_t mac[6]) {
  static const char* kHex = "0123456789ABCDEF";
  String out;
  out.reserve(17);
  for (int i = 0; i < 6; i++) {
    const uint8_t b = mac[i];
    out += kHex[(b >> 4) & 0x0F];
    out += kHex[b & 0x0F];
    if (i != 5) out += ':';
  }
  return out;
}

static String bytesToHexUpper(const std::string& bytes) {
  static const char* kHex = "0123456789ABCDEF";
  String out;
  out.reserve(bytes.size() * 2);
  for (size_t i = 0; i < bytes.size(); i++) {
    const uint8_t b = (uint8_t)bytes[i];
    out += kHex[(b >> 4) & 0x0F];
    out += kHex[b & 0x0F];
  }
  return out;
}

static String buildBtToMqttJson(const SensorRow& s) {
  // Match common BTtoMQTT style payloads (Theengs/OpenMQTTGateway-like).
  // Include raw advertising bytes as hex so the payload is fully reproducible.
  String json;
  json.reserve(256);
  const String id = macNoColonsUpper(s.mac);

  json += "{";
  json += "\"id\":\"" + id + "\"";
  json += ",\"mac\":\"" + s.mac + "\"";
  if (!isnan(s.temperatureC)) json += ",\"tempc\":" + String(s.temperatureC, 2);
  if (!isnan(s.temperatureC)) json += ",\"tempf\":" + String((s.temperatureC * 9.0f / 5.0f) + 32.0f, 2);
  if (!isnan(s.humidityPct)) json += ",\"hum\":" + String(s.humidityPct, 1);
  if (s.batteryPct >= 0) json += ",\"batt\":" + String(s.batteryPct);
  if (s.counter >= 0) json += ",\"cnt\":" + String(s.counter);
  if (s.flags >= 0) json += ",\"flags\":" + String(s.flags);
  json += ",\"rssi\":" + String(s.rssi);
  if (s.lastAdvHex.length()) json += ",\"adv\":\"" + s.lastAdvHex + "\"";
  if (s.lastAdvSrc.length()) json += ",\"adv_src\":\"" + s.lastAdvSrc + "\"";
  json += "}";
  return json;
}

static String buildBtToMqttJsonMinimal(const SensorRow& s) {
  // Minimal payload to resemble TheengsGateway fields.
  String json;
  json.reserve(160);
  bool first = true;

  auto addKey = [&](const char* k) {
    if (!first) json += ",";
    first = false;
    json += "\"";
    json += k;
    json += "\":";
  };

  json += "{";
  if (!isnan(s.temperatureC)) {
    addKey("tempc");
    json += String(s.temperatureC, 2);
    addKey("tempf");
    json += String((s.temperatureC * 9.0f / 5.0f) + 32.0f, 2);
  }
  if (!isnan(s.humidityPct)) {
    addKey("hum");
    json += String(s.humidityPct, 1);
  }
  if (s.batteryPct >= 0) {
    addKey("batt");
    json += String(s.batteryPct);
  }
  json += "}";
  return json;
}

#if !NETTEMP_HEADLESS
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
      M5Cardputer.Display.printf("Enter=OK  Esc=Back  Del=Backspace\n\n");

      M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
      const String shown = secret ? String('*', buf.length()) : buf;
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
static bool editText(const char* title, String& value, bool secret = false) {
  (void)title;
  (void)value;
  (void)secret;
  return false;
}

static void showSplash() {
  // No display in headless mode.
}
#endif

static NimBLEScan* g_scan = nullptr;

static void processAdvertisedDevice(const NimBLEAdvertisedDevice& device) {
  const String advMac = device.getAddress().toString().c_str();
  const String advMacCanon = macWithColonsUpper(advMac);
  const int rssi = device.getRSSI();
  const uint32_t seenMs = millis();

  LYWSD03Reading reading{};
  bool ok = false;
  std::string raw;
  const char* src = "";
  String uuidStr;
  if (device.haveServiceData()) {
    // Prefer the service-data entry whose UUID is 0x181A (ATC/PVVX).
    const uint8_t n = device.getServiceDataCount();
    uint8_t idx = 0xFF;
    for (uint8_t i = 0; i < n; i++) {
      const auto uuid = device.getServiceDataUUID(i);
      if (uuid != NimBLEUUID((uint16_t)0x181A)) continue;
      idx = i;
      uuidStr = uuid.toString().c_str();
      break;
    }
    // Strict: only decode if we actually have 0x181A service data. Don't guess other UUIDs.
    if (idx != 0xFF) {
      raw = device.getServiceData(idx);
      ok = parseLywsd03FromServiceData(raw, reading);
      if (ok) src = "service";
    }
  }
  if (!ok && g_bleParseMfg && device.haveManufacturerData()) {
    raw = device.getManufacturerData();
    ok = parseLywsd03FromManufacturerData(raw, reading);
    if (ok) {
      // Strict check to avoid false positives: embedded MAC (ATC/PVVX) must match advertising address.
      if (!reading.has_mac) {
        ok = false;
      } else {
        const String embeddedRev = formatMacFromBytesReversedUpper(reading.mac);
        const String embeddedFwd = formatMacFromBytesForwardUpper(reading.mac);
        if (embeddedRev != advMacCanon && embeddedFwd != advMacCanon) ok = false;
      }
    }
    if (ok) src = "mfg";
  }
  if (ok) {
    String canonicalMac = advMac;
    if (reading.has_mac) {
      // ATC/PVVX variants embed MAC either reversed or forward; pick the one matching the adv address.
      const String embeddedRev = formatMacFromBytesReversedUpper(reading.mac);
      const String embeddedFwd = formatMacFromBytesForwardUpper(reading.mac);
      if (embeddedFwd == advMacCanon) canonicalMac = embeddedFwd;
      else if (embeddedRev == advMacCanon) canonicalMac = embeddedRev;
      else canonicalMac = advMac;
    }

    auto* row = upsertSensor(canonicalMac);
    row->rssi = rssi;
    row->lastSeenMs = seenMs;
    row->lastAdvHex = bytesToHexUpper(raw);
    row->lastAdvSrc = src;
    row->lastAdvUuid = uuidStr;
    row->temperatureC = reading.temperature_c;
    row->humidityPct = reading.humidity_pct;
    row->batteryPct = reading.battery_pct;
    row->voltageMv = reading.voltage_mv;
    row->counter = reading.counter;
    row->flags = reading.flags;
#if NETTEMP_HEADLESS
    {
      const String macNo = macNoColonsUpper(row->mac);
      bool manualSelected = false;
      if (bleManualGetSelected(macNo, manualSelected)) {
        row->selectionLocked = true;
        row->selected = manualSelected;
      } else {
        row->selectionLocked = false;
        if (g_headlessAutoSelect) row->selected = true;
      }
    }
#endif
  } else {
    // Track non-decoded devices only in UI mode, or temporarily during headless `ble scan`
    // so the user can see that scanning works even if decoding fails.
#if !NETTEMP_HEADLESS
    const bool collectUnknown = true;
#else
    const bool collectUnknown = g_headlessBleScanCollectUnknown;
#endif
    if (collectUnknown) {
      auto* row = upsertSensor(advMac);
      row->rssi = rssi;
      row->lastSeenMs = seenMs;
      if (device.haveServiceData()) {
        // Show first service-data blob (and its UUID) to aid debugging.
        std::string s;
        NimBLEUUID u;
        const uint8_t n = device.getServiceDataCount();
        if (n > 0) {
          u = device.getServiceDataUUID(0);
          s = device.getServiceData(0);
        } else {
          s = device.getServiceData();
        }
        row->lastAdvHex = s.empty() ? "" : bytesToHexUpper(s);
        row->lastAdvSrc = "service";
        row->lastAdvUuid = n > 0 ? String(u.toString().c_str()) : "";
      } else if (device.haveManufacturerData()) {
        const std::string m = device.getManufacturerData();
        row->lastAdvHex = m.empty() ? "" : bytesToHexUpper(m);
        row->lastAdvSrc = "mfg";
        row->lastAdvUuid = "";
      } else {
        row->lastAdvHex = "";
        row->lastAdvSrc = "";
        row->lastAdvUuid = "";
      }
    }
  }
}

class NettempScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
    if (!advertisedDevice) return;
    processAdvertisedDevice(*advertisedDevice);
  }
};

static NettempScanCallbacks g_scanCallbacks;

static void bleStartScanFor(uint32_t durationMs, bool keepExistingResults) {
  if (!g_scan) return;
  g_scan->stop();
  g_scan->clearResults();
  if (keepExistingResults) {
    // We still clear controller-side results, but keep our own decoded map.
  } else {
    g_sensors.clear();
  }

  g_scan->setScanCallbacks(&g_scanCallbacks, /*wantDuplicates=*/true);
  g_scan->setActiveScan(g_activeScan);
  g_scan->setInterval(90);
  g_scan->setWindow(60);
  // Disable the controller-side max-results limit for:
  // - continuous scans (duration=0), because random nearby devices could fill the limit before our target sensors are seen
  // - short, manual scans, because we don't want to miss devices in crowded RF environments
  // NimBLE will still deliver callbacks and then erase per-device results (m_maxResults==0 behavior).
  const bool noLimit = (durationMs == 0) || (durationMs > 0 && !keepExistingResults);
  g_scan->setMaxResults(noLimit ? 0 : g_bleMaxResults);
  // Note: `isContinue` in NimBLEScan::start() is NOT about "keep our decoded results".
  // For reliability across NimBLE-Arduino versions, always start a fresh scan and manage
  // our own result storage (`g_sensors`) via keepExistingResults above.
  g_scan->start(/*durationMs=*/durationMs, /*isContinue=*/false, /*restart=*/true);
}

static void bleConfigureScan() {
  // duration=0 -> scan forever
  bleStartScanFor(/*durationMs=*/0, /*keepExistingResults=*/true);
}

static void bleEnsureAutoScan() {
#if NETTEMP_HEADLESS
  if (!g_headlessBleAutoScan) return;
  if (g_headlessDiagnosticsScanActive) return;
#endif
  if (!g_scan) return;
  if (g_scan->isScanning()) return;
  bleConfigureScan();
}

static std::vector<size_t> buildBleSortedIndex() {
  std::vector<size_t> idx;
  idx.reserve(g_sensors.size());
  for (size_t i = 0; i < g_sensors.size(); i++) idx.push_back(i);
  std::sort(idx.begin(), idx.end(), [](size_t a, size_t b) { return g_sensors[a].lastSeenMs > g_sensors[b].lastSeenMs; });
  return idx;
}

static String serverBleFieldsEnabledString() {
  String out;
  auto add = [&](const char* s) {
    if (out.length()) out += " ";
    out += s;
  };
  if (g_srvBleFields & SRV_BLE_TEMPC) add("tempc");
  if (g_srvBleFields & SRV_BLE_TEMPF) add("tempf");
  if (g_srvBleFields & SRV_BLE_HUM) add("hum");
  if (g_srvBleFields & SRV_BLE_BATT) add("batt");
  if (g_srvBleFields & SRV_BLE_VOLT) add("volt");
  if (g_srvBleFields & SRV_BLE_RSSI) add("rssi");
  if (!out.length()) out = "(none)";
  return out;
}

static String serverBleFieldsAvailableString() {
  return String("tempc tempf hum batt volt rssi");
}

#if NETTEMP_ENABLE_I2C
static String serverI2cFieldsEnabledString() {
  String out;
  auto add = [&](const char* s) {
    if (out.length()) out += " ";
    out += s;
  };
  if (g_srvI2cFields & SRV_I2C_TEMPC) add("tempc");
  if (g_srvI2cFields & SRV_I2C_HUM) add("hum");
  if (g_srvI2cFields & SRV_I2C_PRESS) add("press_hpa");
  if (!out.length()) out = "(none)";
  return out;
}

static String serverI2cFieldsAvailableString() {
  return String("tempc hum press_hpa");
}
#else
static String serverI2cFieldsEnabledString() { return String("DISABLED"); }
static String serverI2cFieldsAvailableString() { return String("DISABLED"); }
#endif

#if NETTEMP_ENABLE_I2C
static void i2cApplySelectionToDetected() {
  if (!g_i2cSelDefined) return;
  for (auto& s : g_i2cSensors) {
    bool sel = false;
    for (const auto a : g_i2cSelAddrs) {
      if (a == s.address) {
        sel = true;
        break;
      }
    }
    s.selected = sel;
  }
}
#endif

static void dsEnsureBus() {
  if (!g_cfg.dsEnabled) {
    if (g_oneWire) {
      delete g_oneWire;
      g_oneWire = nullptr;
    }
    g_dsRoms.clear();
    g_dsTempsC.clear();
    g_dsConvertInProgress = false;
    return;
  }
  if (g_cfg.dsPin < 0) return;

  // Recreate bus if pin changed (or first init).
  static int s_lastPin = -999;
  if (!g_oneWire || s_lastPin != g_cfg.dsPin) {
    if (g_oneWire) delete g_oneWire;
    g_oneWire = new OneWire((uint8_t)g_cfg.dsPin);
    s_lastPin = g_cfg.dsPin;
    g_dsRoms.clear();
    g_dsTempsC.clear();
    g_dsConvertInProgress = false;
  }
}

static void dsRescan() {
  dsEnsureBus();
  g_dsRoms.clear();
  g_dsTempsC.clear();
  g_dsConvertInProgress = false;
  if (!g_cfg.dsEnabled) return;
  if (!g_oneWire) return;

  uint8_t addr[8]{};
  g_oneWire->reset_search();
  while (g_oneWire->search(addr)) {
    if (OneWire::crc8(addr, 7) != addr[7]) continue;
    // DS18B20 family=0x28, DS1822=0x22, DS18S20=0x10
    if (addr[0] != 0x28 && addr[0] != 0x22 && addr[0] != 0x10) continue;
    std::array<uint8_t, 8> rom{};
    for (int i = 0; i < 8; i++) rom[i] = addr[i];
    g_dsRoms.push_back(rom);
  }

  g_dsTempsC.assign(g_dsRoms.size(), NAN);
}

static void dsTick() {
  if (!g_cfg.dsEnabled) return;
  dsEnsureBus();
  if (!g_oneWire) return;

  const uint32_t nowMs = millis();
  if (g_dsRoms.empty()) {
    // Best-effort: rescan occasionally (e.g. first boot or after settings change).
    static uint32_t s_lastRescanMs = 0;
    if (s_lastRescanMs == 0 || (nowMs - s_lastRescanMs) > 10'000UL) {
      s_lastRescanMs = nowMs;
      dsRescan();
    } else {
      return;
    }
  }
  if (g_dsRoms.empty()) return;

  // Non-blocking conversion: start conversion, then read scratchpad after ~800ms.
  if (!g_dsConvertInProgress) {
    // Start conversion at most every 2 seconds.
    if (g_dsLastConvertStartMs != 0 && (nowMs - g_dsLastConvertStartMs) < 2000UL) return;
    if (!g_oneWire->reset()) return;
    g_oneWire->skip();
    g_oneWire->write(0x44, 1); // CONVERT T (parasite power on)
    g_dsConvertInProgress = true;
    g_dsLastConvertStartMs = nowMs;
    return;
  }

  if ((nowMs - g_dsLastConvertStartMs) < 800UL) return;

  // Read all sensors (blocking but small: 9 bytes each).
  for (size_t i = 0; i < g_dsRoms.size(); i++) {
    uint8_t data[9]{};
    if (!g_oneWire->reset()) continue;
    g_oneWire->select(g_dsRoms[i].data());
    g_oneWire->write(0xBE); // READ SCRATCHPAD
    for (int j = 0; j < 9; j++) data[j] = g_oneWire->read();
    if (OneWire::crc8(data, 8) != data[8]) continue;

    const int16_t raw = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
    const uint8_t family = g_dsRoms[i][0];
    float tempC = NAN;
    if (family == 0x28 || family == 0x22) {
      tempC = (float)raw / 16.0f;
    } else if (family == 0x10) {
      // DS18S20 (9-bit, 0.5°C steps)
      tempC = (float)raw / 2.0f;
    }
    g_dsTempsC[i] = tempC;
    g_dsLastSeenMs = nowMs;
  }

  g_dsConvertInProgress = false;
}

static bool dhtReadOnce(int gpio, int type, float& outTempC, float& outHumPct) {
  outTempC = NAN;
  outHumPct = NAN;
  if (gpio < 0) return false;
  if (type != 11 && type != 22) return false;

  uint8_t data[5]{};

  // Start signal
  pinMode(gpio, OUTPUT);
  digitalWrite(gpio, LOW);
  delay(20);
  digitalWrite(gpio, HIGH);
  delayMicroseconds(40);
  pinMode(gpio, INPUT_PULLUP);

  auto waitForLevel = [&](int level, uint32_t timeoutUs) -> uint32_t {
    const uint32_t start = micros();
    while ((int)digitalRead(gpio) != level) {
      if ((micros() - start) > timeoutUs) return 0;
    }
    return micros() - start;
  };

  // Sensor response: LOW ~80us, HIGH ~80us
  if (!waitForLevel(LOW, 200)) return false;
  if (!waitForLevel(HIGH, 200)) return false;
  if (!waitForLevel(LOW, 200)) return false;

  // Read 40 bits
  for (int i = 0; i < 40; i++) {
    // Each bit starts with ~50us LOW
    if (!waitForLevel(HIGH, 120)) return false;
    const uint32_t tHigh = waitForLevel(LOW, 140);
    if (!tHigh) return false;
    const int bit = (tHigh > 50) ? 1 : 0;
    data[i / 8] <<= 1;
    data[i / 8] |= (uint8_t)bit;
  }

  const uint8_t sum = (uint8_t)(data[0] + data[1] + data[2] + data[3]);
  if (sum != data[4]) return false;

  if (type == 11) {
    outHumPct = (float)data[0];
    outTempC = (float)data[2];
    return true;
  }

  const uint16_t rawHum = (uint16_t)data[0] << 8 | data[1];
  const uint16_t rawTemp = (uint16_t)data[2] << 8 | data[3];
  outHumPct = (float)rawHum / 10.0f;
  const bool neg = (rawTemp & 0x8000u) != 0;
  const uint16_t t = (uint16_t)(rawTemp & 0x7FFFu);
  outTempC = (float)t / 10.0f;
  if (neg) outTempC = -outTempC;
  return true;
}

static void dhtTick() {
  if (!g_cfg.dhtEnabled) return;
  const uint32_t nowMs = millis();
  if (g_dhtLastReadMs != 0 && (nowMs - g_dhtLastReadMs) < 2500UL) return;
  g_dhtLastReadMs = nowMs;

  float t = NAN, h = NAN;
  if (!dhtReadOnce(g_cfg.dhtPin, g_cfg.dhtType, t, h)) return;
  g_dhtTempC = t;
  g_dhtHumPct = h;
  g_dhtLastSeenMs = nowMs;
}

static void soilTick() {
  if (!g_cfg.soilEnabled) {
    g_soilRaw = -1;
    g_soilPct = NAN;
    g_soilLastSeenMs = 0;
    return;
  }
  if (g_cfg.soilAdcPin < 0) return;
  g_soilRaw = analogRead(g_cfg.soilAdcPin);
  if (g_cfg.soilDryRaw == g_cfg.soilWetRaw) {
    g_soilPct = NAN;
  } else {
    const int dry = g_cfg.soilDryRaw;
    const int wet = g_cfg.soilWetRaw;
    float pct = 0.0f;
    if (dry > wet) {
      pct = (float)(dry - g_soilRaw) * 100.0f / (float)(dry - wet);
    } else {
      pct = (float)(g_soilRaw - dry) * 100.0f / (float)(wet - dry);
    }
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    g_soilPct = pct;
  }
  g_soilLastSeenMs = millis();
}

#include "nettemp_power.inc"

static void blePurgeUndecoded() {
  // Keep only rows that have decoded sensor values.
  std::vector<SensorRow> kept;
  kept.reserve(g_sensors.size());
  for (const auto& s : g_sensors) {
    const bool hasValues = !isnan(s.temperatureC) || !isnan(s.humidityPct) || s.batteryPct >= 0;
    if (hasValues) kept.push_back(s);
  }
  g_sensors.swap(kept);
}

#if !NETTEMP_HEADLESS
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
    String("Scan: I2C addresses") + (NETTEMP_ENABLE_I2C ? "" : " (disabled)"),
    String("WiFi: ") + (wifiConnected() ? "connected" : "not connected"),
    String("Send: MQTT ") + (NETTEMP_ENABLE_MQTT ? (g_cfg.mqttEnabled ? "[on]" : "[off]") : "(disabled)"),
    String("Send: Server ") + (NETTEMP_ENABLE_SERVER ? (g_cfg.serverEnabled ? "[on]" : "[off]") : "(disabled)"),
    "Pair: QR import token",
    "About"
  };
  drawMenu("NETTEMP (Cardputer)", items, g_menuIndex);

  drawFooter("BtnA/BtnB=nav  Enter=select");
}

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
#endif
}

static void viewScanBle() {
  M5Cardputer.Display.fillScreen(COLOR_BG);
  drawHeader("BLE Scan (LYWSD03MMC)");
  M5Cardputer.Display.setCursor(0, 14);
  M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
  M5Cardputer.Display.printf("Mode: %s  Enter=toggle\n", g_activeScan ? "ACTIVE" : "PASSIVE");
  M5Cardputer.Display.printf("BtnA/BtnB=nav  Space=select  Esc=back\n");
  M5Cardputer.Display.drawFastHLine(0, 40, M5Cardputer.Display.width(), COLOR_MUTED);

  const auto idx = buildBleSortedIndex();
  if (!idx.empty()) {
    if (g_bleCursor < 0) g_bleCursor = 0;
    if (g_bleCursor >= (int)idx.size()) g_bleCursor = (int)idx.size() - 1;
  } else {
    g_bleCursor = 0;
  }

  const int startY = 44;
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
  drawFooter("Esc=back");
  return;
#else
  M5Cardputer.Display.fillScreen(COLOR_BG);
  drawHeader("I2C Sensors (BMP280/TMP102/SHT3x)");
  M5Cardputer.Display.setCursor(0, 14);
  M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
  M5Cardputer.Display.print("BtnA/BtnB=nav  Space=select  Enter=rescan  Esc=back\n");

  if (g_i2cSensors.empty()) {
    M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
    M5Cardputer.Display.setCursor(0, 34);
    M5Cardputer.Display.print("(no known I2C sensors found)");
    M5Cardputer.Display.setCursor(0, 48);
    M5Cardputer.Display.print("Try: BMP280 0x76/0x77, TMP102 0x48..0x4B, SHT3x 0x44/0x45");
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
#endif
}
#endif

static void wifiConnectIfConfigured() {
  if (wifiConnected()) return;
  if (g_cfg.wifiSsid.length() == 0) return;

  // Avoid calling WiFi.begin() repeatedly while already connecting (spams logs).
  if (WiFi.status() == WL_IDLE_STATUS) return;
  const uint32_t now = millis();
  if (now - g_lastWifiBeginMs < 10'000) return;
  g_lastWifiBeginMs = now;

  WiFi.mode(WIFI_STA);
  WiFi.begin(g_cfg.wifiSsid.c_str(), g_cfg.wifiPass.c_str());
}

#include "nettemp_headless.inc"


#if !NETTEMP_HEADLESS
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
    if (M5Cardputer.BtnA.wasPressed()) idx = (idx - 1 + n) % (n == 0 ? 1 : n);
    if (M5Cardputer.BtnB.wasPressed()) idx = (idx + 1) % (n == 0 ? 1 : n);

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
    M5Cardputer.Display.print("BtnA/BtnB=nav  Enter=select  Esc=back\n");
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

#if !NETTEMP_HEADLESS
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
  M5Cardputer.Display.printf("BLE scan: %s\n", g_activeScan ? "ACTIVE" : "PASSIVE");
#endif
}

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

static void viewAbout() {
  M5Cardputer.Display.fillScreen(COLOR_BG);
  drawHeader("About");
  M5Cardputer.Display.setCursor(0, 16);
  M5Cardputer.Display.setTextColor(COLOR_FG, COLOR_BG);
  M5Cardputer.Display.print("Nettemp Cardputer\n");
  M5Cardputer.Display.setTextColor(COLOR_MUTED, COLOR_BG);
  M5Cardputer.Display.print("BLE (ATC/PVVX) -> Screen/MQTT/Cloud\n\n");
  M5Cardputer.Display.printf("WiFi: %s\n", wifiConnected() ? WiFi.localIP().toString().c_str() : "(not connected)");
  M5Cardputer.Display.printf("BLE scan: %s\n", g_activeScan ? "ACTIVE" : "PASSIVE");
  drawFooter("Esc=back");
}
#endif

static void tickSendMqtt() {
#if !NETTEMP_ENABLE_MQTT
  return;
#else
#if NETTEMP_HEADLESS
  // In headless mode: publish only when background autoscan is enabled.
  // Diagnostic scans (`ble scan`) must never publish.
  if (!g_headlessBleAutoScan) return;
  if (g_headlessDiagnosticsScanActive) return;
#endif
  if (!g_cfg.mqttEnabled) return;
  if (!wifiConnected()) return;
  mqttEnsureConnected();
  if (!g_mqtt.connected()) return;

  auto publishNettemp = [&](const String& deviceId, const String& sensorId, const char* sensorType, const String& name,
                            const String& valueStr) {
    const String topic = "nettemp/" + deviceId + "/" + sensorId;
    String payload;
    payload.reserve(200);
    payload += "{";
    payload += "\"device_id\":\"" + deviceId + "\"";
    payload += ",\"sensor_id\":\"" + sensorId + "\"";
    payload += ",\"value\":" + valueStr;
    payload += ",\"sensor_type\":\"" + String(sensorType) + "\"";
    payload += ",\"name\":\"" + name + "\"";
    payload += "}";
    g_mqtt.publish(topic.c_str(), payload.c_str(), false);
  };

  if (g_cfg.bleSendMqtt) {
    for (auto& s : g_sensors) {
      if (!s.selected) continue;
      if (isnan(s.temperatureC) && isnan(s.humidityPct) && s.batteryPct < 0) continue;

      const uint32_t now = millis();
      // Allow the first publish immediately after boot/selection (lastMqttSentMs==0).
      if (g_cfg.mqttIntervalMs > 0 && s.lastMqttSentMs != 0 && now - s.lastMqttSentMs < g_cfg.mqttIntervalMs) continue;

      const String dev = macNoColonsUpper(s.mac);
      if (!isnan(s.temperatureC)) {
        publishNettemp(dev, dev + "_tempc", "temperature", "temperature", String(s.temperatureC, 2));
      }
      if (!isnan(s.humidityPct)) {
        publishNettemp(dev, dev + "_hum", "humidity", "humidity", String(s.humidityPct, 1));
      }
      if (s.batteryPct >= 0) {
        publishNettemp(dev, dev + "_batt", "battery", "battery", String(s.batteryPct));
      }

      s.lastMqttSentMs = now;
    }
  }

  // Local sensors (I2C / DS18B20 / DHT / VBAT): publish multiple messages (one per sensor_id).
  {
    const uint32_t now = millis();
    if (g_cfg.mqttIntervalMs == 0 || g_lastMqttLocalSentMs == 0 || (now - g_lastMqttLocalSentMs) >= g_cfg.mqttIntervalMs) {
      const String dev = g_cfg.deviceId.length() ? g_cfg.deviceId : String("nettemp_esp32");
      auto publishReading = [&](const String& sensorId, const char* sensorType, const String& name, const String& valueStr) {
        publishNettemp(dev, sensorId, sensorType, name, valueStr);
      };

#if NETTEMP_ENABLE_I2C
      if (g_cfg.i2cSendMqtt) {
        for (auto& s : g_i2cSensors) {
          if (!s.selected) continue;
          if (!s.reading.ok) continue;

          String addrHex = String(s.address, HEX);
          addrHex.toLowerCase();
          if (addrHex.length() == 1) addrHex = "0" + addrHex;
          const String typeName = String(i2cSensorTypeName(s.type));
          const String base = dev + "-i2c_0x" + addrHex + "_" + typeName;

          if (!isnan(s.reading.temperature_c)) {
            publishReading(base + "_tempc", "temperature", typeName + " temp", String(s.reading.temperature_c, 2));
          }
          if (!isnan(s.reading.humidity_pct)) {
            publishReading(base + "_hum", "humidity", typeName + " hum", String(s.reading.humidity_pct, 1));
          }
          if (!isnan(s.reading.pressure_hpa)) {
            publishReading(base + "_press_hpa", "pressure", typeName + " press", String(s.reading.pressure_hpa, 1));
          }

          s.last_mqtt_sent_ms = now;
        }
      }
#endif

      if (g_cfg.gpioSendMqtt) {
        if (g_cfg.dsEnabled && !g_dsRoms.empty()) {
          for (size_t i = 0; i < g_dsRoms.size(); i++) {
            if (i >= g_dsTempsC.size() || isnan(g_dsTempsC[i])) continue;
            char romHex[17]{};
            for (int b = 0; b < 8; b++) sprintf(romHex + b * 2, "%02X", g_dsRoms[i][b]);
            publishReading(dev + "-ds18b20_" + String(romHex) + "_tempc", "temperature", "DS18B20 temp", String(g_dsTempsC[i], 2));
          }
        }

        if (g_cfg.dhtEnabled && !isnan(g_dhtTempC)) {
          const String base = dev + "-dht" + String(g_cfg.dhtType) + "_gpio" + String(g_cfg.dhtPin);
          publishReading(base + "_tempc", "temperature", "DHT temp", String(g_dhtTempC, 2));
          if (!isnan(g_dhtHumPct)) publishReading(base + "_hum", "humidity", "DHT hum", String(g_dhtHumPct, 1));
        }

        if (g_cfg.vbatMode != 0 && g_vbatPct >= 0) {
          publishReading(dev + "-batt", "battery", "battery", String(g_vbatPct));
        }

        if (g_cfg.soilEnabled && g_soilRaw >= 0) {
          const String base = dev + "-soil_adc" + String(g_cfg.soilAdcPin);
          publishReading(base + "_raw", "soil_raw", "soil raw", String(g_soilRaw));
          if (!isnan(g_soilPct)) {
            publishReading(base + "_pct", "soil_moisture", "soil", String(g_soilPct, 1));
          }
        }
      }

      g_lastMqttLocalSentMs = now;
    }
  }

  g_mqtt.loop();
#endif
}

static void tickSendServer() {
#if !NETTEMP_ENABLE_SERVER
  return;
#else
  if (!g_cfg.serverEnabled) return;
  if (!wifiConnected()) return;
  if (g_cfg.serverApiKey.length() == 0) return;
  if (g_cfg.serverIntervalMs > 0 && g_lastServerSendMs != 0 && (millis() - g_lastServerSendMs) < g_cfg.serverIntervalMs) return;

  const uint32_t ts = (uint32_t)(time(nullptr) > 100000 ? time(nullptr) : (millis() / 1000));
  bool anySent = false;

  if (g_cfg.bleSendServer) {
    for (const auto& s : g_sensors) {
      if (!s.selected) continue;
    const String mac = s.mac;
    const String macNoColons = macNoColonsUpper(mac);

    NettempBatch batch;
    batch.deviceId = macNoColons;
    batch.apiKey = g_cfg.serverApiKey;
    batch.baseUrl = g_cfg.serverBaseUrl;

    if ((g_srvBleFields & SRV_BLE_TEMPC) && !isnan(s.temperatureC)) {
      batch.readings.push_back(NettempReading{
        .sensorId = macNoColons + "_tempc",
        .value = s.temperatureC,
        .sensorType = "temperature",
        .unit = "°C",
        .timestamp = ts,
        .friendlyName = "temperature",
      });
    }
    if ((g_srvBleFields & SRV_BLE_TEMPF) && !isnan(s.temperatureC)) {
      batch.readings.push_back(NettempReading{
        .sensorId = macNoColons + "_tempf",
        .value = (s.temperatureC * 9.0f / 5.0f) + 32.0f,
        .sensorType = "temperature_f",
        .unit = "",
        .timestamp = ts,
        .friendlyName = "temperature_f",
      });
    }
    if ((g_srvBleFields & SRV_BLE_HUM) && !isnan(s.humidityPct)) {
      batch.readings.push_back(NettempReading{
        .sensorId = macNoColons + "_hum",
        .value = s.humidityPct,
        .sensorType = "humidity",
        .unit = "%",
        .timestamp = ts,
        .friendlyName = "humidity",
      });
    }
    if ((g_srvBleFields & SRV_BLE_BATT) && s.batteryPct >= 0) {
      batch.readings.push_back(NettempReading{
        .sensorId = macNoColons + "_batt",
        .value = (float)s.batteryPct,
        .sensorType = "battery",
        .unit = "%",
        .timestamp = ts,
        .friendlyName = "battery",
      });
    }
    if ((g_srvBleFields & SRV_BLE_VOLT) && s.voltageMv >= 0) {
      batch.readings.push_back(NettempReading{
        .sensorId = macNoColons + "_volt",
        .value = (float)s.voltageMv / 1000.0f,
        .sensorType = "voltage",
        .unit = "",
        .timestamp = ts,
        .friendlyName = "volt",
      });
    }
    if ((g_srvBleFields & SRV_BLE_RSSI) && s.rssi != 0) {
      batch.readings.push_back(NettempReading{
        .sensorId = macNoColons + "_rssi",
        .value = (float)s.rssi,
        .sensorType = "rssi",
        .unit = "dBm",
        .timestamp = ts,
        .friendlyName = "rssi",
      });
    }

    if (!batch.readings.empty()) {
      nettempPostBatch(g_tlsClient, batch);
      anySent = true;
    }
    }
  }

  if (g_cfg.i2cSendServer) {
    const String i2cDeviceId = g_cfg.deviceId.length() ? g_cfg.deviceId : String("nettemp_esp32");
    for (const auto& s : g_i2cSensors) {
      if (!s.selected) continue;
      if (!s.reading.ok) continue;

      String addrHex = String(s.address, HEX);
      addrHex.toLowerCase();
      if (addrHex.length() == 1) addrHex = "0" + addrHex;
      const String typeName = String(i2cSensorTypeName(s.type));
      const String base = i2cDeviceId + "-i2c_" + typeName + "_0x" + addrHex;

      NettempBatch batch;
      batch.deviceId = i2cDeviceId;
      batch.apiKey = g_cfg.serverApiKey;
      batch.baseUrl = g_cfg.serverBaseUrl;

      if ((g_srvI2cFields & SRV_I2C_TEMPC) && !isnan(s.reading.temperature_c)) {
        batch.readings.push_back(NettempReading{
          .sensorId = base + "_temp",
          .value = s.reading.temperature_c,
          .sensorType = "temperature",
          .unit = "°C",
          .timestamp = ts,
          .friendlyName = typeName + " temp",
        });
      }
      if ((g_srvI2cFields & SRV_I2C_HUM) && !isnan(s.reading.humidity_pct)) {
        batch.readings.push_back(NettempReading{
          .sensorId = base + "_hum",
          .value = s.reading.humidity_pct,
          .sensorType = "humidity",
          .unit = "%",
          .timestamp = ts,
          .friendlyName = typeName + " hum",
        });
      }
      if ((g_srvI2cFields & SRV_I2C_PRESS) && !isnan(s.reading.pressure_hpa)) {
        batch.readings.push_back(NettempReading{
          .sensorId = base + "_press",
          .value = s.reading.pressure_hpa,
          .sensorType = "pressure",
          .unit = "hPa",
          .timestamp = ts,
          .friendlyName = typeName + " press",
        });
      }

      if (!batch.readings.empty()) {
        nettempPostBatch(g_tlsClient, batch);
        anySent = true;
      }
    }
  }

  // GPIO sensors (DS18B20 / DHT) are sent under the configured device_id (same as I2C).
  if (g_cfg.gpioSendServer) {
    const String gpioDeviceId = g_cfg.deviceId.length() ? g_cfg.deviceId : String("nettemp_esp32");
    NettempBatch batch;
    batch.deviceId = gpioDeviceId;
    batch.apiKey = g_cfg.serverApiKey;
    batch.baseUrl = g_cfg.serverBaseUrl;

    if (g_cfg.dsEnabled && !g_dsRoms.empty()) {
      for (size_t i = 0; i < g_dsRoms.size(); i++) {
        if (i >= g_dsTempsC.size() || isnan(g_dsTempsC[i])) continue;
        char romHex[17]{};
        for (int b = 0; b < 8; b++) sprintf(romHex + b * 2, "%02X", g_dsRoms[i][b]);
        const String base = gpioDeviceId + "-ds18b20_" + String(romHex);
        batch.readings.push_back(NettempReading{
          .sensorId = base + "_temp",
          .value = g_dsTempsC[i],
          .sensorType = "temperature",
          .unit = "°C",
          .timestamp = ts,
          .friendlyName = "DS18B20 temp",
        });
      }
    }

    if (g_cfg.dhtEnabled && !isnan(g_dhtTempC)) {
      const String base = gpioDeviceId + "-dht" + String(g_cfg.dhtType) + "_gpio" + String(g_cfg.dhtPin);
      batch.readings.push_back(NettempReading{
        .sensorId = base + "_temp",
        .value = g_dhtTempC,
        .sensorType = "temperature",
        .unit = "°C",
        .timestamp = ts,
        .friendlyName = "DHT temp",
      });
      if (!isnan(g_dhtHumPct)) {
        batch.readings.push_back(NettempReading{
          .sensorId = base + "_hum",
          .value = g_dhtHumPct,
          .sensorType = "humidity",
          .unit = "%",
          .timestamp = ts,
          .friendlyName = "DHT hum",
        });
      }
    }

    if (g_cfg.vbatMode != 0 && g_vbatPct >= 0) {
      batch.readings.push_back(NettempReading{
        .sensorId = gpioDeviceId + "_batt",
        .value = (float)g_vbatPct,
        .sensorType = "battery",
        .unit = "%",
        .timestamp = ts,
        .friendlyName = "battery",
      });
      if (g_cfg.vbatSendVolt && !isnan(g_vbatVolts)) {
        batch.readings.push_back(NettempReading{
          .sensorId = gpioDeviceId + "_vbat",
          .value = g_vbatVolts,
          .sensorType = "voltage",
          .unit = "V",
          .timestamp = ts,
          .friendlyName = "vbat",
        });
      }
    }

    if (g_cfg.soilEnabled && g_soilRaw >= 0) {
      const String base = gpioDeviceId + "-soil_adc" + String(g_cfg.soilAdcPin);
      if (g_cfg.soilSendRaw) {
        batch.readings.push_back(NettempReading{
          .sensorId = base + "_raw",
          .value = (float)g_soilRaw,
          .sensorType = "soil_raw",
          .unit = "",
          .timestamp = ts,
          .friendlyName = "soil raw",
        });
      }
      if (g_cfg.soilSendPct && !isnan(g_soilPct)) {
        batch.readings.push_back(NettempReading{
          .sensorId = base + "_pct",
          .value = g_soilPct,
          .sensorType = "soil_moisture",
          .unit = "%",
          .timestamp = ts,
          .friendlyName = "soil",
        });
      }
    }

    if (!batch.readings.empty()) {
      nettempPostBatch(g_tlsClient, batch);
      anySent = true;
    }
  }

  if (anySent) g_lastServerSendMs = millis();
#endif
}

#if NETTEMP_ENABLE_SERVER
static void webhookAppendEscaped(String& out, const String& s) {
  for (size_t i = 0; i < s.length(); i++) {
    const char c = s[i];
    if (c == '\\' || c == '"') {
      out += '\\';
      out += c;
    } else if (c == '\n') {
      out += "\\n";
    } else if (c == '\r') {
      out += "\\r";
    } else if (c == '\t') {
      out += "\\t";
    } else {
      out += c;
    }
  }
}

static String buildWebhookPayload(const NettempBatch& batch) {
  String json;
  json.reserve(512 + batch.readings.size() * 160);
  json += "{\"device_id\":\"";
  webhookAppendEscaped(json, batch.deviceId);
  json += "\",\"readings\":[";
  for (size_t i = 0; i < batch.readings.size(); i++) {
    const auto& r = batch.readings[i];
    if (i) json += ",";
    json += "{";
    json += "\"sensor_id\":\"";
    webhookAppendEscaped(json, r.sensorId);
    json += "\",\"value\":";
    json += String(r.value, 3);
    if (r.sensorType.length()) {
      json += ",\"sensor_type\":\"";
      webhookAppendEscaped(json, r.sensorType);
      json += "\"";
    }
    if (r.friendlyName.length()) {
      json += ",\"metadata\":{\"name\":\"";
      webhookAppendEscaped(json, r.friendlyName);
      json += "\"}";
    }
    json += "}";
  }
  json += "]}";
  return json;
}

static bool webhookPostJson(const String& url, const String& payload) {
  if (!url.length() || payload.length() == 0) return false;
  HTTPClient http;
  bool ok = false;
  if (url.startsWith("https://")) {
    ok = http.begin(g_tlsClient, url);
  } else if (url.startsWith("http://")) {
    ok = http.begin(g_wifiClient, url);
  } else {
    return false;
  }
  if (!ok) return false;
  http.addHeader("Content-Type", "application/json");
  const int code = http.POST((uint8_t*)payload.c_str(), payload.length());
  http.end();
  return code >= 200 && code < 300;
}

static void tickSendWebhook() {
  if (!g_cfg.webhookEnabled) return;
  if (!wifiConnected()) return;
  if (!g_cfg.webhookUrl.length()) return;
  if (g_cfg.webhookIntervalMs > 0 && g_lastWebhookSendMs != 0 &&
      (millis() - g_lastWebhookSendMs) < g_cfg.webhookIntervalMs) return;

  const uint32_t ts = (uint32_t)(time(nullptr) > 100000 ? time(nullptr) : (millis() / 1000));
  bool anySent = false;

  if (g_cfg.bleSendWebhook) {
    for (const auto& s : g_sensors) {
      if (!s.selected) continue;
    const String mac = s.mac;
    const String macNoColons = macNoColonsUpper(mac);
    NettempBatch batch;
    batch.deviceId = macNoColons;
    if ((g_srvBleFields & SRV_BLE_TEMPC) && !isnan(s.temperatureC)) {
      batch.readings.push_back(NettempReading{
        .sensorId = macNoColons + "_tempc",
        .value = s.temperatureC,
        .sensorType = "temperature",
        .unit = "°C",
        .timestamp = ts,
        .friendlyName = "temperature",
      });
    }
    if ((g_srvBleFields & SRV_BLE_TEMPF) && !isnan(s.temperatureC)) {
      batch.readings.push_back(NettempReading{
        .sensorId = macNoColons + "_tempf",
        .value = (s.temperatureC * 9.0f / 5.0f) + 32.0f,
        .sensorType = "temperature_f",
        .unit = "",
        .timestamp = ts,
        .friendlyName = "temperature_f",
      });
    }
    if ((g_srvBleFields & SRV_BLE_HUM) && !isnan(s.humidityPct)) {
      batch.readings.push_back(NettempReading{
        .sensorId = macNoColons + "_hum",
        .value = s.humidityPct,
        .sensorType = "humidity",
        .unit = "%",
        .timestamp = ts,
        .friendlyName = "humidity",
      });
    }
    if ((g_srvBleFields & SRV_BLE_BATT) && s.batteryPct >= 0) {
      batch.readings.push_back(NettempReading{
        .sensorId = macNoColons + "_batt",
        .value = (float)s.batteryPct,
        .sensorType = "battery",
        .unit = "%",
        .timestamp = ts,
        .friendlyName = "battery",
      });
    }
    if ((g_srvBleFields & SRV_BLE_VOLT) && s.voltageMv >= 0) {
      batch.readings.push_back(NettempReading{
        .sensorId = macNoColons + "_volt",
        .value = (float)s.voltageMv / 1000.0f,
        .sensorType = "voltage",
        .unit = "",
        .timestamp = ts,
        .friendlyName = "volt",
      });
    }
    if ((g_srvBleFields & SRV_BLE_RSSI) && s.rssi != 0) {
      batch.readings.push_back(NettempReading{
        .sensorId = macNoColons + "_rssi",
        .value = (float)s.rssi,
        .sensorType = "rssi",
        .unit = "dBm",
        .timestamp = ts,
        .friendlyName = "rssi",
      });
    }
    if (!batch.readings.empty()) {
      const String payload = buildWebhookPayload(batch);
      webhookPostJson(g_cfg.webhookUrl, payload);
      anySent = true;
    }
    }
  }

#if NETTEMP_ENABLE_I2C
  if (g_cfg.i2cSendWebhook) {
    const String i2cDeviceId = g_cfg.deviceId.length() ? g_cfg.deviceId : String("nettemp_esp32");
    for (const auto& s : g_i2cSensors) {
      if (!s.selected) continue;
      if (!s.reading.ok) continue;
      String addrHex = String(s.address, HEX);
      addrHex.toLowerCase();
      if (addrHex.length() == 1) addrHex = "0" + addrHex;
      const String typeName = String(i2cSensorTypeName(s.type));
      const String base = i2cDeviceId + "-i2c_" + typeName + "_0x" + addrHex;
      NettempBatch batch;
      batch.deviceId = i2cDeviceId;
      if ((g_srvI2cFields & SRV_I2C_TEMPC) && !isnan(s.reading.temperature_c)) {
        batch.readings.push_back(NettempReading{
          .sensorId = base + "_temp",
          .value = s.reading.temperature_c,
          .sensorType = "temperature",
          .unit = "°C",
          .timestamp = ts,
          .friendlyName = typeName + " temp",
        });
      }
      if ((g_srvI2cFields & SRV_I2C_HUM) && !isnan(s.reading.humidity_pct)) {
        batch.readings.push_back(NettempReading{
          .sensorId = base + "_hum",
          .value = s.reading.humidity_pct,
          .sensorType = "humidity",
          .unit = "%",
          .timestamp = ts,
          .friendlyName = typeName + " hum",
        });
      }
      if ((g_srvI2cFields & SRV_I2C_PRESS) && !isnan(s.reading.pressure_hpa)) {
        batch.readings.push_back(NettempReading{
          .sensorId = base + "_press",
          .value = s.reading.pressure_hpa,
          .sensorType = "pressure",
          .unit = "hPa",
          .timestamp = ts,
          .friendlyName = typeName + " press",
        });
      }
      if (!batch.readings.empty()) {
        const String payload = buildWebhookPayload(batch);
        webhookPostJson(g_cfg.webhookUrl, payload);
        anySent = true;
      }
    }
  }
#endif

  if (g_cfg.gpioSendWebhook) {
    const String deviceId = g_cfg.deviceId.length() ? g_cfg.deviceId : String("nettemp_esp32");
    NettempBatch batch;
    batch.deviceId = deviceId;
    if (g_cfg.dsEnabled && !g_dsRoms.empty()) {
      for (size_t i = 0; i < g_dsRoms.size(); i++) {
        if (i >= g_dsTempsC.size() || isnan(g_dsTempsC[i])) continue;
        char romHex[17]{};
        for (int b = 0; b < 8; b++) sprintf(romHex + b * 2, "%02X", g_dsRoms[i][b]);
        const String base = deviceId + "-ds18b20_" + String(romHex);
        batch.readings.push_back(NettempReading{
          .sensorId = base + "_temp",
          .value = g_dsTempsC[i],
          .sensorType = "temperature",
          .unit = "°C",
          .timestamp = ts,
          .friendlyName = "DS18B20 temp",
        });
      }
    }
    if (g_cfg.dhtEnabled && !isnan(g_dhtTempC)) {
      const String base = deviceId + "-dht" + String(g_cfg.dhtType) + "_gpio" + String(g_cfg.dhtPin);
      batch.readings.push_back(NettempReading{
        .sensorId = base + "_temp",
        .value = g_dhtTempC,
        .sensorType = "temperature",
        .unit = "°C",
        .timestamp = ts,
        .friendlyName = "DHT temp",
      });
      if (!isnan(g_dhtHumPct)) {
        batch.readings.push_back(NettempReading{
          .sensorId = base + "_hum",
          .value = g_dhtHumPct,
          .sensorType = "humidity",
          .unit = "%",
          .timestamp = ts,
          .friendlyName = "DHT hum",
        });
      }
    }
    if (g_cfg.vbatMode != 0 && g_vbatPct >= 0) {
      batch.readings.push_back(NettempReading{
        .sensorId = deviceId + "_batt",
        .value = (float)g_vbatPct,
        .sensorType = "battery",
        .unit = "%",
        .timestamp = ts,
        .friendlyName = "battery",
      });
      if (g_cfg.vbatSendVolt && !isnan(g_vbatVolts)) {
        batch.readings.push_back(NettempReading{
          .sensorId = deviceId + "_vbat",
          .value = g_vbatVolts,
          .sensorType = "voltage",
          .unit = "V",
          .timestamp = ts,
          .friendlyName = "vbat",
        });
      }
    }
    if (g_cfg.soilEnabled && g_soilRaw >= 0) {
      const String base = deviceId + "-soil_adc" + String(g_cfg.soilAdcPin);
      batch.readings.push_back(NettempReading{
        .sensorId = base + "_raw",
        .value = (float)g_soilRaw,
        .sensorType = "soil_raw",
        .unit = "",
        .timestamp = ts,
        .friendlyName = "soil raw",
      });
      if (!isnan(g_soilPct)) {
        batch.readings.push_back(NettempReading{
          .sensorId = base + "_pct",
          .value = g_soilPct,
          .sensorType = "soil_moisture",
          .unit = "%",
          .timestamp = ts,
          .friendlyName = "soil",
        });
      }
    }
    if (!batch.readings.empty()) {
      const String payload = buildWebhookPayload(batch);
      webhookPostJson(g_cfg.webhookUrl, payload);
      anySent = true;
    }
  }

  if (anySent) g_lastWebhookSendMs = millis();
}
#endif

#if !NETTEMP_HEADLESS
static void handleKeys() {
  if (keyboardPressedEsc()) {
    g_view = View::MainMenu;
    return;
  }

  if (g_view == View::MainMenu) {
    if (M5Cardputer.BtnA.wasPressed()) g_menuIndex = (g_menuIndex - 1 + 7) % 7;
    if (M5Cardputer.BtnB.wasPressed()) g_menuIndex = (g_menuIndex + 1) % 7;
    if (keyboardPressedEnter()) {
      switch (g_menuIndex) {
        case 0: g_view = View::ScanBle; break;
        case 1: g_view = View::ScanI2c; break;
        case 2: g_view = View::WifiSetup; break;
        case 3: g_view = View::MqttSetup; break;
        case 4: g_view = View::ServerSetup; break;
        case 5: g_view = View::PairQr; break;
        case 6: g_view = View::About; break;
        default: break;
      }
    }
    return;
  }

  if (g_view == View::ScanBle) {
    const int count = (int)buildBleSortedIndex().size();
    if (M5Cardputer.BtnA.wasPressed() && count > 0) g_bleCursor = (g_bleCursor - 1 + count) % count;
    if (M5Cardputer.BtnB.wasPressed() && count > 0) g_bleCursor = (g_bleCursor + 1) % count;
    if (keyboardPressedEnter()) {
      g_activeScan = !g_activeScan;
      prefsSave();
      bleConfigureScan();
      return;
    }
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
    if (M5Cardputer.BtnA.wasPressed() && count > 0) g_i2cCursor = (g_i2cCursor - 1 + count) % count;
    if (M5Cardputer.BtnB.wasPressed() && count > 0) g_i2cCursor = (g_i2cCursor + 1) % count;
    if (keyboardPressedSpace() && count > 0) {
      g_i2cSensors[g_i2cCursor].selected = !g_i2cSensors[g_i2cCursor].selected;
    }
    if (keyboardPressedEnter()) {
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

} // namespace

void setup() {
  Serial.begin(NETTEMP_SERIAL_BAUD);
  delay(1000);
  Serial.println(NETTEMP_HEADLESS ? "Nettemp ESP32 starting (HEADLESS)..." : "Nettemp Cardputer starting...");
  g_powerBootMs = millis();
  g_powerBootCycleDone = false;

#if !NETTEMP_HEADLESS
  auto cfg = M5.config();
  M5Cardputer.begin(cfg);
  M5Cardputer.Display.setRotation(1);
  M5Cardputer.Display.fillScreen(COLOR_BG);
#endif

  prefsLoad();
  showSplash();
  yield();

  WiFi.mode(WIFI_STA);
  wifiConnectIfConfigured();
  yield();
#if NETTEMP_ENABLE_PORTAL
  g_portalBootMs = millis();
  if (g_portalAuto && g_cfg.wifiSsid.length() == 0) {
    portalStart();
    yield();
  }
#endif

  i2cInitBus();
#if NETTEMP_ENABLE_I2C
  g_i2cSensors = i2cDetectKnownSensors(Wire);
  if (!g_i2cSensors.empty()) i2cUpdateReadings(Wire, g_i2cSensors);
#if NETTEMP_HEADLESS
  if (g_i2cSelDefined) {
    i2cApplySelectionToDetected();
  } else if (g_headlessAutoSelect) {
    for (auto& s : g_i2cSensors) s.selected = true;
  }
#else
  if (g_i2cSelDefined) {
    i2cApplySelectionToDetected();
  }
#endif
#endif

  dsEnsureBus();
  if (g_cfg.dsEnabled) dsRescan();
  vbatTick();
  yield();

#if NETTEMP_HEADLESS
  headlessPrintConfig();
#endif

  NimBLEDevice::init("nettemp-esp32");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  g_scan = NimBLEDevice::getScan();
#if NETTEMP_HEADLESS
  if (g_headlessBleAutoScan) bleConfigureScan();
#else
  bleConfigureScan();
#endif

  // Duty-cycle runs in loop() so portal/webserver can run too (useful for debugging / short wake windows).
}

void loop() {
#if !NETTEMP_HEADLESS
  M5Cardputer.update();
  handleKeys();
#else
  headlessSerialPoll();
  headlessTickLog();
#endif

  // Captive portal (first-time setup)
#if NETTEMP_ENABLE_PORTAL
  portalTick();
  webTickMaybeStartConfig();
  if (g_portalAuto && !g_portalRunning && !wifiConnected() && g_cfg.wifiSsid.length() > 0) {
    // If configured WiFi didn't connect shortly after boot, start portal as fallback.
    // In deep-sleep duty-cycle mode the device should conserve power; avoid starting portal automatically.
    if (!g_cfg.powerSleepEnabled && g_portalBootMs != 0 && (millis() - g_portalBootMs) > 20'000UL) {
      portalStart();
    }
  }
#endif

  // Deep sleep duty-cycle (runs on timer wakeups, and optionally after a boot grace window).
  powerSleepCycleMaybe();

  // Keep WiFi alive
  if (!wifiConnected()) wifiConnectIfConfigured();

  // Local GPIO sensors (DS18B20 / DHT)
  dsTick();
  dhtTick();
  soilTick();
  vbatTick();

  // Sending
  tickSendMqtt();
  tickSendServer();
#if NETTEMP_ENABLE_SERVER
  tickSendWebhook();
#endif

  // Ensure background BLE scan stays running in headless autoscan mode.
  const uint32_t nowMs = millis();
  if (nowMs - g_lastBleEnsureMs >= 2000) {
    g_lastBleEnsureMs = nowMs;
    bleEnsureAutoScan();
  }

#if NETTEMP_ENABLE_I2C
  if (nowMs - g_lastI2cPollMs >= 2000) {
    g_lastI2cPollMs = nowMs;
    if (!g_i2cSensors.empty()) i2cUpdateReadings(Wire, g_i2cSensors);
  }
#endif

#if !NETTEMP_HEADLESS
  // UI
  const uint32_t now = millis();
  if (now - g_lastUiMs >= UI_REFRESH_MS) {
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

  delay(10);
}
