#ifndef NETTEMP_CARDPUTER_UI
#if defined(ARDUINO_M5STACK_CARDPUTER) || defined(ARDUINO_M5STACK_Cardputer) || defined(M5CARDPUTER)
#define NETTEMP_CARDPUTER_UI 1
#else
#define NETTEMP_CARDPUTER_UI 0
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

#ifndef NETTEMP_ENABLE_OLED
#define NETTEMP_ENABLE_OLED 1
#endif

#ifndef NETTEMP_ENABLE_PORTAL
#define NETTEMP_ENABLE_PORTAL 1
#endif

#ifndef NETTEMP_BLE_PARSE_MFG
// Manufacturer-data parsing is intentionally off by default because it can produce false-positives
// (random devices whose payload accidentally matches the ATC frame shape).
#define NETTEMP_BLE_PARSE_MFG 0
#endif

#if NETTEMP_CARDPUTER_UI
#include <M5Cardputer.h>
#endif
#if NETTEMP_ENABLE_SERVER
#include <HTTPClient.h>
#endif
#if NETTEMP_ENABLE_PORTAL
#include <Update.h>
#endif
#include <esp_task_wdt.h>  // Hardware watchdog timer
#if NETTEMP_ENABLE_OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#endif

#include <Preferences.h>
#include <WiFi.h>
#include <vector>
#include <algorithm>
#include "nettemp_core.inc"
#include "nettemp_channels.inc"

#ifndef NETTEMP_SERIAL_LOG
#define NETTEMP_SERIAL_LOG 0
#endif

#if NETTEMP_SERIAL_LOG
#define LOG_PRINTF(...) LOG_PRINTF(__VA_ARGS__)
#define LOG_PRINTLN(...) LOG_PRINTLN(__VA_ARGS__)
#define LOG_PRINT(...) LOG_PRINT(__VA_ARGS__)
#else
#define LOG_PRINTF(...) do {} while (0)
#define LOG_PRINTLN(...) do {} while (0)
#define LOG_PRINT(...) do {} while (0)
#endif

#if NETTEMP_ENABLE_OLED
constexpr uint8_t OLED_I2C_ADDR = 0x3C;
constexpr int OLED_WIDTH = 128;
constexpr int OLED_HEIGHT = 64;
static Adafruit_SSD1306 g_oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
static bool g_oledOk = false;
static uint32_t g_oledLastUpdateMs = 0;
static uint32_t g_oledLastSwitchMs = 0;
static size_t g_oledEntryIndex = 0;
static uint32_t g_oledBootMs = 0;

struct OledEntry {
  String name;
  String lines[4];
  int lineCount = 0;
};

static void oledEntryAddLine(OledEntry& entry, const String& line) {
  if (entry.lineCount >= 4) return;
  entry.lines[entry.lineCount++] = line;
}

static bool oledPickBleTempc(float& outC) {
  const auto idx = buildBleSortedIndex();
  for (size_t k = 0; k < idx.size(); k++) {
    const auto& s = g_sensors[idx[k]];
    if (!s.selected) continue;
    if (!isnan(s.temperatureC)) {
      outC = s.temperatureC;
      return true;
    }
  }
  return false;
}

static bool oledPickBleHum(float& outH) {
  const auto idx = buildBleSortedIndex();
  for (size_t k = 0; k < idx.size(); k++) {
    const auto& s = g_sensors[idx[k]];
    if (!s.selected) continue;
    if (!isnan(s.humidityPct)) {
      outH = s.humidityPct;
      return true;
    }
  }
  return false;
}

static bool oledPickBleVolt(float& outV) {
  const auto idx = buildBleSortedIndex();
  for (size_t k = 0; k < idx.size(); k++) {
    const auto& s = g_sensors[idx[k]];
    if (!s.selected) continue;
    if (s.voltageMv >= 0) {
      outV = (float)s.voltageMv / 1000.0f;
      return true;
    }
  }
  return false;
}

static bool oledPickBleBatt(int& outPct) {
  const auto idx = buildBleSortedIndex();
  for (size_t k = 0; k < idx.size(); k++) {
    const auto& s = g_sensors[idx[k]];
    if (!s.selected) continue;
    if (s.batteryPct >= 0) {
      outPct = s.batteryPct;
      return true;
    }
  }
  return false;
}

static bool oledMatchBleMacNo(const String& macNo, const SensorRow& s) {
  return macNoColonsUpper(s.mac) == macNo;
}

static bool oledPickBleTempcByMac(const String& macNo, float& outC) {
  const auto idx = buildBleSortedIndex();
  for (size_t k = 0; k < idx.size(); k++) {
    const auto& s = g_sensors[idx[k]];
    if (!s.selected) continue;
    if (!oledMatchBleMacNo(macNo, s)) continue;
    if (!isnan(s.temperatureC)) {
      outC = s.temperatureC;
      return true;
    }
  }
  return false;
}

static bool oledPickBleHumByMac(const String& macNo, float& outH) {
  const auto idx = buildBleSortedIndex();
  for (size_t k = 0; k < idx.size(); k++) {
    const auto& s = g_sensors[idx[k]];
    if (!s.selected) continue;
    if (!oledMatchBleMacNo(macNo, s)) continue;
    if (!isnan(s.humidityPct)) {
      outH = s.humidityPct;
      return true;
    }
  }
  return false;
}

static bool oledPickBleVoltByMac(const String& macNo, float& outV) {
  const auto idx = buildBleSortedIndex();
  for (size_t k = 0; k < idx.size(); k++) {
    const auto& s = g_sensors[idx[k]];
    if (!s.selected) continue;
    if (!oledMatchBleMacNo(macNo, s)) continue;
    if (s.voltageMv >= 0) {
      outV = (float)s.voltageMv / 1000.0f;
      return true;
    }
  }
  return false;
}

static bool oledPickBleBattByMac(const String& macNo, int& outPct) {
  const auto idx = buildBleSortedIndex();
  for (size_t k = 0; k < idx.size(); k++) {
    const auto& s = g_sensors[idx[k]];
    if (!s.selected) continue;
    if (!oledMatchBleMacNo(macNo, s)) continue;
    if (s.batteryPct >= 0) {
      outPct = s.batteryPct;
      return true;
    }
  }
  return false;
}

static bool oledParseHexByte(const String& hex, uint8_t& out) {
  if (hex.length() == 0) return false;
  char* end = nullptr;
  const long v = strtol(hex.c_str(), &end, 16);
  if (end == hex.c_str()) return false;
  if (v < 0 || v > 255) return false;
  out = (uint8_t)v;
  return true;
}

static bool oledParseDsRomHex(const String& hex, std::array<uint8_t, 8>& out) {
  if (hex.length() != 16) return false;
  for (int i = 0; i < 8; i++) {
    const String part = hex.substring(i * 2, i * 2 + 2);
    uint8_t v = 0;
    if (!oledParseHexByte(part, v)) return false;
    out[i] = v;
  }
  return true;
}

static bool oledPickDsTempcByRom(const String& romHex, float& outC) {
  std::array<uint8_t, 8> rom{};
  if (!oledParseDsRomHex(romHex, rom)) return false;
  for (size_t i = 0; i < g_dsRoms.size(); i++) {
    if (g_dsRoms[i] != rom) continue;
    if (i >= g_dsTempsC.size()) return false;
    if (!isnan(g_dsTempsC[i])) {
      outC = g_dsTempsC[i];
      return true;
    }
  }
  return false;
}

static bool oledPickI2cTempcByAddr(const String& addrHex, float& outC) {
#if NETTEMP_ENABLE_I2C
  uint8_t addr = 0;
  if (!oledParseHexByte(addrHex, addr)) return false;
  for (const auto& s : g_i2cSensors) {
    if (!s.selected || !s.reading.ok) continue;
    if (s.address != addr) continue;
    if (!isnan(s.reading.temperature_c)) {
      outC = s.reading.temperature_c;
      return true;
    }
  }
#else
  (void)addrHex;
  (void)outC;
#endif
  return false;
}

static bool oledPickI2cHumByAddr(const String& addrHex, float& outH) {
#if NETTEMP_ENABLE_I2C
  uint8_t addr = 0;
  if (!oledParseHexByte(addrHex, addr)) return false;
  for (const auto& s : g_i2cSensors) {
    if (!s.selected || !s.reading.ok) continue;
    if (s.address != addr) continue;
    if (!isnan(s.reading.humidity_pct)) {
      outH = s.reading.humidity_pct;
      return true;
    }
  }
#else
  (void)addrHex;
  (void)outH;
#endif
  return false;
}

static bool oledPickI2cTempc(float& outC) {
#if NETTEMP_ENABLE_I2C
  for (const auto& s : g_i2cSensors) {
    if (!s.selected || !s.reading.ok) continue;
    if (!isnan(s.reading.temperature_c)) {
      outC = s.reading.temperature_c;
      return true;
    }
  }
#endif
  return false;
}

static bool oledPickI2cHum(float& outH) {
#if NETTEMP_ENABLE_I2C
  for (const auto& s : g_i2cSensors) {
    if (!s.selected || !s.reading.ok) continue;
    if (!isnan(s.reading.humidity_pct)) {
      outH = s.reading.humidity_pct;
      return true;
    }
  }
#endif
  return false;
}

static bool oledPickTempc(float& outC) {
  if (g_cfg.dhtEnabled && !isnan(g_dhtTempC)) {
    outC = g_dhtTempC;
    return true;
  }
  if (g_cfg.dsEnabled) {
    for (size_t i = 0; i < g_dsTempsC.size(); i++) {
      if (!isnan(g_dsTempsC[i])) {
        outC = g_dsTempsC[i];
        return true;
      }
    }
  }
  if (oledPickI2cTempc(outC)) return true;
  return oledPickBleTempc(outC);
}

static bool oledPickHum(float& outH) {
  if (g_cfg.dhtEnabled && !isnan(g_dhtHumPct)) {
    outH = g_dhtHumPct;
    return true;
  }
  if (oledPickI2cHum(outH)) return true;
  return oledPickBleHum(outH);
}

static bool oledPickVolt(float& outV) {
  if (g_cfg.vbatMode != 0 && !isnan(g_vbatVolts)) {
    outV = g_vbatVolts;
    return true;
  }
  return oledPickBleVolt(outV);
}

static bool oledPickBatt(int& outPct) {
  if (g_cfg.vbatMode != 0 && g_vbatPct >= 0) {
    outPct = g_vbatPct;
    return true;
  }
  return oledPickBleBatt(outPct);
}

static bool oledPickTempcFromSource(const String& src, float& outC) {
  if (src.length() == 0 || src == "auto") return oledPickTempc(outC);
  if (src == "dht") {
    if (g_cfg.dhtEnabled && !isnan(g_dhtTempC)) {
      outC = g_dhtTempC;
      return true;
    }
    return false;
  }
  if (src.startsWith("ds:")) {
    return oledPickDsTempcByRom(src.substring(3), outC);
  }
  if (src.startsWith("i2c:")) {
    return oledPickI2cTempcByAddr(src.substring(4), outC);
  }
  if (src.startsWith("ble:")) {
    return oledPickBleTempcByMac(src.substring(4), outC);
  }
  return false;
}

static bool oledPickHumFromSource(const String& src, float& outH) {
  if (src.length() == 0 || src == "auto") return oledPickHum(outH);
  if (src == "dht") {
    if (g_cfg.dhtEnabled && !isnan(g_dhtHumPct)) {
      outH = g_dhtHumPct;
      return true;
    }
    return false;
  }
  if (src.startsWith("i2c:")) {
    return oledPickI2cHumByAddr(src.substring(4), outH);
  }
  if (src.startsWith("ble:")) {
    return oledPickBleHumByMac(src.substring(4), outH);
  }
  return false;
}

static bool oledPickVoltFromSource(const String& src, float& outV) {
  if (src.length() == 0 || src == "auto") return oledPickVolt(outV);
  if (src == "vbat") {
    if (g_cfg.vbatMode != 0 && !isnan(g_vbatVolts)) {
      outV = g_vbatVolts;
      return true;
    }
    return false;
  }
  if (src.startsWith("ble:")) {
    return oledPickBleVoltByMac(src.substring(4), outV);
  }
  return false;
}

static bool oledPickBattFromSource(const String& src, int& outPct) {
  if (src.length() == 0 || src == "auto") return oledPickBatt(outPct);
  if (src == "vbat") {
    if (g_cfg.vbatMode != 0 && g_vbatPct >= 0) {
      outPct = g_vbatPct;
      return true;
    }
    return false;
  }
  if (src.startsWith("ble:")) {
    return oledPickBleBattByMac(src.substring(4), outPct);
  }
  return false;
}

static bool oledPickSoilRawFromSource(const String& src, int& outRaw) {
  if (src.length() == 0 || src == "auto" || src == "soil") {
    if (g_cfg.soilEnabled && g_soilRaw >= 0) {
      outRaw = g_soilRaw;
      return true;
    }
  }
  return false;
}

static bool oledPickSoilPctFromSource(const String& src, float& outPct) {
  if (src.length() == 0 || src == "auto" || src == "soil") {
    if (g_cfg.soilEnabled && !isnan(g_soilPct)) {
      outPct = g_soilPct;
      return true;
    }
  }
  return false;
}

static bool oledPickDistFromSource(const String& src, float& outCm) {
  if (src.length() == 0 || src == "auto" || src == "hcsr04") {
    if (g_cfg.hcsr04Enabled && !isnan(g_hcsr04Cm)) {
      outCm = g_hcsr04Cm;
      return true;
    }
  }
  return false;
}

static void oledInit() {
  g_oledOk = g_oled.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR);
  if (!g_oledOk) return;
  g_oledBootMs = millis();
  g_oled.clearDisplay();
  g_oled.setTextSize(g_cfg.oledTextSize);
  g_oled.setTextColor(SSD1306_WHITE);
  g_oled.setCursor(0, 0);
  g_oled.println(g_cfg.deviceId);
  g_oled.println("oled ready");
  g_oled.display();
}

static void oledTick() {
  if (!g_oledOk) return;
  if (!g_cfg.oledEnabled) return;
  const uint32_t nowMs = millis();
  if (nowMs - g_oledLastUpdateMs < 2000UL) return;
  g_oledLastUpdateMs = nowMs;
  g_oled.setTextSize(g_cfg.oledTextSize);
  if (g_oledBootMs != 0 && (nowMs - g_oledBootMs) < 3000UL) {
    g_oled.clearDisplay();
    g_oled.setCursor(0, 0);
    g_oled.println(g_cfg.deviceId);
    if (wifiConnected()) {
      g_oled.print("IP: ");
      g_oled.println(WiFi.localIP());
    } else {
      g_oled.println("WiFi: offline");
    }
    g_oled.display();
    return;
  }
  if (g_oledLastSwitchMs == 0) g_oledLastSwitchMs = nowMs;
  if (nowMs - g_oledLastSwitchMs >= 3000UL) {
    g_oledLastSwitchMs = nowMs;
    g_oledEntryIndex++;
  }

  std::vector<OledEntry> entries;
  g_prefs.begin("nettemp", true);
  auto prefBool = [&](const String& key) -> bool {
    return g_prefs.getBool(key.c_str(), false);
  };
  auto prefName = [&](const String& key, const String& fallback) -> String {
    const String v = g_prefs.getString(key.c_str(), "");
    return v.length() ? v : fallback;
  };
  auto prefDsName = [&](const String& romHex, const String& fallback) -> String {
    String v = g_prefs.getString(prefKeyDsName(romHex).c_str(), "");
    if (!v.length()) v = g_prefs.getString(prefKeyDsNameLegacy(romHex).c_str(), "");
    return v.length() ? v : fallback;
  };

  for (const auto& s : g_sensors) {
    const String macNo = macNoColonsUpper(s.mac);
    const String selKey = prefKeyBleOledSel(macNo);
    if (!prefBool(selKey)) continue;
    OledEntry entry;
    entry.name = prefName(prefKeyBleName(macNo), String("BLE ") + macWithColonsUpper(s.mac));
    if (prefBool(prefKeyBleOledTempc(macNo))) {
      oledEntryAddLine(entry, "T: " + (isnan(s.temperatureC) ? String("-") : String(s.temperatureC, 1)) + "C");
    }
    if (prefBool(prefKeyBleOledHum(macNo))) {
      oledEntryAddLine(entry, "H: " + (isnan(s.humidityPct) ? String("-") : String(s.humidityPct, 1)) + "%");
    }
    if (prefBool(prefKeyBleOledVolt(macNo))) {
      oledEntryAddLine(entry, "V: " + (s.voltageMv < 0 ? String("-") : String((float)s.voltageMv / 1000.0f, 2)));
    }
    if (prefBool(prefKeyBleOledBatt(macNo))) {
      oledEntryAddLine(entry, "B: " + (s.batteryPct < 0 ? String("-") : String(s.batteryPct)) + "%");
    }
    if (entry.lineCount == 0) oledEntryAddLine(entry, "No fields selected");
    entries.push_back(entry);
  }

#if NETTEMP_ENABLE_I2C
  for (const auto& s : g_i2cSensors) {
    String addrHex = (s.address < 16 ? String("0") : String("")) + String(s.address, HEX);
    addrHex.toLowerCase();
    const String selKey = prefKeyI2cOledSel(addrHex);
    if (!prefBool(selKey)) continue;
    OledEntry entry;
    entry.name = prefName(prefKeyI2cName(addrHex), prefName(String("i2cname_") + addrHex, String("I2C 0x") + addrHex + " " + i2cSensorTypeName(s.type)));
    if (prefBool(prefKeyI2cOledTempc(addrHex))) {
      oledEntryAddLine(entry, "T: " + (isnan(s.reading.temperature_c) ? String("-") : String(s.reading.temperature_c, 1)) + "C");
    }
    if (prefBool(prefKeyI2cOledHum(addrHex))) {
      oledEntryAddLine(entry, "H: " + (isnan(s.reading.humidity_pct) ? String("-") : String(s.reading.humidity_pct, 1)) + "%");
    }
    if (prefBool(prefKeyI2cOledPress(addrHex))) {
      oledEntryAddLine(entry, "P: " + (isnan(s.reading.pressure_hpa) ? String("-") : String(s.reading.pressure_hpa, 1)));
    }
    if (entry.lineCount == 0) oledEntryAddLine(entry, "No fields selected");
    entries.push_back(entry);
  }
#endif

  if (prefBool(PREF_OLED_DHT)) {
    OledEntry entry;
    entry.name = prefName("name_dht", String("DHT") + String(g_cfg.dhtType) + " GPIO" + String(g_cfg.dhtPin));
    if (prefBool(PREF_OLED_DHT_T)) {
      oledEntryAddLine(entry, "T: " + (isnan(g_dhtTempC) ? String("-") : String(g_dhtTempC, 1)) + "C");
    }
    if (prefBool(PREF_OLED_DHT_F)) {
      const float tempf = isnan(g_dhtTempC) ? NAN : (g_dhtTempC * 9.0f / 5.0f) + 32.0f;
      oledEntryAddLine(entry, "Tf: " + (isnan(tempf) ? String("-") : String(tempf, 1)) + "F");
    }
    if (prefBool(PREF_OLED_DHT_H)) {
      oledEntryAddLine(entry, "H: " + (isnan(g_dhtHumPct) ? String("-") : String(g_dhtHumPct, 1)) + "%");
    }
    if (entry.lineCount == 0) oledEntryAddLine(entry, "No fields selected");
    entries.push_back(entry);
  }

  for (size_t i = 0; i < g_dsRoms.size(); i++) {
    char romHex[17]{};
    for (int b = 0; b < 8; b++) sprintf(romHex + b * 2, "%02X", g_dsRoms[i][b]);
    const String selKey = prefKeyDsOledSel(romHex);
    if (!prefBool(selKey)) continue;
    OledEntry entry;
    entry.name = prefDsName(romHex, String("DS18B20 ") + romHex);
    const float tempc = (i < g_dsTempsC.size()) ? g_dsTempsC[i] : NAN;
    if (prefBool(prefKeyDsOledTempc(romHex))) {
      oledEntryAddLine(entry, "T: " + (isnan(tempc) ? String("-") : String(tempc, 1)) + "C");
    }
    if (prefBool(prefKeyDsOledTempf(romHex))) {
      const float tempf = isnan(tempc) ? NAN : (tempc * 9.0f / 5.0f) + 32.0f;
      oledEntryAddLine(entry, "Tf: " + (isnan(tempf) ? String("-") : String(tempf, 1)) + "F");
    }
    if (entry.lineCount == 0) oledEntryAddLine(entry, "No fields selected");
    entries.push_back(entry);
  }

  if (prefBool(PREF_OLED_SOIL)) {
    OledEntry entry;
    entry.name = prefName("name_soil", String("SOIL ADC") + String(g_cfg.soilAdcPin));
    if (prefBool(PREF_OLED_SOIL_RAW)) {
      oledEntryAddLine(entry, "Raw: " + (g_soilRaw < 0 ? String("-") : String(g_soilRaw)));
    }
    if (prefBool(PREF_OLED_SOIL_PCT)) {
      oledEntryAddLine(entry, "Pct: " + (isnan(g_soilPct) ? String("-") : String(g_soilPct, 1)) + "%");
    }
    if (entry.lineCount == 0) oledEntryAddLine(entry, "No fields selected");
    entries.push_back(entry);
  }

  if (prefBool(PREF_OLED_HC)) {
    OledEntry entry;
    entry.name = prefName("name_hcsr04", String("HC-SR04 GPIO") + String(g_cfg.hcsr04TrigPin) + "/" + String(g_cfg.hcsr04EchoPin));
    if (prefBool(PREF_OLED_HC_DIST)) {
      oledEntryAddLine(entry, "Dist: " + (isnan(g_hcsr04Cm) ? String("-") : String(g_hcsr04Cm, 1)) + "cm");
    }
    if (entry.lineCount == 0) oledEntryAddLine(entry, "No fields selected");
    entries.push_back(entry);
  }

  if (prefBool(PREF_OLED_VBAT)) {
    OledEntry entry;
    entry.name = prefName("name_vbat", "VBAT");
    if (prefBool(PREF_OLED_VBAT_V)) {
      oledEntryAddLine(entry, "V: " + (isnan(g_vbatVolts) ? String("-") : String(g_vbatVolts, 2)));
    }
    if (prefBool(PREF_OLED_VBAT_B)) {
      oledEntryAddLine(entry, "B: " + (g_vbatPct < 0 ? String("-") : String(g_vbatPct)) + "%");
    }
    if (entry.lineCount == 0) oledEntryAddLine(entry, "No fields selected");
    entries.push_back(entry);
  }
  g_prefs.end();

  g_oled.clearDisplay();
  g_oled.setCursor(0, 0);
  if (entries.empty()) {
    g_oled.println("Waiting for sensors");
  } else {
    if (g_oledEntryIndex >= entries.size()) g_oledEntryIndex = 0;
    const auto& entry = entries[g_oledEntryIndex];
    g_oled.println(entry.name);
    for (int i = 0; i < entry.lineCount; i++) {
      g_oled.println(entry.lines[i]);
    }
  }
  g_oled.display();
}
#endif



static uint32_t g_lastMqttSkipLogMs = 0;
static uint32_t g_lastServerSkipLogMs = 0;
static uint32_t g_lastLocalServerSkipLogMs = 0;
static uint32_t g_lastWebhookSkipLogMs = 0;

static void logSkipEvery(uint32_t& lastLogMs, const char* msg, uint32_t intervalMs = 5000) {
  const uint32_t now = millis();
  if (lastLogMs == 0 || (now - lastLogMs) >= intervalMs) {
    lastLogMs = now;
    LOG_PRINTLN(msg);
  }
}

static void mqttEnsureConnected() {
#if !NETTEMP_ENABLE_MQTT
  return;
#else
  if (!g_cfg.mqttEnabled) {
    logSkipEvery(g_lastMqttSkipLogMs, "MQTT: Disabled");
    return;
  }
  if (!wifiConnected()) {
    logSkipEvery(g_lastMqttSkipLogMs, "MQTT: WiFi not connected");
    return;
  }

  g_mqtt.setServer(g_cfg.mqttHost.c_str(), g_cfg.mqttPort);
  g_mqtt.setSocketTimeout(5);  // 5 second timeout to prevent indefinite hanging

  if (g_mqtt.connected()) return;

  // Rate limit connection attempts to avoid repeated blocking (try every 10 seconds max)
  static uint32_t lastConnectAttemptMs = 0;
  const uint32_t now = millis();
  if (now - lastConnectAttemptMs < 10000) return;
  lastConnectAttemptMs = now;

  LOG_PRINTF("MQTT: Connecting to %s:%u...\n", g_cfg.mqttHost.c_str(), g_cfg.mqttPort);
  String clientId = "nettemp_esp32-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  bool connected = false;
  if (g_cfg.mqttUser.length() || g_cfg.mqttPass.length()) {
    connected = g_mqtt.connect(clientId.c_str(), g_cfg.mqttUser.c_str(), g_cfg.mqttPass.c_str());
  } else {
    connected = g_mqtt.connect(clientId.c_str());
  }
  if (connected) {
    LOG_PRINTLN("MQTT: Connected successfully");
  } else {
    LOG_PRINTF("MQTT: Connection failed, state=%d\n", g_mqtt.state());
  }
#endif
}

#include "nettemp_ui.inc"

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

static String dsRomLinuxIdSimple(const std::array<uint8_t, 8>& rom) {
  char out[24]{};
  char serial[13]{};
  // Linux w1 serial is bytes 1..6 reversed (no CRC).
  snprintf(serial, sizeof(serial), "%02X%02X%02X%02X%02X%02X",
           rom[6], rom[5], rom[4], rom[3], rom[2], rom[1]);
  snprintf(out, sizeof(out), "ds_%02X_%s", (unsigned)rom[0], serial);
  String s(out);
  s.toLowerCase();
  return s;
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

static NimBLEScan* g_scan = nullptr;

static void processAdvertisedDevice(const NimBLEAdvertisedDevice& device) {
  const String advMac = device.getAddress().toString().c_str();
  const String advMacCanon = macWithColonsUpper(advMac);
  const int rssi = device.getRSSI();
  const uint32_t seenMs = millis();

  // STRICT MODE: Only track selected sensors (ignore all others)
  if (g_bleStrictMode) {
    bool isSelected = false;
    for (const auto& s : g_sensors) {
      if (macWithColonsUpper(s.mac) == advMacCanon) {
        if (s.selected || s.selectionLocked) {
          isSelected = true;
          break;
        }
      }
    }
    // If not in selected list, skip this advertisement completely
    if (!isSelected) return;
  }

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
  } else {
    // Track non-decoded devices so the user can see that scanning works even if decoding fails.
    const bool collectUnknown = true;
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
  if (!g_scan) return;
  if (!g_bleAutoScan) {
    if (g_scan->isScanning()) g_scan->stop();
    return;
  }
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

  LOG_PRINTLN("Scanning 1-Wire bus for DS18B20 sensors...");
  uint8_t addr[8]{};
  g_oneWire->reset_search();
  while (g_oneWire->search(addr)) {
    if (OneWire::crc8(addr, 7) != addr[7]) {
      LOG_PRINTLN("  CRC error - invalid device");
      continue;
    }
    // DS18B20 family=0x28, DS1822=0x22, DS18S20=0x10
    if (addr[0] != 0x28 && addr[0] != 0x22 && addr[0] != 0x10) {
      LOG_PRINTF("  Skipping non-temperature device (family 0x%02X)\n", addr[0]);
      continue;
    }
    std::array<uint8_t, 8> rom{};
    for (int i = 0; i < 8; i++) rom[i] = addr[i];
    bool exists = false;
    for (const auto& e : g_dsRoms) {
      if (e == rom) {
        exists = true;
        break;
      }
    }
    if (exists) {
      LOG_PRINTLN("  Duplicate DS18B20 ignored");
      continue;
    }
    g_dsRoms.push_back(rom);
    LOG_PRINTF("  Found DS18B20: ");
    for (int i = 0; i < 8; i++) LOG_PRINTF("%02X", rom[i]);
    LOG_PRINTLN();
  }

  g_dsTempsC.assign(g_dsRoms.size(), NAN);
  LOG_PRINTF("1-Wire scan complete: found %u DS18B20 sensor(s)\n", (unsigned)g_dsRoms.size());
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
    if (!g_oneWire->reset()) {
      LOG_PRINTLN("DS18B20: Failed to reset bus for conversion");
      return;
    }
    g_oneWire->skip();
    g_oneWire->write(0x44, 1); // CONVERT T (parasite power on)
    g_dsConvertInProgress = true;
    g_dsLastConvertStartMs = nowMs;
    LOG_PRINTF("DS18B20: Started temperature conversion for %u sensor(s)\n", (unsigned)g_dsRoms.size());
    return;
  }

  if ((nowMs - g_dsLastConvertStartMs) < 800UL) return;

  // Read all sensors (blocking but small: 9 bytes each).
  LOG_PRINTF("DS18B20: Reading %u sensor(s)...\n", (unsigned)g_dsRoms.size());
  unsigned int successCount = 0;
  for (size_t i = 0; i < g_dsRoms.size(); i++) {
    uint8_t data[9]{};
    if (!g_oneWire->reset()) {
      LOG_PRINTF("  [%u] Reset failed\n", (unsigned)i);
      continue;
    }
    g_oneWire->select(g_dsRoms[i].data());
    g_oneWire->write(0xBE); // READ SCRATCHPAD
    for (int j = 0; j < 9; j++) data[j] = g_oneWire->read();
    if (OneWire::crc8(data, 8) != data[8]) {
      LOG_PRINTF("  [%u] CRC error\n", (unsigned)i);
      continue;
    }

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
    LOG_PRINTF("  [%u] OK: %.2f°C (raw=0x%04X)\n", (unsigned)i, tempC, (unsigned)raw);
    successCount++;
  }

  LOG_PRINTF("DS18B20: Read complete - %u/%u successful\n", successCount, (unsigned)g_dsRoms.size());
  g_dsLastReadStatus = String(successCount) + "/" + String((unsigned)g_dsRoms.size()) + " OK";
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

static void hcsr04Tick() {
  if (!g_cfg.hcsr04Enabled) {
    g_hcsr04Cm = NAN;
    g_hcsr04LastSeenMs = 0;
    return;
  }
  if (g_cfg.hcsr04TrigPin < 0 || g_cfg.hcsr04EchoPin < 0) return;
  const uint32_t nowMs = millis();
  if (g_hcsr04LastReadMs != 0 && (nowMs - g_hcsr04LastReadMs) < 1200UL) return;
  g_hcsr04LastReadMs = nowMs;

  pinMode(g_cfg.hcsr04TrigPin, OUTPUT);
  pinMode(g_cfg.hcsr04EchoPin, INPUT);
  digitalWrite(g_cfg.hcsr04TrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(g_cfg.hcsr04TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(g_cfg.hcsr04TrigPin, LOW);

  const unsigned long duration = pulseIn(g_cfg.hcsr04EchoPin, HIGH, 30000UL);
  if (duration == 0) return;
  g_hcsr04Cm = (float)duration / 58.0f;
  g_hcsr04LastSeenMs = nowMs;
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

static void tickSendMqtt() {
#if !NETTEMP_ENABLE_MQTT
  return;
#else
  if (!g_cfg.mqttEnabled) {
    logSkipEvery(g_lastMqttSkipLogMs, "MQTT send skipped: MQTT not enabled (check Channels→MQTT→Enable MQTT checkbox)");
    return;
  }
  if (!wifiConnected()) {
    logSkipEvery(g_lastMqttSkipLogMs, "MQTT send skipped: WiFi not connected");
    return;
  }
  mqttEnsureConnected();
  if (!g_mqtt.connected()) {
    logSkipEvery(g_lastMqttSkipLogMs, "MQTT send failed: not connected to broker");
    return;
  }

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
    const bool ok = g_mqtt.publish(topic.c_str(), payload.c_str(), false);
    if (!ok) logSkipEvery(g_lastMqttSkipLogMs, "MQTT publish failed");
    (void)ok;
  };

  if (g_cfg.bleSendMqtt) {
    const String dev = g_cfg.deviceId.length() ? g_cfg.deviceId : String("nettemp_esp32");
    g_prefs.begin("nettemp", true);
    auto bleNameOr = [&](const String& macNo, const char* fallback) -> String {
      const String name = g_prefs.getString(prefKeyBleName(macNo).c_str(), "");
      return name.length() ? name : String(fallback);
    };
    for (auto& s : g_sensors) {
      if (!bleIsSelected(s)) continue;
      if (isnan(s.temperatureC) && isnan(s.humidityPct) && s.batteryPct < 0 && s.voltageMv < 0 && s.rssi == 0) continue;

      const uint32_t now = millis();
      // Allow the first publish immediately after boot/selection (lastMqttSentMs==0).
      if (g_cfg.mqttIntervalMs > 0 && s.lastMqttSentMs != 0 && now - s.lastMqttSentMs < g_cfg.mqttIntervalMs) continue;

      const String macNo = macNoColonsUpper(s.mac);
      const String base = dev + "-ble_" + macNo;
      auto publishReading = [&](const String& sensorId, const char* sensorType, const String& name,
                                const String& valueStr) {
        publishNettemp(dev, sensorId, sensorType, name, valueStr);
      };
      auto nameFor = [&](const char* fallback) { return bleNameOr(macNo, fallback); };
      publishBleReadingsMqtt(s, base, g_mqttBleFields, publishReading, nameFor);

      s.lastMqttSentMs = now;
    }
    g_prefs.end();
  }

  // Local sensors (I2C / DS18B20 / DHT / VBAT): publish multiple messages (one per sensor_id).
  {
    const uint32_t now = millis();
    if (g_cfg.mqttIntervalMs == 0 || g_lastMqttLocalSentMs == 0 || (now - g_lastMqttLocalSentMs) >= g_cfg.mqttIntervalMs) {
      bool anyLocalSent = false;
      const String dev = g_cfg.deviceId.length() ? g_cfg.deviceId : String("nettemp_esp32");
      auto publishReading = [&](const String& sensorId, const char* sensorType, const String& name, const String& valueStr) {
        publishNettemp(dev, sensorId, sensorType, name, valueStr);
        anyLocalSent = true;
      };
      auto publishReadingFor = [&](const String& deviceId, const String& sensorId, const char* sensorType, const String& name,
                                   const String& valueStr) {
        publishNettemp(deviceId, sensorId, sensorType, name, valueStr);
        anyLocalSent = true;
      };

#if NETTEMP_ENABLE_I2C
      if (g_cfg.i2cSendMqtt) {
        unsigned int sentCount = 0;
        for (auto& s : g_i2cSensors) {
          String addrHex = String(s.address, HEX);
          addrHex.toLowerCase();
          if (addrHex.length() == 1) addrHex = "0" + addrHex;
          const String typeName = String(i2cSensorTypeName(s.type));

          if (!s.selected) continue;
          if (!s.reading.ok) continue;

          const String base = dev + "-i2c_0x" + addrHex + "_" + typeName;
          auto nameFor = [&](const char* suffix) { return typeName + " " + suffix; };
          publishI2cReadingsMqtt(s, base, g_mqttI2cFields, publishReading, nameFor);

          s.last_mqtt_sent_ms = now;
          sentCount++;
        }
        (void)sentCount;
      }
#endif

      if (g_cfg.gpioSendMqtt) {
        auto tempfFromC = [](float tempc) { return (tempc * 9.0f / 5.0f) + 32.0f; };
        if (g_cfg.dsEnabled && !g_dsRoms.empty()) {
          for (size_t i = 0; i < g_dsRoms.size(); i++) {
            if (i >= g_dsTempsC.size() || isnan(g_dsTempsC[i])) continue;
            char romHex[17]{};
            for (int b = 0; b < 8; b++) sprintf(romHex + b * 2, "%02X", g_dsRoms[i][b]);
            // Check if this sensor is selected for sending
            const String selKey = String("dsSel_") + romHex;
            g_prefs.begin("nettemp", true);
            const bool selected = g_prefs.getBool(selKey.c_str(), true); // Default: selected
            g_prefs.end();
            if (!selected) continue;
            const String dsId = dsRomLinuxIdSimple(g_dsRoms[i]);
            const String sensorId = dev + "-" + dsId;
            if (g_mqttGpioFields & SRV_GPIO_TEMPC) {
              publishReadingFor(dev, sensorId, "temperature", "DS18B20 temp", String(g_dsTempsC[i], 2));
            }
            if (g_mqttGpioFields & SRV_GPIO_TEMPF) {
              const float tempf = tempfFromC(g_dsTempsC[i]);
              publishReadingFor(dev, sensorId + "_tempf", "temperature_f", "DS18B20 tempf", String(tempf, 2));
            }
          }
        }

        if (g_cfg.dhtEnabled && !isnan(g_dhtTempC)) {
          const String base = dev + "-dht" + String(g_cfg.dhtType) + "_gpio" + String(g_cfg.dhtPin);
          if (g_mqttGpioFields & SRV_GPIO_TEMPC) {
            publishReading(base + "_tempc", "temperature", "DHT temp", String(g_dhtTempC, 2));
          }
          if (g_mqttGpioFields & SRV_GPIO_TEMPF) {
            const float tempf = tempfFromC(g_dhtTempC);
            publishReading(base + "_tempf", "temperature_f", "DHT tempf", String(tempf, 2));
          }
          if ((g_mqttGpioFields & SRV_GPIO_HUM) && !isnan(g_dhtHumPct)) {
            publishReading(base + "_hum", "humidity", "DHT hum", String(g_dhtHumPct, 1));
          }
        }

        if (g_cfg.vbatMode != 0 && g_vbatPct >= 0) {
          if (g_mqttGpioFields & SRV_GPIO_BATT) {
            publishReading(dev + "-batt", "battery", "battery", String(g_vbatPct));
          }
          if ((g_mqttGpioFields & SRV_GPIO_VOLT) && g_cfg.vbatSendVolt && !isnan(g_vbatVolts)) {
            publishReading(dev + "-vbat", "voltage", "vbat", String(g_vbatVolts, 3));
          }
        }

        if (g_cfg.soilEnabled && g_soilRaw >= 0) {
          const String base = dev + "-soil_adc" + String(g_cfg.soilAdcPin);
          if (g_mqttGpioFields & SRV_GPIO_SOIL_RAW) {
            publishReading(base + "_raw", "soil_raw", "soil raw", String(g_soilRaw));
          }
          if ((g_mqttGpioFields & SRV_GPIO_SOIL_PCT) && !isnan(g_soilPct)) {
            publishReading(base + "_pct", "soil_moisture", "soil", String(g_soilPct, 1));
          }
        }
        if (g_cfg.hcsr04Enabled && !isnan(g_hcsr04Cm)) {
          const String base = dev + "-hcsr04_gpio" + String(g_cfg.hcsr04TrigPin) + "_" + String(g_cfg.hcsr04EchoPin);
          if (g_mqttGpioFields & SRV_GPIO_DIST) {
            publishReading(base + "_dist", "distance", "distance", String(g_hcsr04Cm, 1));
          }
        }
      }

      if (anyLocalSent) g_lastMqttLocalSentMs = now;
    }
  }

  g_mqtt.loop();
#endif
}

#if NETTEMP_ENABLE_SERVER
// Configuration struct for HTTP-based channels (Server, LocalServer)
struct HttpChannelConfig {
  bool enabled;
  const char* channelName;
  const String& url;
  const String& apiKey;
  bool requireApiKey;
  bool useEndpoint;  // true = set batch.endpoint, false = set batch.baseUrl
  uint32_t intervalMs;
  uint32_t& lastSendMs;
  uint32_t& lastSkipLogMs;
  bool sendBle;
  bool sendI2c;
  bool sendGpio;
  uint32_t bleFieldMask;
  uint32_t i2cFieldMask;
  uint32_t gpioFieldMask;
  bool soilSendRaw;
  bool soilSendPct;
};

// Generic HTTP channel send function (used by Server, LocalServer, Webhook)
static void tickSendHttpChannel(const HttpChannelConfig& cfg) {
  if (!cfg.enabled) {
    char msg[64];
    snprintf(msg, sizeof(msg), "%s send skipped: not enabled", cfg.channelName);
    logSkipEvery(cfg.lastSkipLogMs, msg);
    return;
  }
  if (!wifiConnected()) {
    char msg[64];
    snprintf(msg, sizeof(msg), "%s send skipped: WiFi not connected", cfg.channelName);
    logSkipEvery(cfg.lastSkipLogMs, msg);
    return;
  }
  if (cfg.url.length() == 0) {
    char msg[64];
    snprintf(msg, sizeof(msg), "%s send skipped: URL not set", cfg.channelName);
    logSkipEvery(cfg.lastSkipLogMs, msg);
    return;
  }
  if (cfg.requireApiKey && cfg.apiKey.length() == 0) {
    char msg[64];
    snprintf(msg, sizeof(msg), "%s send skipped: API key not set", cfg.channelName);
    logSkipEvery(cfg.lastSkipLogMs, msg);
    return;
  }
  if (cfg.intervalMs > 0 && cfg.lastSendMs != 0 && (millis() - cfg.lastSendMs) < cfg.intervalMs) return;

  const uint32_t ts = (uint32_t)(time(nullptr) > 100000 ? time(nullptr) : (millis() / 1000));
  bool anySent = false;

  // BLE sensors
  if (cfg.sendBle) {
    const String deviceId = g_cfg.deviceId.length() ? g_cfg.deviceId : String("nettemp_esp32");
    g_prefs.begin("nettemp", true);
    auto nameOr = [&](const String& macNo, const char* fallback) -> String {
      const String name = g_prefs.getString(prefKeyBleName(macNo).c_str(), "");
      return name.length() ? name : String(fallback);
    };
    for (const auto& s : g_sensors) {
      if (!bleIsSelected(s)) continue;
      const String macNo = macNoColonsUpper(s.mac);
      const String base = deviceId + "-ble_" + macNo;

      NettempBatch batch;
      batch.deviceId = deviceId;
      batch.apiKey = cfg.apiKey;
      if (cfg.useEndpoint) {
        batch.endpoint = cfg.url;
      } else {
        batch.baseUrl = cfg.url;
      }
      batch.requireApiKey = cfg.requireApiKey;

      auto nameFor = [&](const char* fallback) { return nameOr(macNo, fallback); };
      appendBleReadingsServerLike(batch, s, base, cfg.bleFieldMask, ts, nameFor);

      if (!batch.readings.empty()) {
        const bool ok = nettempPostBatch(g_tlsClient, batch);
        if (!ok) {
          char msg[64];
          snprintf(msg, sizeof(msg), "%s send failed", cfg.channelName);
          logSkipEvery(cfg.lastSkipLogMs, msg);
        }
        anySent = true;
      }
    }
    g_prefs.end();
  }

  // I2C sensors
#if NETTEMP_ENABLE_I2C
  if (cfg.sendI2c && !g_i2cSensors.empty()) {
    const String deviceId = g_cfg.deviceId.length() ? g_cfg.deviceId : String("nettemp_esp32");
    g_prefs.begin("nettemp", true);
    auto nameOr = [&](const String& addrHex, const char* fallback) -> String {
      const String name = g_prefs.getString(prefKeyI2cName(addrHex).c_str(), "");
      return name.length() ? name : String(fallback);
    };
    for (const auto& s : g_i2cSensors) {
      if (!s.selected || !s.reading.ok) continue;
      String addrHex = String(s.address, HEX);
      addrHex.toLowerCase();
      if (addrHex.length() == 1) addrHex = "0" + addrHex;
      const String typeName = String(i2cSensorTypeName(s.type));
      const String base = deviceId + "-i2c_0x" + addrHex + "_" + typeName;

      NettempBatch batch;
      batch.deviceId = deviceId;
      batch.apiKey = cfg.apiKey;
      if (cfg.useEndpoint) {
        batch.endpoint = cfg.url;
      } else {
        batch.baseUrl = cfg.url;
      }
      batch.requireApiKey = cfg.requireApiKey;

      auto nameFor = [&](const char* field) {
        return nameOr(addrHex, (typeName + " " + field).c_str());
      };
      appendI2cReadingsServerLike(batch, s, base, cfg.i2cFieldMask, ts, nameFor);

      if (!batch.readings.empty()) {
        const bool ok = nettempPostBatch(g_tlsClient, batch);
        if (!ok) {
          char msg[64];
          snprintf(msg, sizeof(msg), "%s send failed", cfg.channelName);
          logSkipEvery(cfg.lastSkipLogMs, msg);
        }
        anySent = true;
      }
    }
    g_prefs.end();
  }
#endif

  // GPIO sensors
  if (cfg.sendGpio) {
    const String deviceId = g_cfg.deviceId.length() ? g_cfg.deviceId : String("nettemp_esp32");

    // DS18B20 sensors
    if (g_cfg.dsEnabled && !g_dsRoms.empty()) {
      g_prefs.begin("nettemp", true);
      auto nameOr = [&](const String& romHex, const char* fallback) -> String {
        const String name = g_prefs.getString(prefKeyDsName(romHex).c_str(), "");
        if (name.length()) return name;
        const String legacyName = g_prefs.getString(prefKeyDsNameLegacy(romHex).c_str(), "");
        return legacyName.length() ? legacyName : String(fallback);
      };
      for (size_t i = 0; i < g_dsRoms.size(); i++) {
        char romHex[17]{};
        for (int b = 0; b < 8; b++) sprintf(romHex + b * 2, "%02X", g_dsRoms[i][b]);
        const String selKey = String("dsSel_") + romHex;
        const bool selected = g_prefs.getBool(selKey.c_str(), true);
        if (!selected) continue;

        const String linuxId = dsRomLinuxId(g_dsRoms[i], 0, false);
        const String base = deviceId + "-" + linuxId;

        NettempBatch batch;
        batch.deviceId = deviceId;
        batch.apiKey = cfg.apiKey;
        if (cfg.useEndpoint) {
          batch.endpoint = cfg.url;
        } else {
          batch.baseUrl = cfg.url;
        }
        batch.requireApiKey = cfg.requireApiKey;

        auto nameFor = [&](const char* fallback) { return nameOr(romHex, fallback); };
        if (i < g_dsTempsC.size() && !isnan(g_dsTempsC[i])) {
          if (cfg.gpioFieldMask & SRV_GPIO_TEMPC) {
            batch.readings.push_back(NettempReading{
              .sensorId = base + "_temp",
              .value = g_dsTempsC[i],
              .sensorType = "temperature",
              .unit = "°C",
              .timestamp = ts,
              .friendlyName = nameFor("DS18B20"),
            });
          }
          if (cfg.gpioFieldMask & SRV_GPIO_TEMPF) {
            batch.readings.push_back(NettempReading{
              .sensorId = base + "_tempf",
              .value = (g_dsTempsC[i] * 9.0f / 5.0f) + 32.0f,
              .sensorType = "temperature_f",
              .unit = "°F",
              .timestamp = ts,
              .friendlyName = nameFor("DS18B20"),
            });
          }
        }

        if (!batch.readings.empty()) {
          const bool ok = nettempPostBatch(g_tlsClient, batch);
          if (!ok) {
            char msg[64];
            snprintf(msg, sizeof(msg), "%s send failed", cfg.channelName);
            logSkipEvery(cfg.lastSkipLogMs, msg);
          }
          anySent = true;
        }
      }
      g_prefs.end();
    }

    // DHT sensor
    if (g_cfg.dhtEnabled && (!isnan(g_dhtTempC) || !isnan(g_dhtHumPct))) {
      g_prefs.begin("nettemp", true);
      const String dhtName = g_prefs.getString("name_dht", "");
      g_prefs.end();
      const String base = deviceId + "-dht" + String(g_cfg.dhtType) + "_gpio" + String(g_cfg.dhtPin);

      NettempBatch batch;
      batch.deviceId = deviceId;
      batch.apiKey = cfg.apiKey;
      if (cfg.useEndpoint) {
        batch.endpoint = cfg.url;
      } else {
        batch.baseUrl = cfg.url;
      }
      batch.requireApiKey = cfg.requireApiKey;

      if (!isnan(g_dhtTempC)) {
        if (cfg.gpioFieldMask & SRV_GPIO_TEMPC) {
          batch.readings.push_back(NettempReading{
            .sensorId = base + "_temp",
            .value = g_dhtTempC,
            .sensorType = "temperature",
            .unit = "°C",
            .timestamp = ts,
            .friendlyName = dhtName.length() ? dhtName : String("DHT"),
          });
        }
        if (cfg.gpioFieldMask & SRV_GPIO_TEMPF) {
          batch.readings.push_back(NettempReading{
            .sensorId = base + "_tempf",
            .value = (g_dhtTempC * 9.0f / 5.0f) + 32.0f,
            .sensorType = "temperature_f",
            .unit = "°F",
            .timestamp = ts,
            .friendlyName = dhtName.length() ? dhtName : String("DHT"),
          });
        }
      }
      if ((cfg.gpioFieldMask & SRV_GPIO_HUM) && !isnan(g_dhtHumPct)) {
        batch.readings.push_back(NettempReading{
          .sensorId = base + "_hum",
          .value = g_dhtHumPct,
          .sensorType = "humidity",
          .unit = "%",
          .timestamp = ts,
          .friendlyName = dhtName.length() ? dhtName : String("DHT"),
        });
      }

      if (!batch.readings.empty()) {
        const bool ok = nettempPostBatch(g_tlsClient, batch);
        if (!ok) {
          char msg[64];
          snprintf(msg, sizeof(msg), "%s send failed", cfg.channelName);
          logSkipEvery(cfg.lastSkipLogMs, msg);
        }
        anySent = true;
      }
    }

    // VBAT sensor
    if (g_cfg.vbatMode != 0 && (g_vbatPct >= 0 || !isnan(g_vbatVolts))) {
      g_prefs.begin("nettemp", true);
      const String vbatName = g_prefs.getString("name_vbat", "");
      g_prefs.end();
      const String base = deviceId + "-vbat";

      NettempBatch batch;
      batch.deviceId = deviceId;
      batch.apiKey = cfg.apiKey;
      if (cfg.useEndpoint) {
        batch.endpoint = cfg.url;
      } else {
        batch.baseUrl = cfg.url;
      }
      batch.requireApiKey = cfg.requireApiKey;

      if (g_vbatPct >= 0 && (cfg.gpioFieldMask & SRV_GPIO_BATT)) {
        batch.readings.push_back(NettempReading{
          .sensorId = base + "_batt",
          .value = (float)g_vbatPct,
          .sensorType = "battery",
          .unit = "%",
          .timestamp = ts,
          .friendlyName = vbatName.length() ? vbatName : String("VBAT"),
        });
      }
      if (!isnan(g_vbatVolts) && (cfg.gpioFieldMask & SRV_GPIO_VOLT)) {
        batch.readings.push_back(NettempReading{
          .sensorId = base + "_volt",
          .value = g_vbatVolts,
          .sensorType = "voltage",
          .unit = "V",
          .timestamp = ts,
          .friendlyName = vbatName.length() ? vbatName : String("VBAT"),
        });
      }

      if (!batch.readings.empty()) {
        const bool ok = nettempPostBatch(g_tlsClient, batch);
        if (!ok) {
          char msg[64];
          snprintf(msg, sizeof(msg), "%s send failed", cfg.channelName);
          logSkipEvery(cfg.lastSkipLogMs, msg);
        }
        anySent = true;
      }
    }

    // Soil sensor
    if (g_cfg.soilEnabled && (g_soilRaw >= 0 || !isnan(g_soilPct))) {
      g_prefs.begin("nettemp", true);
      const String soilName = g_prefs.getString("name_soil", "");
      g_prefs.end();
      const String base = deviceId + "-soil_adc" + String(g_cfg.soilAdcPin);

      NettempBatch batch;
      batch.deviceId = deviceId;
      batch.apiKey = cfg.apiKey;
      if (cfg.useEndpoint) {
        batch.endpoint = cfg.url;
      } else {
        batch.baseUrl = cfg.url;
      }
      batch.requireApiKey = cfg.requireApiKey;

      if (g_soilRaw >= 0 && cfg.soilSendRaw && (cfg.gpioFieldMask & SRV_GPIO_SOIL_RAW)) {
        batch.readings.push_back(NettempReading{
          .sensorId = base + "_raw",
          .value = (float)g_soilRaw,
          .sensorType = "soil_moisture_raw",
          .unit = "",
          .timestamp = ts,
          .friendlyName = soilName.length() ? soilName : String("Soil"),
        });
      }
      if (!isnan(g_soilPct) && cfg.soilSendPct && (cfg.gpioFieldMask & SRV_GPIO_SOIL_PCT)) {
        batch.readings.push_back(NettempReading{
          .sensorId = base + "_pct",
          .value = g_soilPct,
          .sensorType = "soil_moisture",
          .unit = "%",
          .timestamp = ts,
          .friendlyName = soilName.length() ? soilName : String("Soil"),
        });
      }

      if (!batch.readings.empty()) {
        const bool ok = nettempPostBatch(g_tlsClient, batch);
        if (!ok) {
          char msg[64];
          snprintf(msg, sizeof(msg), "%s send failed", cfg.channelName);
          logSkipEvery(cfg.lastSkipLogMs, msg);
        }
        anySent = true;
      }
    }

    // HC-SR04 sensor
    if (g_cfg.hcsr04Enabled && !isnan(g_hcsr04Cm) && (cfg.gpioFieldMask & SRV_GPIO_DIST)) {
      g_prefs.begin("nettemp", true);
      const String hcName = g_prefs.getString("name_hcsr04", "");
      g_prefs.end();
      const String base = deviceId + "-hcsr04_gpio" + String(g_cfg.hcsr04TrigPin) + "_" + String(g_cfg.hcsr04EchoPin);

      NettempBatch batch;
      batch.deviceId = deviceId;
      batch.apiKey = cfg.apiKey;
      if (cfg.useEndpoint) {
        batch.endpoint = cfg.url;
      } else {
        batch.baseUrl = cfg.url;
      }
      batch.requireApiKey = cfg.requireApiKey;

      batch.readings.push_back(NettempReading{
        .sensorId = base + "_dist",
        .value = g_hcsr04Cm,
        .sensorType = "distance",
        .unit = "cm",
        .timestamp = ts,
        .friendlyName = hcName.length() ? hcName : String("HC-SR04"),
      });

      if (!batch.readings.empty()) {
        const bool ok = nettempPostBatch(g_tlsClient, batch);
        if (!ok) {
          char msg[64];
          snprintf(msg, sizeof(msg), "%s send failed", cfg.channelName);
          logSkipEvery(cfg.lastSkipLogMs, msg);
        }
        anySent = true;
      }
    }
  }

  if (anySent) cfg.lastSendMs = millis();
}
#endif // NETTEMP_ENABLE_SERVER

static void tickSendServer() {
#if !NETTEMP_ENABLE_SERVER
  return;
#else
  HttpChannelConfig cfg = {
    .enabled = g_cfg.serverEnabled,
    .channelName = "Server",
    .url = g_cfg.serverBaseUrl,
    .apiKey = g_cfg.serverApiKey,
    .requireApiKey = true,
    .useEndpoint = false,
    .intervalMs = g_cfg.serverIntervalMs,
    .lastSendMs = g_lastServerSendMs,
    .lastSkipLogMs = g_lastServerSkipLogMs,
    .sendBle = g_cfg.bleSendServer,
    .sendI2c = g_cfg.i2cSendServer,
    .sendGpio = g_cfg.gpioSendServer,
    .bleFieldMask = g_srvBleFields,
    .i2cFieldMask = g_srvI2cFields,
    .gpioFieldMask = g_srvGpioFields,
    .soilSendRaw = true,
    .soilSendPct = true,
  };
  tickSendHttpChannel(cfg);
#endif
}

static void tickSendLocalServer() {
#if !NETTEMP_ENABLE_SERVER
  return;
#else
  HttpChannelConfig cfg = {
    .enabled = g_cfg.localServerEnabled,
    .channelName = "LocalServer",
    .url = g_cfg.localServerUrl,
    .apiKey = g_cfg.localServerApiKey,
    .requireApiKey = false,
    .useEndpoint = true,
    .intervalMs = g_cfg.localServerIntervalMs,
    .lastSendMs = g_lastLocalServerSendMs,
    .lastSkipLogMs = g_lastLocalServerSkipLogMs,
    .sendBle = g_cfg.bleSendLocalServer,
    .sendI2c = g_cfg.i2cSendLocalServer,
    .sendGpio = g_cfg.gpioSendLocalServer,
    .bleFieldMask = g_localBleFields,
    .i2cFieldMask = g_localI2cFields,
    .gpioFieldMask = g_localGpioFields,
    .soilSendRaw = true,
    .soilSendPct = true,
  };
  tickSendHttpChannel(cfg);
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

  // Set timeouts to prevent indefinite hanging (critical for async operation)
  http.setConnectTimeout(5000);  // 5 second connection timeout
  http.setTimeout(10000);         // 10 second total timeout

  http.addHeader("Content-Type", "application/json");
  const int code = http.POST((uint8_t*)payload.c_str(), payload.length());
  http.end();
  return code >= 200 && code < 300;
}

static void tickSendWebhook() {
  if (!g_cfg.webhookEnabled) {
    logSkipEvery(g_lastWebhookSkipLogMs, "Webhook send skipped: Webhook not enabled");
    return;
  }
  if (!wifiConnected()) {
    logSkipEvery(g_lastWebhookSkipLogMs, "Webhook send skipped: WiFi not connected");
    return;
  }
  if (!g_cfg.webhookUrl.length()) {
    logSkipEvery(g_lastWebhookSkipLogMs, "Webhook send skipped: Webhook URL not set");
    return;
  }
  if (g_cfg.webhookIntervalMs > 0 && g_lastWebhookSendMs != 0 &&
      (millis() - g_lastWebhookSendMs) < g_cfg.webhookIntervalMs) return;

  const uint32_t ts = (uint32_t)(time(nullptr) > 100000 ? time(nullptr) : (millis() / 1000));
  bool anySent = false;

  if (g_cfg.bleSendWebhook) {
    for (const auto& s : g_sensors) {
      if (!bleIsSelected(s)) continue;
    const String mac = s.mac;
    const String macNoColons = macNoColonsUpper(mac);
    NettempBatch batch;
    batch.deviceId = macNoColons;
    auto nameFor = [&](const char* fallback) { return String(fallback); };
    appendBleReadingsWebhook(batch, s, macNoColons, g_webhookBleFields, ts, nameFor);
    if (!batch.readings.empty()) {
      const String payload = buildWebhookPayload(batch);
      const bool ok = webhookPostJson(g_cfg.webhookUrl, payload);
      if (!ok) logSkipEvery(g_lastWebhookSkipLogMs, "Webhook send failed");
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
      auto nameFor = [&](const char* suffix) { return typeName + " " + suffix; };
      appendI2cReadingsServerLike(batch, s, base, g_webhookI2cFields, ts, nameFor);
      if (!batch.readings.empty()) {
        const String payload = buildWebhookPayload(batch);
        const bool ok = webhookPostJson(g_cfg.webhookUrl, payload);
        if (!ok) logSkipEvery(g_lastWebhookSkipLogMs, "Webhook send failed");
        anySent = true;
      }
    }
  }
#endif

  if (g_cfg.gpioSendWebhook) {
    const String deviceId = g_cfg.deviceId.length() ? g_cfg.deviceId : String("nettemp_esp32");
    NettempBatch batch;
    batch.deviceId = deviceId;
    auto tempfFromC = [](float tempc) { return (tempc * 9.0f / 5.0f) + 32.0f; };
    if (g_cfg.dsEnabled && !g_dsRoms.empty()) {
      for (size_t i = 0; i < g_dsRoms.size(); i++) {
        if (i >= g_dsTempsC.size() || isnan(g_dsTempsC[i])) continue;
        char romHex[17]{};
        for (int b = 0; b < 8; b++) sprintf(romHex + b * 2, "%02X", g_dsRoms[i][b]);
        // Check if this sensor is selected for sending
        const String selKey = String("dsSel_") + romHex;
        g_prefs.begin("nettemp", true);
        const bool selected = g_prefs.getBool(selKey.c_str(), true); // Default: selected
        g_prefs.end();
        if (!selected) continue;
        const String dsId = dsRomLinuxIdSimple(g_dsRoms[i]);
        const String sensorId = deviceId + "-" + dsId;
        NettempBatch dsBatch;
        dsBatch.deviceId = deviceId;
        if (g_webhookGpioFields & SRV_GPIO_TEMPC) {
          dsBatch.readings.push_back(NettempReading{
            .sensorId = sensorId,
            .value = g_dsTempsC[i],
            .sensorType = "temperature",
            .unit = "°C",
            .timestamp = ts,
            .friendlyName = "DS18B20 temp",
          });
        }
        if (g_webhookGpioFields & SRV_GPIO_TEMPF) {
          const float tempf = tempfFromC(g_dsTempsC[i]);
          dsBatch.readings.push_back(NettempReading{
            .sensorId = sensorId + "_tempf",
            .value = tempf,
            .sensorType = "temperature_f",
            .unit = "°F",
            .timestamp = ts,
            .friendlyName = "DS18B20 tempf",
          });
        }
        if (!dsBatch.readings.empty()) {
          const String payload = buildWebhookPayload(dsBatch);
          const bool ok = webhookPostJson(g_cfg.webhookUrl, payload);
          if (!ok) logSkipEvery(g_lastWebhookSkipLogMs, "Webhook send failed");
          anySent = true;
        }
      }
    }
    if (g_cfg.dhtEnabled && !isnan(g_dhtTempC)) {
      const String base = deviceId + "-dht" + String(g_cfg.dhtType) + "_gpio" + String(g_cfg.dhtPin);
      if (g_webhookGpioFields & SRV_GPIO_TEMPC) {
        batch.readings.push_back(NettempReading{
          .sensorId = base + "_temp",
          .value = g_dhtTempC,
          .sensorType = "temperature",
          .unit = "°C",
          .timestamp = ts,
          .friendlyName = "DHT temp",
        });
      }
      if (g_webhookGpioFields & SRV_GPIO_TEMPF) {
        const float tempf = tempfFromC(g_dhtTempC);
        batch.readings.push_back(NettempReading{
          .sensorId = base + "_tempf",
          .value = tempf,
          .sensorType = "temperature_f",
          .unit = "°F",
          .timestamp = ts,
          .friendlyName = "DHT tempf",
        });
      }
      if ((g_webhookGpioFields & SRV_GPIO_HUM) && !isnan(g_dhtHumPct)) {
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
      if (g_webhookGpioFields & SRV_GPIO_BATT) {
        batch.readings.push_back(NettempReading{
          .sensorId = deviceId + "_batt",
          .value = (float)g_vbatPct,
          .sensorType = "battery",
          .unit = "%",
          .timestamp = ts,
          .friendlyName = "battery",
        });
      }
      if ((g_webhookGpioFields & SRV_GPIO_VOLT) && g_cfg.vbatSendVolt && !isnan(g_vbatVolts)) {
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
      if (g_webhookGpioFields & SRV_GPIO_SOIL_RAW) {
        batch.readings.push_back(NettempReading{
          .sensorId = base + "_raw",
          .value = (float)g_soilRaw,
          .sensorType = "soil_raw",
          .unit = "",
          .timestamp = ts,
          .friendlyName = "soil raw",
        });
      }
      if ((g_webhookGpioFields & SRV_GPIO_SOIL_PCT) && !isnan(g_soilPct)) {
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
    if (g_cfg.hcsr04Enabled && !isnan(g_hcsr04Cm)) {
      const String base = deviceId + "-hcsr04_gpio" + String(g_cfg.hcsr04TrigPin) + "_" + String(g_cfg.hcsr04EchoPin);
      if (g_webhookGpioFields & SRV_GPIO_DIST) {
        batch.readings.push_back(NettempReading{
          .sensorId = base + "_dist",
          .value = g_hcsr04Cm,
          .sensorType = "distance",
          .unit = "cm",
          .timestamp = ts,
          .friendlyName = "distance",
        });
      }
    }
    if (!batch.readings.empty()) {
      const String payload = buildWebhookPayload(batch);
      const bool ok = webhookPostJson(g_cfg.webhookUrl, payload);
      if (!ok) logSkipEvery(g_lastWebhookSkipLogMs, "Webhook send failed");
      anySent = true;
    }
  }

  if (anySent) g_lastWebhookSendMs = millis();
}
#endif


} // namespace

void setup() {
  Serial.begin(NETTEMP_SERIAL_BAUD);
  delay(1000);
  LOG_PRINTLN("Nettemp Cardputer starting...");
  g_powerBootMs = millis();
  g_powerBootCycleDone = false;

  // Configure hardware watchdog (framework already initializes it)
  // Load prefs first to get watchdog timeout setting
  prefsLoad();

  // Check for crash info from previous boot
  crashCheckAndLoad();
  if (g_hasCrashInfo) {
    LOG_PRINTLN("\n!!! CRASH INFO FOUND FROM PREVIOUS BOOT !!!");
    LOG_PRINT("Reason: ");
    LOG_PRINTLN(g_crashReason.c_str());
    LOG_PRINT("Location: ");
    LOG_PRINTLN(g_crashLocation.c_str());
    LOG_PRINT("Uptime: ");
    LOG_PRINT(g_crashUptime / 1000);
    LOG_PRINTLN("s");
    LOG_PRINTLN("View crash log in System tab (or download via /crash.txt)\n");
  }

  // Deinit first to avoid "already initialized" error, then reinit with our settings
  esp_task_wdt_deinit();
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = g_cfg.wdtTimeoutSeconds * 1000UL,  // user-configurable timeout
    .idle_core_mask = 0,       // Don't watch idle tasks
    .trigger_panic = true      // Panic and reboot on timeout
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);      // Add current task to watchdog
  LOG_PRINT("Watchdog enabled: ");
  LOG_PRINT(g_cfg.wdtTimeoutSeconds);
  LOG_PRINTLN("s timeout");

  esp_task_wdt_reset();  // Feed watchdog after prefs load

#if NETTEMP_CARDPUTER_UI
  uiSetup();
#endif
  showSplash();
  yield();

  WiFi.mode(WIFI_STA);
  wifiConnectIfConfigured();
  esp_task_wdt_reset();  // Feed watchdog after WiFi attempt
  yield();

#if NETTEMP_ENABLE_PORTAL
  g_portalBootMs = millis();
  if (g_portalAuto && g_cfg.wifiSsid.length() == 0) {
    portalStart();
    esp_task_wdt_reset();  // Feed watchdog after portal start
    yield();
  }
#endif

  i2cInitBus();
#if NETTEMP_ENABLE_OLED
  oledInit();
#endif
#if NETTEMP_ENABLE_I2C
  g_i2cDetectedAddrs = i2cScanAllAddresses(Wire);
  g_i2cSensors = i2cDetectKnownSensors(Wire);
  esp_task_wdt_reset();  // Feed watchdog after I2C scan
  if (!g_i2cSensors.empty()) i2cUpdateReadings(Wire, g_i2cSensors);
  if (g_i2cSensors.empty() && !g_i2cCachedSensors.empty()) {
    g_i2cSensors = g_i2cCachedSensors;
    g_i2cDetectedAddrs.clear();
    for (const auto& s : g_i2cCachedSensors) g_i2cDetectedAddrs.push_back(s.address);
  } else {
    i2cPersistLastDetected();
  }
  if (g_i2cSelDefined) {
    i2cApplySelectionToDetected();
  }
#endif

  dsEnsureBus();
  if (g_cfg.dsEnabled) dsRescan();
  vbatTick();
  esp_task_wdt_reset();  // Feed watchdog after sensor init
  yield();

  crashSaveBreadcrumb("INIT_BLE", "setup:NimBLEDevice_init");
  NimBLEDevice::init("nettemp-esp32");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  g_scan = NimBLEDevice::getScan();
  if (g_bleAutoScan) bleConfigureScan();
  esp_task_wdt_reset();  // Feed watchdog after BLE init

  // Clear crash breadcrumb after successful initialization
  crashClear();
  LOG_PRINTLN("Setup complete, crash tracking active");

  // Duty-cycle runs in loop() so portal/webserver can run too (useful for debugging / short wake windows).
}

void loop() {
  // Feed watchdog to prevent auto-reset (signals "still alive")
  esp_task_wdt_reset();

#if NETTEMP_CARDPUTER_UI
  M5Cardputer.update();
  handleKeys();
  uiLoopTick();
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
  hcsr04Tick();
  vbatTick();
#if NETTEMP_ENABLE_OLED
  oledTick();
#endif

  // Process async send flags from web UI (prevents blocking web server)
  // CRITICAL: Atomic check-and-clear to prevent race conditions
  bool pendingMqtt, pendingServer, pendingLocalServer;
#if NETTEMP_ENABLE_SERVER
  bool pendingWebhook;
#endif

  noInterrupts();  // Disable interrupts for atomic operation
  pendingMqtt = g_pendingSendMqtt;
  g_pendingSendMqtt = false;
  pendingServer = g_pendingSendServer;
  g_pendingSendServer = false;
  pendingLocalServer = g_pendingSendLocalServer;
  g_pendingSendLocalServer = false;
#if NETTEMP_ENABLE_SERVER
  pendingWebhook = g_pendingSendWebhook;
  g_pendingSendWebhook = false;
#endif
  interrupts();  // Re-enable interrupts

  if (pendingMqtt) {
    tickSendMqtt();
    g_sendStatusMqtt = 2;  // Set status to "success"
    esp_task_wdt_reset();  // Feed watchdog after send
  }
  if (pendingServer) {
    tickSendServer();
    g_sendStatusServer = 2;  // Set status to "success"
    esp_task_wdt_reset();  // Feed watchdog after send
  }
  if (pendingLocalServer) {
    tickSendLocalServer();
    g_sendStatusLocalServer = 2;  // Set status to "success"
    esp_task_wdt_reset();  // Feed watchdog after send
  }
#if NETTEMP_ENABLE_SERVER
  if (pendingWebhook) {
    tickSendWebhook();
    g_sendStatusWebhook = 2;  // Set status to "success"
    esp_task_wdt_reset();  // Feed watchdog after send
  }
#endif

  // Regular periodic sending
  tickSendMqtt();
  esp_task_wdt_reset();  // Feed watchdog after MQTT (can block on connect)
  tickSendServer();
  tickSendLocalServer();
#if NETTEMP_ENABLE_SERVER
  tickSendWebhook();
#endif
  esp_task_wdt_reset();  // Feed watchdog after all sends

  // Ensure background BLE scan stays running.
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
  if (g_i2cSensors.empty()) {
    static uint32_t s_lastI2cRescanMs = 0;
    if (s_lastI2cRescanMs == 0 || (nowMs - s_lastI2cRescanMs) >= 10'000UL) {
      s_lastI2cRescanMs = nowMs;
      g_i2cDetectedAddrs = i2cScanAllAddresses(Wire);
      g_i2cSensors = i2cDetectKnownSensors(Wire);
      if (!g_i2cSensors.empty()) i2cUpdateReadings(Wire, g_i2cSensors);
      if (g_i2cSelDefined) i2cApplySelectionToDetected();
    }
  }
#endif


  delay(10);
}
