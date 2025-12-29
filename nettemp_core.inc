#if NETTEMP_ENABLE_PORTAL
#include <DNSServer.h>
#include <WebServer.h>
#endif

#include <algorithm>
#include <array>
#include <cctype>

#if NETTEMP_ENABLE_MQTT
#include <PubSubClient.h>
#endif
#include <OneWire.h>
#if NETTEMP_ENABLE_BLE
#include <NimBLEDevice.h>
#endif
#include <Wire.h>

#include "lywsd03mmc_atc_pvvx.h"
#if NETTEMP_ENABLE_SERVER
#include <WiFiClientSecure.h>
#include "nettemp_http.h"
#endif
#if NETTEMP_ENABLE_I2C
#include "i2c_sensors.h"
#endif

namespace {

constexpr uint32_t UI_REFRESH_MS = 1000;  // 1 second refresh to reduce blinking
constexpr uint32_t STALE_AFTER_MS = 20UL * 60UL * 1000UL;
constexpr uint32_t SPLASH_MS = 1100;
constexpr uint32_t QR_ROTATE_MS = 2000;
constexpr const char* NETTEMP_APP_URL = "https://app.nettemp.pl";

#if NETTEMP_CARDPUTER_UI
constexpr uint16_t COLOR_BG = TFT_BLACK;
constexpr uint16_t COLOR_FG = TFT_WHITE;
constexpr uint16_t COLOR_ACCENT = TFT_BLUE;
constexpr uint16_t COLOR_MUTED = TFT_DARKGREY;
constexpr uint16_t COLOR_WARN = TFT_ORANGE;
#endif

struct AppSettings {
  String deviceId = "nettemp_esp32";

  String wifiSsid;
  String wifiPass;
  String webUser = "admin";
  String webPass;

  bool mqttEnabled = false;
  String mqttHost = "192.168.1.10";
  uint16_t mqttPort = 1883;
  String mqttUser;
  String mqttPass;
  uint32_t mqttIntervalMs = 30'000;

  bool serverEnabled = false;
  String serverBaseUrl = "https://api.nettemp.pl";
  String serverApiKey; // ntk_...
  uint32_t serverIntervalMs = 60'000;
  bool localServerEnabled = false;
  String localServerUrl = "http://192.168.1.10/api/v1/data";
  String localServerApiKey;
  uint32_t localServerIntervalMs = 60'000;
  bool webhookEnabled = false;
  String webhookUrl;
  uint32_t webhookIntervalMs = 60'000;
  bool bleSendServer = true;
  bool bleSendMqtt = true;
  bool bleSendWebhook = true;
  bool bleSendLocalServer = true;
  bool i2cSendServer = true;
  bool i2cSendMqtt = true;
  bool i2cSendWebhook = true;
  bool i2cSendLocalServer = true;
  bool gpioSendServer = true;
  bool gpioSendMqtt = true;
  bool gpioSendWebhook = true;
  bool gpioSendLocalServer = true;

  // GPIO sensors
  bool dsEnabled = false;
  int dsPin = 4;   // GPIO for DS18B20 (1-Wire)
  bool dhtEnabled = false;
  bool dhtSelected = true;
  int dhtPin = 16; // GPIO for DHT11/DHT22
  int dhtType = 22; // 11 or 22
  bool soilEnabled = false;
  bool soilSelected = true;
  int soilAdcPin = 34; // ADC1 pin for capacitive soil sensor
  int soilDryRaw = 3000;
  int soilWetRaw = 1400;
  bool hcsr04Enabled = false;
  bool hcsr04Selected = true;
  int hcsr04TrigPin = 4;
  int hcsr04EchoPin = 16;
  bool oledEnabled = true;
  uint8_t oledTextSize = 1;
  bool oledShowTempc = true;
  bool oledShowTempf = false;
  bool oledShowHum = true;
  bool oledShowVolt = false;
  bool oledShowBatt = true;
  bool oledShowSoilRaw = false;
  bool oledShowSoilPct = true;
  bool oledShowDist = true;
  String oledSrcTempc = "auto";
  String oledSrcTempf = "auto";
  String oledSrcHum = "auto";
  String oledSrcVolt = "auto";
  String oledSrcBatt = "auto";
  String oledSrcSoilRaw = "auto";
  String oledSrcSoilPct = "auto";
  String oledSrcDist = "auto";

  // I2C pins (optional override; -1 uses board defaults)
  int i2cSdaPin = -1;
  int i2cSclPin = -1;

  // Power saving: deep sleep duty-cycle mode
  bool powerSleepEnabled = false;
  uint32_t powerSleepSeconds = 600;    // sleep duration between cycles
  uint32_t powerBleScanSeconds = 5;    // BLE scan duration per cycle
  uint32_t powerWifiWaitSeconds = 8;   // how long to wait for WiFi before giving up
  uint32_t powerBootGraceSeconds = 30; // after power-on/reset, wait this long before first sleep cycle (0=never)
  uint32_t powerAwakeHoldSeconds = 5;  // keep device awake this long after a cycle (for web access)
  // Serial-based sleep blocking removed (use reset to enter portal/config).

  // Watchdog timer
  uint32_t wdtTimeoutSeconds = 30;     // task watchdog timeout (10-300s)

  // Battery (VBAT)
  // mode: 0=off, 1=adc, 2=pmic (Cardputer)
  uint8_t vbatMode =
#if NETTEMP_CARDPUTER_UI
    2
#else
    1
#endif
    ;
  bool vbatSelected = true;
  int vbatAdcPin = 36;             // ESP32 classic: GPIO36 (ADC1); on ESP32-S3 this pin doesn't exist
  uint16_t vbatDividerX1000 = 2000; // divider ratio (VBAT = ADC * ratio). 2000 => 2.0 (e.g. 100k/100k)
  uint16_t vbatFullMv = 4200;      // 4.2V full (18650)
  uint16_t vbatEmptyMv = 3200;     // 3.2V empty (tunable)
  bool vbatSendVolt = false;       // send voltage to Server/MQTT
};

struct SensorRow {
  String mac;
  float temperatureC = NAN;
  float humidityPct = NAN;
  int batteryPct = -1;
  int voltageMv = -1;
  int counter = -1;
  int flags = -1;
  int rssi = 0;
  uint32_t lastSeenMs = 0;
  bool selected = false;
  bool selectionLocked = false; // manual override (persisted via preferences)
  uint32_t lastMqttSentMs = 0;
  String lastAdvHex;
  String lastAdvSrc; // "service" | "mfg"
  String lastAdvUuid; // optional: service-data UUID string
};

Preferences g_prefs;
AppSettings g_cfg;

// BLE → Server (Cloud) field selection (what readings we include in POST /api/v1/data).
// Defaults: tempc + hum + batt
constexpr uint32_t SRV_BLE_TEMPC = 1u << 0;
constexpr uint32_t SRV_BLE_TEMPF = 1u << 1;
constexpr uint32_t SRV_BLE_HUM = 1u << 2;
constexpr uint32_t SRV_BLE_BATT = 1u << 3;
constexpr uint32_t SRV_BLE_VOLT = 1u << 4;
constexpr uint32_t SRV_BLE_RSSI = 1u << 5;
uint32_t g_srvBleFields = (SRV_BLE_TEMPC | SRV_BLE_HUM | SRV_BLE_BATT);
uint32_t g_mqttBleFields = (SRV_BLE_TEMPC | SRV_BLE_HUM | SRV_BLE_BATT);
uint32_t g_webhookBleFields = (SRV_BLE_TEMPC | SRV_BLE_HUM | SRV_BLE_BATT);
uint32_t g_localBleFields = (SRV_BLE_TEMPC | SRV_BLE_HUM | SRV_BLE_BATT);

// I2C → Server (Cloud) field selection.
// Defaults: tempc + hum + press_hpa
constexpr uint32_t SRV_I2C_TEMPC = 1u << 0;
constexpr uint32_t SRV_I2C_HUM = 1u << 1;
constexpr uint32_t SRV_I2C_PRESS = 1u << 2;
constexpr uint32_t SRV_I2C_LUX = 1u << 3;
constexpr uint32_t SRV_I2C_DIST = 1u << 4;
uint32_t g_srvI2cFields = (SRV_I2C_TEMPC | SRV_I2C_HUM | SRV_I2C_PRESS | SRV_I2C_LUX | SRV_I2C_DIST);
uint32_t g_mqttI2cFields = (SRV_I2C_TEMPC | SRV_I2C_HUM | SRV_I2C_PRESS | SRV_I2C_LUX | SRV_I2C_DIST);
uint32_t g_webhookI2cFields = (SRV_I2C_TEMPC | SRV_I2C_HUM | SRV_I2C_PRESS | SRV_I2C_LUX | SRV_I2C_DIST);
uint32_t g_localI2cFields = (SRV_I2C_TEMPC | SRV_I2C_HUM | SRV_I2C_PRESS | SRV_I2C_LUX | SRV_I2C_DIST);

// GPIO → Server/Local/MQTT/Webhook field selection.
// Defaults: all GPIO fields enabled.
constexpr uint32_t SRV_GPIO_TEMPC = 1u << 0;
constexpr uint32_t SRV_GPIO_TEMPF = 1u << 1;
constexpr uint32_t SRV_GPIO_HUM = 1u << 2;
constexpr uint32_t SRV_GPIO_BATT = 1u << 3;
constexpr uint32_t SRV_GPIO_VOLT = 1u << 4;
constexpr uint32_t SRV_GPIO_SOIL_RAW = 1u << 5;
constexpr uint32_t SRV_GPIO_SOIL_PCT = 1u << 6;
constexpr uint32_t SRV_GPIO_DIST = 1u << 7;
constexpr uint32_t SRV_GPIO_DEFAULT =
  (SRV_GPIO_TEMPC | SRV_GPIO_HUM | SRV_GPIO_BATT | SRV_GPIO_VOLT |
   SRV_GPIO_SOIL_RAW | SRV_GPIO_SOIL_PCT | SRV_GPIO_DIST);
constexpr uint32_t SRV_GPIO_DEFAULT_LOCAL = SRV_GPIO_DEFAULT & ~SRV_GPIO_VOLT;
uint32_t g_srvGpioFields = SRV_GPIO_DEFAULT;
uint32_t g_mqttGpioFields = SRV_GPIO_DEFAULT;
uint32_t g_webhookGpioFields = SRV_GPIO_DEFAULT;
uint32_t g_localGpioFields = SRV_GPIO_DEFAULT_LOCAL;

#if NETTEMP_ENABLE_BLE
std::vector<SensorRow> g_sensors;
bool g_bleAutoScan = true;
bool g_activeScan = false;
bool g_bleStrictMode = false;  // If true, only track selected sensors (ignore all others)
int g_bleCursor = 0;
uint8_t g_bleMaxResults = 50;
bool g_bleParseMfg = NETTEMP_BLE_PARSE_MFG;

// Periodic BLE scan timing (5s scan / 55s pause = 60s total cycle)
constexpr uint32_t BLE_PERIODIC_SCAN_MS = 15000;     // Scan for 15 seconds
constexpr uint32_t BLE_PERIODIC_PAUSE_MS = 55000;   // Pause for 55 seconds
uint32_t g_bleLastScanStartMs = 0;
bool g_bleScanPhaseActive = false;  // true=scanning, false=paused
#else
// BLE disabled - provide minimal stubs
std::vector<SensorRow> g_sensors;  // Keep empty vector for compatibility
bool g_bleAutoScan = false;
int g_bleCursor = 0;
#endif

#if NETTEMP_ENABLE_I2C
std::vector<I2cSensorInfo> g_i2cSensors;
std::vector<uint8_t> g_i2cDetectedAddrs;
std::vector<I2cSensorInfo> g_i2cCachedSensors;
int g_i2cCursor = 0;
uint32_t g_lastI2cPollMs = 0;

// Persisted I2C selection by address:
// - g_i2cSelDefined=false => no stored selection (keep defaults)
// - g_i2cSelDefined=true and g_i2cSelAddrs empty => stored "none selected"
// - otherwise => selected addresses
bool g_i2cSelDefined = false;
std::vector<uint8_t> g_i2cSelAddrs;
#endif

// 1-Wire DS18B20 (can be multiple sensors on one pin)
OneWire* g_oneWire = nullptr;
std::vector<std::array<uint8_t, 8>> g_dsRoms;
std::vector<float> g_dsTempsC;
uint32_t g_dsLastSeenMs = 0;
uint32_t g_dsLastConvertStartMs = 0;
bool g_dsConvertInProgress = false;
String g_dsLastReadStatus = "";

// DHT11/DHT22
float g_dhtTempC = NAN;
float g_dhtHumPct = NAN;
uint32_t g_dhtLastSeenMs = 0;
uint32_t g_dhtLastReadMs = 0;
uint32_t g_lastMqttLocalSentMs = 0;
int g_soilRaw = -1;
float g_soilPct = NAN;
uint32_t g_soilLastSeenMs = 0;
float g_hcsr04Cm = NAN;
uint32_t g_hcsr04LastSeenMs = 0;
uint32_t g_hcsr04LastReadMs = 0;

float g_vbatVolts = NAN;
int g_vbatPct = -1;
uint32_t g_vbatLastSeenMs = 0;

WiFiClient g_wifiClient;
#if NETTEMP_ENABLE_MQTT
PubSubClient g_mqtt(g_wifiClient);
#endif

#if NETTEMP_ENABLE_SERVER
WiFiClientSecure g_tlsClient;
#endif

#if NETTEMP_ENABLE_PORTAL
DNSServer g_dnsServer;
WebServer g_webServer(80);
bool g_portalRunning = false;
bool g_portalAuto = true;
uint32_t g_portalBootMs = 0;
#endif

uint32_t g_powerBootMs = 0;
bool g_powerBootCycleDone = false;
bool g_powerCycleDoneThisBoot = false;

uint32_t g_lastUiMs = 0;
uint32_t g_lastServerSendMs = 0;
uint32_t g_lastLocalServerSendMs = 0;
uint32_t g_lastWebhookSendMs = 0;
uint32_t g_lastSerialRxMs = 0;
bool g_i2cBegun = false;

#if NETTEMP_CARDPUTER_UI
// M5Cardputer: Use Wire1 for external sensors to avoid keyboard conflict (Wire = keyboard on GPIO1/2)
TwoWire* g_i2cBus = &Wire1;
#else
// Standard ESP32: Use Wire for sensors
TwoWire* g_i2cBus = &Wire;
#endif

// Async flags for web-triggered channel sends (prevents blocking web server)
volatile bool g_pendingSendMqtt = false;
volatile bool g_pendingSendServer = false;
volatile bool g_pendingSendLocalServer = false;
volatile bool g_pendingSendWebhook = false;

// Send status tracking for web UI feedback (0=idle, 1=sending, 2=success, 3=failed)
volatile uint8_t g_sendStatusMqtt = 0;
volatile uint8_t g_sendStatusServer = 0;
volatile uint8_t g_sendStatusLocalServer = 0;
volatile uint8_t g_sendStatusWebhook = 0;

// Crash tracking (saved to NVS for debugging panics/reboots)
String g_crashReason;
String g_crashLocation;
uint32_t g_crashUptime = 0;
uint32_t g_crashFreeHeap = 0;
bool g_hasCrashInfo = false;

enum class View {
  MainMenu,
  ShowSensors,
  ScanBle,
  ScanI2c,
  WifiSetup,
  MqttSetup,
  ServerSetup,
  PairQr,
  About,
  Channels,
  ResetConfirm,
  GpioDevices,
  GpioDs18b20,
  GpioHcsr04,
  GpioDht,
};

View g_view = View::MainMenu;
View g_prevView = View::MainMenu;
bool g_viewNeedsRedraw = true;
bool g_viewNeedsClear = true;
int g_menuIndex = 0;
uint32_t g_lastQrTickMs = 0;

// Arduino IDE does not auto-generate prototypes inside namespaces.
static void prefsSave();
static String macNoColonsUpper(const String& mac);
static String macWithColonsUpper(const String& mac);
static String normalizeMacToColonsUpper(String mac);
static std::vector<size_t> buildBleSortedIndex();
static String serverBleFieldsEnabledString();
static String serverBleFieldsAvailableString();
static void tickSendMqtt();
static void tickSendServer();
#if NETTEMP_ENABLE_SERVER
static void tickSendLocalServer();
static void tickSendWebhook();
#endif
static void mqttEnsureConnected();
static String serverI2cFieldsEnabledString();
static String serverI2cFieldsAvailableString();
#if NETTEMP_ENABLE_I2C
static void i2cApplySelectionToDetected();
#endif
static void dsEnsureBus();
static void dsRescan();
static void dsTick();
static bool dhtReadOnce(int gpio, int type, float& outTempC, float& outHumPct);
static void dhtTick();
static void wifiConnectIfConfigured();
static void wifiWaitConnected(uint32_t timeoutMs);
#if NETTEMP_ENABLE_PORTAL
static String portalSsid();
#endif
static void powerSleepCycleMaybe();
static void powerEnterDeepSleep(uint32_t sleepSeconds);
static void vbatTick();
static void crashSaveBreadcrumb(const char* reason, const char* location);
static void crashCheckAndLoad();
static void crashClear();
static String crashGetLog();
static bool vbatReadVolts(float& outV);
static int vbatToPercent(float vbatV);

uint32_t g_lastWifiBeginMs = 0;
#if NETTEMP_ENABLE_BLE
uint32_t g_lastBleEnsureMs = 0;
#endif

#if NETTEMP_ENABLE_BLE
struct BleManualSel {
  String macNo;   // A4C138DE459E
  bool selected;  // true=send, false=disabled
};
std::vector<BleManualSel> g_bleManualSel;
String g_bleCacheLast;
uint32_t g_bleCacheLastSaveMs = 0;

static BleManualSel* bleManualFind(const String& macNo) {
  for (auto& e : g_bleManualSel) {
    if (e.macNo == macNo) return &e;
  }
  return nullptr;
}

static bool bleManualGetSelected(const String& macNo, bool& outSelected) {
  const auto* e = bleManualFind(macNo);
  if (!e) return false;
  outSelected = e->selected;
  return true;
}

static bool bleIsSelected(const SensorRow& s) {
  const String macNo = macNoColonsUpper(s.mac);
  bool manualSelected = false;
  const bool manual = bleManualGetSelected(macNo, manualSelected);
  return manual ? manualSelected : s.selected;
}

static void bleManualSet(const String& macNo, bool selected) {
  if (!macNo.length()) return;
  auto* e = bleManualFind(macNo);
  if (e) {
    e->selected = selected;
    return;
  }
  g_bleManualSel.push_back(BleManualSel{.macNo = macNo, .selected = selected});
}

static void bleManualClear(const String& macNo) {
  if (!macNo.length()) return;
  for (size_t i = 0; i < g_bleManualSel.size(); i++) {
    if (g_bleManualSel[i].macNo == macNo) {
      g_bleManualSel.erase(g_bleManualSel.begin() + (int)i);
      return;
    }
  }
}

static bool bleManualHasAnySelected() {
  for (const auto& e : g_bleManualSel) {
    if (e.selected) return true;
  }
  return false;
}

static String bleManualEncode() {
  String out;
  for (size_t i = 0; i < g_bleManualSel.size(); i++) {
    const auto& e = g_bleManualSel[i];
    if (!e.macNo.length()) continue;
    if (out.length()) out += ",";
    out += e.macNo;
    out += ":";
    out += (e.selected ? "1" : "0");
  }
  return out;
}

static void bleManualDecode(const String& s) {
  g_bleManualSel.clear();
  int pos = 0;
  while (pos < (int)s.length()) {
    int comma = s.indexOf(',', pos);
    if (comma < 0) comma = (int)s.length();
    const String part = s.substring(pos, comma);
    pos = comma + 1;
    const int colon = part.indexOf(':');
    if (colon < 0) continue;
    const String macNo = part.substring(0, colon);
    const String val = part.substring(colon + 1);
    if (!macNo.length()) continue;
    const bool selected = val == "1";
    g_bleManualSel.push_back(BleManualSel{.macNo = macNo, .selected = selected});
  }
}

static String bleCacheEncode(size_t maxEntries) {
  String out;
  size_t added = 0;
  for (const auto& s : g_sensors) {
    if (!bleIsSelected(s)) continue;
    if (added >= maxEntries) break;
    if (out.length()) out += ";";
    out += macNoColonsUpper(s.mac);
    out += ",";
    if (!isnan(s.temperatureC)) out += String(s.temperatureC, 2);
    out += ",";
    if (!isnan(s.humidityPct)) out += String(s.humidityPct, 1);
    out += ",";
    if (s.batteryPct >= 0) out += String(s.batteryPct);
    out += ",";
    if (s.voltageMv >= 0) out += String(s.voltageMv);
    added++;
  }
  return out;
}

static void bleCacheEnsureManualSelected() {
  for (const auto& e : g_bleManualSel) {
    if (!e.selected) continue;
    const String mac = normalizeMacToColonsUpper(e.macNo);
    bool found = false;
    for (const auto& s : g_sensors) {
      if (macWithColonsUpper(s.mac) == mac) {
        found = true;
        break;
      }
    }
    if (found) continue;
    SensorRow row;
    row.mac = mac;
    row.selected = true;
    row.selectionLocked = true;
    row.lastSeenMs = millis();
    g_sensors.push_back(row);
  }
}

static void bleCacheDecode(const String& s) {
  if (!s.length()) return;
  g_sensors.clear();
  int pos = 0;
  while (pos < (int)s.length()) {
    int semi = s.indexOf(';', pos);
    if (semi < 0) semi = (int)s.length();
    const String part = s.substring(pos, semi);
    pos = semi + 1;
    if (!part.length()) continue;

    String fields[5];
    int fpos = 0;
    int idx = 0;
    while (idx < 5 && fpos <= (int)part.length()) {
      int comma = part.indexOf(',', fpos);
      if (comma < 0) comma = (int)part.length();
      fields[idx++] = part.substring(fpos, comma);
      fpos = comma + 1;
    }
    const String macNo = fields[0];
    if (!macNo.length()) continue;
    const String mac = normalizeMacToColonsUpper(macNo);
    SensorRow row;
    row.mac = mac;
    if (fields[1].length()) row.temperatureC = fields[1].toFloat();
    if (fields[2].length()) row.humidityPct = fields[2].toFloat();
    if (fields[3].length()) row.batteryPct = fields[3].toInt();
    if (fields[4].length()) row.voltageMv = fields[4].toInt();
    row.lastSeenMs = millis();
    bool manualSelected = false;
    const bool manual = bleManualGetSelected(macNoColonsUpper(row.mac), manualSelected);
    row.selected = manual ? manualSelected : false;
    row.selectionLocked = manual;
    g_sensors.push_back(row);
  }
  bleCacheEnsureManualSelected();
}

static void bleCachePersistMaybe(bool force = false) {
  const uint32_t now = millis();
  if (!force && (now - g_bleCacheLastSaveMs) < 30000) return;
  const size_t maxEntries = (g_bleMaxResults > 0) ? (size_t)g_bleMaxResults : 50;
  const String encoded = bleCacheEncode(maxEntries);
  if (encoded != g_bleCacheLast) {
    g_prefs.begin("nettemp", false);
    g_prefs.putString("bleCache", encoded);
    g_prefs.end();
    g_bleCacheLast = encoded;
  }
  g_bleCacheLastSaveMs = now;
}
#else
// BLE disabled - provide stub functions
static void bleManualSet(const String&, bool) {}
static void bleManualClear(const String&) {}
static String bleManualEncode() { return ""; }
static void bleManualDecode(const String&) {}
static bool bleManualGetSelected(const String&, bool&) { return false; }
static bool bleIsSelected(const SensorRow&) { return false; }
static bool bleManualHasAnySelected() { return false; }
#endif  // NETTEMP_ENABLE_BLE

#if NETTEMP_ENABLE_I2C
static String i2cSerializeSensors(const std::vector<I2cSensorInfo>& sensors) {
  String out;
  for (const auto& s : sensors) {
    if (out.length()) out += ",";
    if (s.address < 16) out += "0";
    out += String(s.address, HEX);
    out += ":";
    out += String((int)s.type);
  }
  out.toLowerCase();
  return out;
}

static void i2cLoadCachedSensors(const String& s) {
  g_i2cCachedSensors.clear();
  int pos = 0;
  while (pos < (int)s.length()) {
    int comma = s.indexOf(',', pos);
    if (comma < 0) comma = (int)s.length();
    const String part = s.substring(pos, comma);
    pos = comma + 1;
    if (!part.length()) continue;
    const int colon = part.indexOf(':');
    if (colon <= 0) continue;
    const String addrStr = part.substring(0, colon);
    const String typeStr = part.substring(colon + 1);
    const uint8_t addr = (uint8_t)strtoul(addrStr.c_str(), nullptr, 16);
    const int typeVal = typeStr.toInt();
    I2cSensorInfo info;
    info.address = addr;
    info.type = (I2cSensorType)typeVal;
    info.selected = true;
    g_i2cCachedSensors.push_back(info);
  }
}

static void i2cPersistLastDetected() {
  if (g_i2cSensors.empty()) return;
  const String now = i2cSerializeSensors(g_i2cSensors);
  if (!now.length()) return;
  g_prefs.begin("nettemp", false);
  const String prev = g_prefs.getString("i2cLast", "");
  if (prev != now) g_prefs.putString("i2cLast", now);
  g_prefs.end();
}
#endif

static void prefsLoad() {
  g_prefs.begin("nettemp", true);
  g_cfg.deviceId = g_prefs.getString("deviceId", g_cfg.deviceId);
  g_cfg.wifiSsid = g_prefs.getString("wifiSsid", "");
  g_cfg.wifiPass = g_prefs.getString("wifiPass", "");
  g_cfg.webUser = g_prefs.getString("webUser", g_cfg.webUser);
  g_cfg.webPass = g_prefs.getString("webPass", "");

  g_cfg.mqttEnabled = g_prefs.getBool("mqttEn", false);
  g_cfg.mqttHost = g_prefs.getString("mqttHost", g_cfg.mqttHost);
  g_cfg.mqttPort = g_prefs.getUShort("mqttPort", g_cfg.mqttPort);
  g_cfg.mqttUser = g_prefs.getString("mqttUser", "");
  g_cfg.mqttPass = g_prefs.getString("mqttPass", "");
  g_cfg.mqttIntervalMs = g_prefs.getUInt("mqttInt", g_cfg.mqttIntervalMs);

  g_cfg.serverEnabled = g_prefs.getBool("srvEn", false);
  g_cfg.serverBaseUrl = g_prefs.getString("srvUrl", g_cfg.serverBaseUrl);
  g_cfg.serverApiKey = g_prefs.getString("srvKey", "");
  g_cfg.serverIntervalMs = g_prefs.getUInt("srvInt", g_cfg.serverIntervalMs);
  g_cfg.localServerEnabled = g_prefs.getBool("lsEn", g_cfg.localServerEnabled);
  g_cfg.localServerUrl = g_prefs.getString("lsUrl", g_cfg.localServerUrl);
  g_cfg.localServerApiKey = g_prefs.getString("lsKey", g_cfg.localServerApiKey);
  g_cfg.localServerIntervalMs = g_prefs.getUInt("lsInt", g_cfg.localServerIntervalMs);
  g_cfg.webhookEnabled = g_prefs.getBool("whEn", g_cfg.webhookEnabled);
  g_cfg.webhookUrl = g_prefs.getString("whUrl", g_cfg.webhookUrl);
  g_cfg.webhookIntervalMs = g_prefs.getUInt("whInt", g_cfg.webhookIntervalMs);
  g_cfg.bleSendServer = g_prefs.getBool("bleSrv", g_cfg.bleSendServer);
  g_cfg.bleSendMqtt = g_prefs.getBool("bleMq", g_cfg.bleSendMqtt);
  g_cfg.bleSendWebhook = g_prefs.getBool("bleWh", g_cfg.bleSendWebhook);
  g_cfg.bleSendLocalServer = g_prefs.getBool("bleLs", g_cfg.bleSendLocalServer);
  g_cfg.i2cSendServer = g_prefs.getBool("i2cSrv", g_cfg.i2cSendServer);
  g_cfg.i2cSendMqtt = g_prefs.getBool("i2cMq", g_cfg.i2cSendMqtt);
  g_cfg.i2cSendWebhook = g_prefs.getBool("i2cWh", g_cfg.i2cSendWebhook);
  g_cfg.i2cSendLocalServer = g_prefs.getBool("i2cLs", g_cfg.i2cSendLocalServer);
  g_cfg.gpioSendServer = g_prefs.getBool("gpioSrv", g_cfg.gpioSendServer);
  g_cfg.gpioSendMqtt = g_prefs.getBool("gpioMq", g_cfg.gpioSendMqtt);
  g_cfg.gpioSendWebhook = g_prefs.getBool("gpioWh", g_cfg.gpioSendWebhook);
  g_cfg.gpioSendLocalServer = g_prefs.getBool("gpioLs", g_cfg.gpioSendLocalServer);
  if (g_cfg.webhookIntervalMs < 5000) g_cfg.webhookIntervalMs = 5000;
  if (g_cfg.webhookIntervalMs > 3600000) g_cfg.webhookIntervalMs = 3600000;
  g_srvBleFields = g_prefs.getUInt("srvBleF", g_srvBleFields);
  if (g_srvBleFields == 0) g_srvBleFields = (SRV_BLE_TEMPC | SRV_BLE_HUM | SRV_BLE_BATT);
  g_srvI2cFields = g_prefs.getUInt("srvI2cF", g_srvI2cFields);
  if (g_srvI2cFields == 0) g_srvI2cFields = (SRV_I2C_TEMPC | SRV_I2C_HUM | SRV_I2C_PRESS | SRV_I2C_LUX | SRV_I2C_DIST);
  g_mqttBleFields = g_prefs.getUInt("mqBleF", g_mqttBleFields);
  if (g_mqttBleFields == 0) g_mqttBleFields = (SRV_BLE_TEMPC | SRV_BLE_HUM | SRV_BLE_BATT);
  g_mqttI2cFields = g_prefs.getUInt("mqI2cF", g_mqttI2cFields);
  if (g_mqttI2cFields == 0) g_mqttI2cFields = (SRV_I2C_TEMPC | SRV_I2C_HUM | SRV_I2C_PRESS | SRV_I2C_LUX | SRV_I2C_DIST);
  g_webhookBleFields = g_prefs.getUInt("whBleF", g_webhookBleFields);
  if (g_webhookBleFields == 0) g_webhookBleFields = (SRV_BLE_TEMPC | SRV_BLE_HUM | SRV_BLE_BATT);
  g_webhookI2cFields = g_prefs.getUInt("whI2cF", g_webhookI2cFields);
  if (g_webhookI2cFields == 0) g_webhookI2cFields = (SRV_I2C_TEMPC | SRV_I2C_HUM | SRV_I2C_PRESS | SRV_I2C_LUX | SRV_I2C_DIST);
  g_localBleFields = g_prefs.getUInt("lsBleF", g_localBleFields);
  if (g_localBleFields == 0) g_localBleFields = (SRV_BLE_TEMPC | SRV_BLE_HUM | SRV_BLE_BATT);
  g_localI2cFields = g_prefs.getUInt("lsI2cF", g_localI2cFields);
  if (g_localI2cFields == 0) g_localI2cFields = (SRV_I2C_TEMPC | SRV_I2C_HUM | SRV_I2C_PRESS | SRV_I2C_LUX | SRV_I2C_DIST);
  g_srvGpioFields = g_prefs.getUInt("srvGpioF", g_srvGpioFields);
  if (g_srvGpioFields == 0) g_srvGpioFields = SRV_GPIO_DEFAULT;
  g_mqttGpioFields = g_prefs.getUInt("mqGpioF", g_mqttGpioFields);
  if (g_mqttGpioFields == 0) g_mqttGpioFields = SRV_GPIO_DEFAULT;
  g_webhookGpioFields = g_prefs.getUInt("whGpioF", g_webhookGpioFields);
  if (g_webhookGpioFields == 0) g_webhookGpioFields = SRV_GPIO_DEFAULT;
  g_localGpioFields = g_prefs.getUInt("lsGpioF", g_localGpioFields);
  if (g_localGpioFields == 0) g_localGpioFields = SRV_GPIO_DEFAULT_LOCAL;
  bleManualDecode(g_prefs.getString("bleMan", ""));
#if NETTEMP_ENABLE_BLE
  {
    const String cache = g_prefs.getString("bleCache", "");
    g_bleCacheLast = cache;
    bleCacheDecode(cache);
  }
  bleCacheEnsureManualSelected();
#endif
#if NETTEMP_ENABLE_BLE
  if (bleManualHasAnySelected()) g_bleStrictMode = true;
#endif

  g_cfg.dsEnabled = g_prefs.getBool("dsEn", g_cfg.dsEnabled);
  g_cfg.dsPin = (int)g_prefs.getInt("dsPin", g_cfg.dsPin);
  g_cfg.dhtEnabled = g_prefs.getBool("dhtEn", g_cfg.dhtEnabled);
  g_cfg.dhtSelected = g_prefs.getBool("dhtSel", g_cfg.dhtSelected);
  g_cfg.dhtPin = (int)g_prefs.getInt("dhtPin", g_cfg.dhtPin);
  g_cfg.dhtType = (int)g_prefs.getInt("dhtType", g_cfg.dhtType);
  g_cfg.soilEnabled = g_prefs.getBool("soilEn", g_cfg.soilEnabled);
  g_cfg.soilSelected = g_prefs.getBool("soilSel", g_cfg.soilSelected);
  g_cfg.soilAdcPin = (int)g_prefs.getInt("soilPin", g_cfg.soilAdcPin);
  g_cfg.soilDryRaw = (int)g_prefs.getInt("soilDry", g_cfg.soilDryRaw);
  g_cfg.soilWetRaw = (int)g_prefs.getInt("soilWet", g_cfg.soilWetRaw);
  g_cfg.hcsr04Enabled = g_prefs.getBool("hcEn", g_cfg.hcsr04Enabled);
  g_cfg.hcsr04Selected = g_prefs.getBool("hcSel", g_cfg.hcsr04Selected);
  g_cfg.hcsr04TrigPin = (int)g_prefs.getInt("hcTrig", g_cfg.hcsr04TrigPin);
  g_cfg.hcsr04EchoPin = (int)g_prefs.getInt("hcEcho", g_cfg.hcsr04EchoPin);
  g_cfg.oledEnabled = g_prefs.getBool("oledEn", g_cfg.oledEnabled);
  g_cfg.oledTextSize = (uint8_t)g_prefs.getUChar("oledSz", g_cfg.oledTextSize);
  if (g_cfg.oledTextSize < 1 || g_cfg.oledTextSize > 3) g_cfg.oledTextSize = 1;
  g_cfg.oledShowTempc = g_prefs.getBool("oledTc", g_cfg.oledShowTempc);
  g_cfg.oledShowTempf = g_prefs.getBool("oledTf", g_cfg.oledShowTempf);
  g_cfg.oledShowHum = g_prefs.getBool("oledHum", g_cfg.oledShowHum);
  g_cfg.oledShowVolt = g_prefs.getBool("oledVolt", g_cfg.oledShowVolt);
  g_cfg.oledShowBatt = g_prefs.getBool("oledBatt", g_cfg.oledShowBatt);
  g_cfg.oledShowSoilRaw = g_prefs.getBool("oledSRaw", g_cfg.oledShowSoilRaw);
  g_cfg.oledShowSoilPct = g_prefs.getBool("oledSPct", g_cfg.oledShowSoilPct);
  g_cfg.oledShowDist = g_prefs.getBool("oledDist", g_cfg.oledShowDist);
  g_cfg.oledSrcTempc = g_prefs.getString("oledSrcTc", g_cfg.oledSrcTempc);
  g_cfg.oledSrcTempf = g_prefs.getString("oledSrcTf", g_cfg.oledSrcTempf);
  g_cfg.oledSrcHum = g_prefs.getString("oledSrcHum", g_cfg.oledSrcHum);
  g_cfg.oledSrcVolt = g_prefs.getString("oledSrcVolt", g_cfg.oledSrcVolt);
  g_cfg.oledSrcBatt = g_prefs.getString("oledSrcBatt", g_cfg.oledSrcBatt);
  g_cfg.oledSrcSoilRaw = g_prefs.getString("oledSrcSRaw", g_cfg.oledSrcSoilRaw);
  g_cfg.oledSrcSoilPct = g_prefs.getString("oledSrcSPct", g_cfg.oledSrcSoilPct);
  g_cfg.oledSrcDist = g_prefs.getString("oledSrcDist", g_cfg.oledSrcDist);
  if (!g_cfg.oledSrcTempc.length()) g_cfg.oledSrcTempc = "auto";
  if (!g_cfg.oledSrcTempf.length()) g_cfg.oledSrcTempf = "auto";
  if (!g_cfg.oledSrcHum.length()) g_cfg.oledSrcHum = "auto";
  if (!g_cfg.oledSrcVolt.length()) g_cfg.oledSrcVolt = "auto";
  if (!g_cfg.oledSrcBatt.length()) g_cfg.oledSrcBatt = "auto";
  if (!g_cfg.oledSrcSoilRaw.length()) g_cfg.oledSrcSoilRaw = "auto";
  if (!g_cfg.oledSrcSoilPct.length()) g_cfg.oledSrcSoilPct = "auto";
  if (!g_cfg.oledSrcDist.length()) g_cfg.oledSrcDist = "auto";
  if (g_cfg.dhtType != 11 && g_cfg.dhtType != 22) g_cfg.dhtType = 22;

  g_cfg.i2cSdaPin = (int)g_prefs.getInt("i2cSda", g_cfg.i2cSdaPin);
  g_cfg.i2cSclPin = (int)g_prefs.getInt("i2cScl", g_cfg.i2cSclPin);

  g_cfg.powerSleepEnabled = g_prefs.getBool("pwrSlp", g_cfg.powerSleepEnabled);
  g_cfg.powerSleepSeconds = g_prefs.getUInt("pwrSlpS", g_cfg.powerSleepSeconds);
  g_cfg.powerBleScanSeconds = g_prefs.getUInt("pwrBleS", g_cfg.powerBleScanSeconds);
  g_cfg.powerWifiWaitSeconds = g_prefs.getUInt("pwrWiS", g_cfg.powerWifiWaitSeconds);
  g_cfg.powerBootGraceSeconds = g_prefs.getUInt("pwrBootS", g_cfg.powerBootGraceSeconds);
  g_cfg.powerAwakeHoldSeconds = g_prefs.getUInt("pwrHoldS", g_cfg.powerAwakeHoldSeconds);
  if (g_cfg.powerSleepSeconds < 10) g_cfg.powerSleepSeconds = 10;
  if (g_cfg.powerSleepSeconds > 86400) g_cfg.powerSleepSeconds = 86400;
  if (g_cfg.powerBleScanSeconds < 1) g_cfg.powerBleScanSeconds = 1;
  if (g_cfg.powerBleScanSeconds > 30) g_cfg.powerBleScanSeconds = 30;
  if (g_cfg.powerWifiWaitSeconds < 2) g_cfg.powerWifiWaitSeconds = 2;
  if (g_cfg.powerWifiWaitSeconds > 30) g_cfg.powerWifiWaitSeconds = 30;
  if (g_cfg.powerBootGraceSeconds > 600) g_cfg.powerBootGraceSeconds = 600;
  if (g_cfg.powerAwakeHoldSeconds > 120) g_cfg.powerAwakeHoldSeconds = 120;

  g_cfg.wdtTimeoutSeconds = g_prefs.getUInt("wdtTO", g_cfg.wdtTimeoutSeconds);
  if (g_cfg.wdtTimeoutSeconds < 10) g_cfg.wdtTimeoutSeconds = 10;
  if (g_cfg.wdtTimeoutSeconds > 300) g_cfg.wdtTimeoutSeconds = 300;

  g_cfg.vbatMode = (uint8_t)g_prefs.getUChar("vbMode", g_cfg.vbatMode);
#if NETTEMP_CARDPUTER_UI
  if (!g_prefs.isKey("vbMode")) g_cfg.vbatMode = 2;
#endif
  g_cfg.vbatSelected = g_prefs.getBool("vbSel", g_cfg.vbatSelected);
  g_cfg.vbatAdcPin = (int)g_prefs.getInt("vbPin", g_cfg.vbatAdcPin);
  g_cfg.vbatDividerX1000 = (uint16_t)g_prefs.getUShort("vbDiv", g_cfg.vbatDividerX1000);
  g_cfg.vbatFullMv = (uint16_t)g_prefs.getUShort("vbFull", g_cfg.vbatFullMv);
  g_cfg.vbatEmptyMv = (uint16_t)g_prefs.getUShort("vbEmp", g_cfg.vbatEmptyMv);
  g_cfg.vbatSendVolt = g_prefs.getBool("vbSV", g_cfg.vbatSendVolt);
  if (g_cfg.vbatMode > 2) g_cfg.vbatMode = 0;
#if CONFIG_IDF_TARGET_ESP32S3
  // ESP32-S3 ADC path is incompatible; allow PMIC mode only.
  if (g_cfg.vbatMode == 1) g_cfg.vbatMode = 0;
#endif
  if (g_cfg.vbatDividerX1000 < 500) g_cfg.vbatDividerX1000 = 500;
  if (g_cfg.vbatDividerX1000 > 5000) g_cfg.vbatDividerX1000 = 5000;
  if (g_cfg.vbatFullMv < 3600) g_cfg.vbatFullMv = 4200;
  if (g_cfg.vbatFullMv > 5000) g_cfg.vbatFullMv = 4200;
  if (g_cfg.vbatEmptyMv < 2500) g_cfg.vbatEmptyMv = 3200;
  if (g_cfg.vbatEmptyMv > 4200) g_cfg.vbatEmptyMv = 3200;

#if NETTEMP_ENABLE_BLE
  g_bleAutoScan = g_prefs.getBool("bleAuto", true);
  g_activeScan = g_prefs.getBool("bleAct", false);
  g_bleStrictMode = g_prefs.getBool("bleStrict", false);
  {
    const uint32_t v = g_prefs.getUInt("bleMax", (uint32_t)g_bleMaxResults);
    if (v < 5) g_bleMaxResults = 5;
    else if (v > 200) g_bleMaxResults = 200;
    else g_bleMaxResults = (uint8_t)v;
  }
  g_bleParseMfg = g_prefs.getBool("bleMfg", g_bleParseMfg);
#endif
#if NETTEMP_ENABLE_PORTAL
  g_portalAuto = g_prefs.getBool("apAuto", g_portalAuto);
#endif
#if NETTEMP_ENABLE_I2C
  {
    const String s = g_prefs.getString("i2cSel", "");
    g_i2cSelDefined = false;
    g_i2cSelAddrs.clear();
    if (s.length()) {
      g_i2cSelDefined = true;
      if (s != "-") {
        int pos = 0;
        while (pos < (int)s.length()) {
          int comma = s.indexOf(',', pos);
          if (comma < 0) comma = (int)s.length();
          const String part = s.substring(pos, comma);
          pos = comma + 1;
          if (!part.length()) continue;
          const uint8_t addr = (uint8_t)strtoul(part.c_str(), nullptr, 16);
          g_i2cSelAddrs.push_back(addr);
        }
      }
    }
  }
  i2cLoadCachedSensors(g_prefs.getString("i2cLast", ""));
#endif
  g_prefs.end();

#if NETTEMP_ENABLE_SERVER
  g_tlsClient.setInsecure(); // simplest for embedded; use CA pinning if you want stricter TLS
#endif
}

static char randomTokenChar(uint8_t v) {
  // Allowed by frontend validator: [A-Za-z0-9_]
  static const char kAlphabet[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789_";
  return kAlphabet[v % (sizeof(kAlphabet) - 1)];
}

static String generateNtkToken() {
  String t = "ntk_";
  t.reserve(4 + 64);
  for (int i = 0; i < 64; i++) {
    const uint32_t r = (uint32_t)esp_random();
    t += randomTokenChar((uint8_t)(r & 0xFF));
  }
  return t;
}

static void ensureServerApiKey() {
  if (g_cfg.serverApiKey.length() > 0) return;
  g_cfg.serverApiKey = generateNtkToken();
  prefsSave();
}

static void prefsSave() {
  g_prefs.begin("nettemp", false);
  g_prefs.putString("deviceId", g_cfg.deviceId);
  g_prefs.putString("wifiSsid", g_cfg.wifiSsid);
  g_prefs.putString("wifiPass", g_cfg.wifiPass);
  g_prefs.putString("webUser", g_cfg.webUser);
  g_prefs.putString("webPass", g_cfg.webPass);

  g_prefs.putBool("mqttEn", g_cfg.mqttEnabled);
  g_prefs.putString("mqttHost", g_cfg.mqttHost);
  g_prefs.putUShort("mqttPort", g_cfg.mqttPort);
  g_prefs.putString("mqttUser", g_cfg.mqttUser);
  g_prefs.putString("mqttPass", g_cfg.mqttPass);
  g_prefs.putUInt("mqttInt", g_cfg.mqttIntervalMs);

  g_prefs.putBool("srvEn", g_cfg.serverEnabled);
  g_prefs.putString("srvUrl", g_cfg.serverBaseUrl);
  g_prefs.putString("srvKey", g_cfg.serverApiKey);
  g_prefs.putUInt("srvInt", g_cfg.serverIntervalMs);
  g_prefs.putBool("lsEn", g_cfg.localServerEnabled);
  g_prefs.putString("lsUrl", g_cfg.localServerUrl);
  g_prefs.putString("lsKey", g_cfg.localServerApiKey);
  g_prefs.putUInt("lsInt", g_cfg.localServerIntervalMs);
  g_prefs.putBool("whEn", g_cfg.webhookEnabled);
  g_prefs.putString("whUrl", g_cfg.webhookUrl);
  g_prefs.putUInt("whInt", g_cfg.webhookIntervalMs);
  g_prefs.putBool("bleSrv", g_cfg.bleSendServer);
  g_prefs.putBool("bleMq", g_cfg.bleSendMqtt);
  g_prefs.putBool("bleWh", g_cfg.bleSendWebhook);
  g_prefs.putBool("bleLs", g_cfg.bleSendLocalServer);
  g_prefs.putBool("i2cSrv", g_cfg.i2cSendServer);
  g_prefs.putBool("i2cMq", g_cfg.i2cSendMqtt);
  g_prefs.putBool("i2cWh", g_cfg.i2cSendWebhook);
  g_prefs.putBool("i2cLs", g_cfg.i2cSendLocalServer);
  g_prefs.putBool("gpioSrv", g_cfg.gpioSendServer);
  g_prefs.putBool("gpioMq", g_cfg.gpioSendMqtt);
  g_prefs.putBool("gpioWh", g_cfg.gpioSendWebhook);
  g_prefs.putBool("gpioLs", g_cfg.gpioSendLocalServer);
  g_prefs.putUInt("srvBleF", g_srvBleFields);
  g_prefs.putUInt("srvI2cF", g_srvI2cFields);
  g_prefs.putUInt("mqBleF", g_mqttBleFields);
  g_prefs.putUInt("mqI2cF", g_mqttI2cFields);
  g_prefs.putUInt("whBleF", g_webhookBleFields);
  g_prefs.putUInt("whI2cF", g_webhookI2cFields);
  g_prefs.putUInt("lsBleF", g_localBleFields);
  g_prefs.putUInt("lsI2cF", g_localI2cFields);
  g_prefs.putUInt("srvGpioF", g_srvGpioFields);
  g_prefs.putUInt("mqGpioF", g_mqttGpioFields);
  g_prefs.putUInt("whGpioF", g_webhookGpioFields);
  g_prefs.putUInt("lsGpioF", g_localGpioFields);
  g_prefs.putString("bleMan", bleManualEncode());
#if NETTEMP_ENABLE_I2C
  if (!g_i2cSensors.empty()) {
    g_prefs.putString("i2cLast", i2cSerializeSensors(g_i2cSensors));
  }
#endif

  g_prefs.putBool("dsEn", g_cfg.dsEnabled);
  g_prefs.putInt("dsPin", g_cfg.dsPin);
  g_prefs.putBool("dhtEn", g_cfg.dhtEnabled);
  g_prefs.putBool("dhtSel", g_cfg.dhtSelected);
  g_prefs.putInt("dhtPin", g_cfg.dhtPin);
  g_prefs.putInt("dhtType", g_cfg.dhtType);
  g_prefs.putBool("soilEn", g_cfg.soilEnabled);
  g_prefs.putBool("soilSel", g_cfg.soilSelected);
  g_prefs.putInt("soilPin", g_cfg.soilAdcPin);
  g_prefs.putInt("soilDry", g_cfg.soilDryRaw);
  g_prefs.putInt("soilWet", g_cfg.soilWetRaw);
  g_prefs.putBool("hcEn", g_cfg.hcsr04Enabled);
  g_prefs.putBool("hcSel", g_cfg.hcsr04Selected);
  g_prefs.putInt("hcTrig", g_cfg.hcsr04TrigPin);
  g_prefs.putInt("hcEcho", g_cfg.hcsr04EchoPin);
  g_prefs.putBool("oledEn", g_cfg.oledEnabled);
  g_prefs.putUChar("oledSz", g_cfg.oledTextSize);
  g_prefs.putBool("oledTc", g_cfg.oledShowTempc);
  g_prefs.putBool("oledTf", g_cfg.oledShowTempf);
  g_prefs.putBool("oledHum", g_cfg.oledShowHum);
  g_prefs.putBool("oledVolt", g_cfg.oledShowVolt);
  g_prefs.putBool("oledBatt", g_cfg.oledShowBatt);
  g_prefs.putBool("oledSRaw", g_cfg.oledShowSoilRaw);
  g_prefs.putBool("oledSPct", g_cfg.oledShowSoilPct);
  g_prefs.putBool("oledDist", g_cfg.oledShowDist);
  g_prefs.putString("oledSrcTc", g_cfg.oledSrcTempc);
  g_prefs.putString("oledSrcTf", g_cfg.oledSrcTempf);
  g_prefs.putString("oledSrcHum", g_cfg.oledSrcHum);
  g_prefs.putString("oledSrcVolt", g_cfg.oledSrcVolt);
  g_prefs.putString("oledSrcBatt", g_cfg.oledSrcBatt);
  g_prefs.putString("oledSrcSRaw", g_cfg.oledSrcSoilRaw);
  g_prefs.putString("oledSrcSPct", g_cfg.oledSrcSoilPct);
  g_prefs.putString("oledSrcDist", g_cfg.oledSrcDist);

  g_prefs.putInt("i2cSda", g_cfg.i2cSdaPin);
  g_prefs.putInt("i2cScl", g_cfg.i2cSclPin);

  g_prefs.putBool("pwrSlp", g_cfg.powerSleepEnabled);
  g_prefs.putUInt("pwrSlpS", g_cfg.powerSleepSeconds);
  g_prefs.putUInt("pwrBleS", g_cfg.powerBleScanSeconds);
  g_prefs.putUInt("pwrWiS", g_cfg.powerWifiWaitSeconds);
  g_prefs.putUInt("pwrBootS", g_cfg.powerBootGraceSeconds);
  g_prefs.putUInt("pwrHoldS", g_cfg.powerAwakeHoldSeconds);

  g_prefs.putUInt("wdtTO", g_cfg.wdtTimeoutSeconds);

  g_prefs.putUChar("vbMode", g_cfg.vbatMode);
  g_prefs.putInt("vbPin", g_cfg.vbatAdcPin);
  g_prefs.putUShort("vbDiv", g_cfg.vbatDividerX1000);
  g_prefs.putUShort("vbFull", g_cfg.vbatFullMv);
  g_prefs.putUShort("vbEmp", g_cfg.vbatEmptyMv);
  g_prefs.putBool("vbSV", g_cfg.vbatSendVolt);
  g_prefs.putBool("vbSel", g_cfg.vbatSelected);

#if NETTEMP_ENABLE_BLE
  g_prefs.putBool("bleAuto", g_bleAutoScan);
  g_prefs.putBool("bleAct", g_activeScan);
  g_prefs.putBool("bleStrict", g_bleStrictMode);
  g_prefs.putUInt("bleMax", (uint32_t)g_bleMaxResults);
  g_prefs.putBool("bleMfg", g_bleParseMfg);
#endif
#if NETTEMP_ENABLE_PORTAL
  g_prefs.putBool("apAuto", g_portalAuto);
#endif
#if NETTEMP_ENABLE_I2C
  if (!g_i2cSelDefined) {
    g_prefs.putString("i2cSel", "");
  } else if (g_i2cSelAddrs.empty()) {
    g_prefs.putString("i2cSel", "-");
  } else {
    String s;
    for (size_t i = 0; i < g_i2cSelAddrs.size(); i++) {
      if (s.length()) s += ",";
      const uint8_t a = g_i2cSelAddrs[i];
      if (a < 16) s += "0";
      s += String(a, HEX);
    }
    s.toLowerCase();
    g_prefs.putString("i2cSel", s);
  }
#endif
  g_prefs.end();
}

// ============================================================================
// CRASH TRACKING
// ============================================================================

static void crashSaveBreadcrumb(const char* reason, const char* location) {
  Preferences prefs;
  prefs.begin("crash", false);
  prefs.putString("reason", reason);
  prefs.putString("location", location);
  prefs.putUInt("uptime", millis());
  prefs.putUInt("heap", ESP.getFreeHeap());
  prefs.end();
}

static void crashCheckAndLoad() {
  Preferences prefs;
  prefs.begin("crash", true);
  g_crashReason = prefs.getString("reason", "");
  if (g_crashReason.length() > 0) {
    g_hasCrashInfo = true;
    g_crashLocation = prefs.getString("location", "");
    g_crashUptime = prefs.getUInt("uptime", 0);
    g_crashFreeHeap = prefs.getUInt("heap", 0);
  } else {
    g_hasCrashInfo = false;
  }
  prefs.end();
}

static void crashClear() {
  Preferences prefs;
  prefs.begin("crash", false);
  prefs.clear();
  prefs.end();
  g_hasCrashInfo = false;
  g_crashReason = "";
  g_crashLocation = "";
  g_crashUptime = 0;
  g_crashFreeHeap = 0;
}

static String crashGetLog() {
  if (!g_hasCrashInfo) return "";

  String log;
  log.reserve(512);
  log += "=== ESP32 CRASH LOG ===\n\n";
  log += "Device ID: " + g_cfg.deviceId + "\n";
  log += "Crash Reason: " + g_crashReason + "\n";
  log += "Location: " + g_crashLocation + "\n";
  log += "Uptime at crash: " + String(g_crashUptime / 1000) + " seconds\n";
  log += "Free heap at crash: " + String(g_crashFreeHeap) + " bytes\n";
  log += "Min free heap: " + String(ESP.getMinFreeHeap()) + " bytes\n";
  log += "Heap size: " + String(ESP.getHeapSize()) + " bytes\n\n";

  const auto rr = esp_reset_reason();
  log += "Reset reason: ";
  if (rr == ESP_RST_UNKNOWN) log += "unknown";
  else if (rr == ESP_RST_POWERON) log += "poweron";
  else if (rr == ESP_RST_EXT) log += "ext";
  else if (rr == ESP_RST_SW) log += "sw";
  else if (rr == ESP_RST_PANIC) log += "panic";
  else if (rr == ESP_RST_INT_WDT) log += "int_wdt";
  else if (rr == ESP_RST_TASK_WDT) log += "task_wdt";
  else if (rr == ESP_RST_WDT) log += "wdt";
  else if (rr == ESP_RST_DEEPSLEEP) log += "deepsleep";
  else if (rr == ESP_RST_BROWNOUT) log += "brownout";
  else if (rr == ESP_RST_SDIO) log += "sdio";
  else log += String((int)rr);
  log += "\n\n";

  log += "Generated: " + String(millis() / 1000) + "s after current boot\n";
  log += "======================\n";

  return log;
}

static void i2cInitBus() {
  // If pins are not configured, use the board defaults.
  if (g_i2cBegun) {
#if defined(ARDUINO_ARCH_ESP32)
    g_i2cBus->end();
#endif
    g_i2cBegun = false;
  }
  if (g_cfg.i2cSdaPin >= 0 && g_cfg.i2cSclPin >= 0) {
    g_i2cBus->begin(g_cfg.i2cSdaPin, g_cfg.i2cSclPin);
  } else {
#if NETTEMP_CARDPUTER_UI
    // M5Cardputer: Use GPIO8/GPIO9 for external sensors
    // Wire1 is used to avoid conflict with keyboard (Wire on GPIO1/GPIO2)
    // You can change these pins via the web interface at /i2c_pins
    g_i2cBus->begin(8, 9);  // SDA=GPIO8, SCL=GPIO9
#else
    g_i2cBus->begin();
#endif
  }
  g_i2cBegun = true;
}

static SensorRow* upsertSensor(const String& mac) {
  for (auto& s : g_sensors) {
    if (s.mac == mac) return &s;
  }
  g_sensors.push_back(SensorRow{});
  g_sensors.back().mac = mac;
  return &g_sensors.back();
}

static String macShort(const String& mac) {
  if (mac.length() <= 8) return mac;
  return mac.substring(mac.length() - 8);
}

static String keyTail10(const String& s) {
  if (s.length() <= 10) return s;
  return s.substring(s.length() - 10);
}

static String prefKeyBleName(const String& macNo) { return String("bn") + keyTail10(macNo); }
static uint32_t fnv1a32(const String& s) {
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < s.length(); i++) {
    h ^= (uint8_t)s[i];
    h *= 16777619u;
  }
  return h;
}

static String dsKeySuffix(const String& romHex) {
  char buf[9]{};
  snprintf(buf, sizeof(buf), "%08x", (unsigned)fnv1a32(romHex));
  return String(buf);
}

static String prefKeyDsNameLegacy(const String& romHex) { return String("dn") + keyTail10(romHex); }
static String prefKeyDsName(const String& romHex) { return String("dn") + dsKeySuffix(romHex); }
static String prefKeyDsSel(const String& romHex) { return String("dsS") + dsKeySuffix(romHex); }
static String prefKeyI2cName(const String& addrHex) { return String("in") + addrHex; }
static String prefKeyDhtName() { return String("gd"); }  // GPIO DHT name
static String prefKeyHcsr04Name() { return String("gh"); }  // GPIO HC-SR04 name

static String prefKeyBleOledSel(const String& macNo) { return String("ob") + keyTail10(macNo); }
static String prefKeyBleOledTempc(const String& macNo) { return String("obT") + keyTail10(macNo); }
static String prefKeyBleOledHum(const String& macNo) { return String("obH") + keyTail10(macNo); }
static String prefKeyBleOledVolt(const String& macNo) { return String("obV") + keyTail10(macNo); }
static String prefKeyBleOledBatt(const String& macNo) { return String("obB") + keyTail10(macNo); }

static String prefKeyI2cOledSel(const String& addrHex) { return String("oi") + addrHex; }
static String prefKeyI2cOledTempc(const String& addrHex) { return String("oiT") + addrHex; }
static String prefKeyI2cOledHum(const String& addrHex) { return String("oiH") + addrHex; }
static String prefKeyI2cOledPress(const String& addrHex) { return String("oiP") + addrHex; }

static String prefKeyDsOledSel(const String& romHex) { return String("od") + keyTail10(romHex); }
static String prefKeyDsOledTempc(const String& romHex) { return String("odT") + keyTail10(romHex); }
static String prefKeyDsOledTempf(const String& romHex) { return String("odF") + keyTail10(romHex); }

constexpr const char* PREF_OLED_DHT = "odht";
constexpr const char* PREF_OLED_DHT_T = "odhtT";
constexpr const char* PREF_OLED_DHT_F = "odhtF";
constexpr const char* PREF_OLED_DHT_H = "odhtH";
constexpr const char* PREF_OLED_SOIL = "os";
constexpr const char* PREF_OLED_SOIL_RAW = "osR";
constexpr const char* PREF_OLED_SOIL_PCT = "osP";
constexpr const char* PREF_OLED_HC = "oh";
constexpr const char* PREF_OLED_HC_DIST = "ohD";
constexpr const char* PREF_OLED_VBAT = "ov";
constexpr const char* PREF_OLED_VBAT_V = "ovV";
constexpr const char* PREF_OLED_VBAT_B = "ovB";

static bool wifiConnected() {
  return WiFi.status() == WL_CONNECTED;
}

#include "nettemp_web.inc"
