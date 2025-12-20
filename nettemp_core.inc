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
#include <NimBLEDevice.h>
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

constexpr uint32_t UI_REFRESH_MS = 120;
constexpr uint32_t STALE_AFTER_MS = 20UL * 60UL * 1000UL;
constexpr uint32_t SPLASH_MS = 1100;
constexpr uint32_t QR_ROTATE_MS = 2000;
constexpr const char* NETTEMP_APP_URL = "https://app.nettemp.pl";

#if !NETTEMP_HEADLESS
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
  bool webhookEnabled = false;
  String webhookUrl;
  uint32_t webhookIntervalMs = 60'000;
  bool bleSendServer = true;
  bool bleSendMqtt = true;
  bool bleSendWebhook = true;
  bool i2cSendServer = true;
  bool i2cSendMqtt = true;
  bool i2cSendWebhook = true;
  bool gpioSendServer = true;
  bool gpioSendMqtt = true;
  bool gpioSendWebhook = true;

  // GPIO sensors
  bool dsEnabled = false;
  int dsPin = 4;   // GPIO for DS18B20 (1-Wire)
  bool dhtEnabled = false;
  int dhtPin = 16; // GPIO for DHT11/DHT22
  int dhtType = 22; // 11 or 22
  bool soilEnabled = false;
  int soilAdcPin = 34; // ADC1 pin for capacitive soil sensor
  int soilDryRaw = 3000;
  int soilWetRaw = 1400;
  bool soilSendRaw = true;
  bool soilSendPct = true;
  bool mqttSoilSendRaw = true;
  bool mqttSoilSendPct = true;
  bool hcsr04Enabled = false;
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

  // Battery (VBAT)
  // mode: 0=off, 1=adc, 2=pmic (Cardputer)
  uint8_t vbatMode =
#if !NETTEMP_HEADLESS
    2
#else
    1
#endif
    ;
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

// I2C → Server (Cloud) field selection.
// Defaults: tempc + hum + press_hpa
constexpr uint32_t SRV_I2C_TEMPC = 1u << 0;
constexpr uint32_t SRV_I2C_HUM = 1u << 1;
constexpr uint32_t SRV_I2C_PRESS = 1u << 2;
uint32_t g_srvI2cFields = (SRV_I2C_TEMPC | SRV_I2C_HUM | SRV_I2C_PRESS);
uint32_t g_mqttI2cFields = (SRV_I2C_TEMPC | SRV_I2C_HUM | SRV_I2C_PRESS);

std::vector<SensorRow> g_sensors;
bool g_activeScan = false;
int g_bleCursor = 0;
uint8_t g_bleMaxResults = 20;
bool g_bleParseMfg = NETTEMP_BLE_PARSE_MFG;

#if NETTEMP_ENABLE_I2C
std::vector<I2cSensorInfo> g_i2cSensors;
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
uint32_t g_lastWebhookSendMs = 0;
uint32_t g_lastSerialRxMs = 0;
bool g_i2cBegun = false;

#if !NETTEMP_HEADLESS
enum class View {
  MainMenu,
  ScanBle,
  ScanI2c,
  WifiSetup,
  MqttSetup,
  ServerSetup,
  PairQr,
  About,
};

View g_view = View::MainMenu;
int g_menuIndex = 0;
uint32_t g_lastQrTickMs = 0;
#else
bool g_headlessAutoSelect = true;
bool g_headlessStatusLogEnabled = false;
uint32_t g_headlessStatusLogIntervalMs = 30'000;
bool g_headlessBleAutoScan = false;
bool g_headlessBleScanCollectUnknown = false;
bool g_headlessDiagnosticsScanActive = false;
uint32_t g_lastHeadlessLogMs = 0;
String g_serialLineBuf;
#endif

// Arduino IDE does not auto-generate prototypes inside namespaces.
static void prefsSave();
static String macNoColonsUpper(const String& mac);
static String macWithColonsUpper(const String& mac);
static std::vector<size_t> buildBleSortedIndex();
static String serverBleFieldsEnabledString();
static String serverBleFieldsAvailableString();
static void tickSendMqtt();
static void tickSendServer();
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
static void powerSleepCycleMaybe();
static void powerEnterDeepSleep(uint32_t sleepSeconds);
static void vbatTick();
static bool vbatReadVolts(float& outV);
static int vbatToPercent(float vbatV);

uint32_t g_lastWifiBeginMs = 0;
uint32_t g_lastBleEnsureMs = 0;

struct BleManualSel {
  String macNo;   // A4C138DE459E
  bool selected;  // true=send, false=disabled
};
std::vector<BleManualSel> g_bleManualSel;

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
  g_cfg.webhookEnabled = g_prefs.getBool("whEn", g_cfg.webhookEnabled);
  g_cfg.webhookUrl = g_prefs.getString("whUrl", g_cfg.webhookUrl);
  g_cfg.webhookIntervalMs = g_prefs.getUInt("whInt", g_cfg.webhookIntervalMs);
  g_cfg.bleSendServer = g_prefs.getBool("bleSrv", g_cfg.bleSendServer);
  g_cfg.bleSendMqtt = g_prefs.getBool("bleMq", g_cfg.bleSendMqtt);
  g_cfg.bleSendWebhook = g_prefs.getBool("bleWh", g_cfg.bleSendWebhook);
  g_cfg.i2cSendServer = g_prefs.getBool("i2cSrv", g_cfg.i2cSendServer);
  g_cfg.i2cSendMqtt = g_prefs.getBool("i2cMq", g_cfg.i2cSendMqtt);
  g_cfg.i2cSendWebhook = g_prefs.getBool("i2cWh", g_cfg.i2cSendWebhook);
  g_cfg.gpioSendServer = g_prefs.getBool("gpioSrv", g_cfg.gpioSendServer);
  g_cfg.gpioSendMqtt = g_prefs.getBool("gpioMq", g_cfg.gpioSendMqtt);
  g_cfg.gpioSendWebhook = g_prefs.getBool("gpioWh", g_cfg.gpioSendWebhook);
  if (g_cfg.webhookIntervalMs < 5000) g_cfg.webhookIntervalMs = 5000;
  if (g_cfg.webhookIntervalMs > 3600000) g_cfg.webhookIntervalMs = 3600000;
  g_srvBleFields = g_prefs.getUInt("srvBleF", g_srvBleFields);
  if (g_srvBleFields == 0) g_srvBleFields = (SRV_BLE_TEMPC | SRV_BLE_HUM | SRV_BLE_BATT);
  g_srvI2cFields = g_prefs.getUInt("srvI2cF", g_srvI2cFields);
  if (g_srvI2cFields == 0) g_srvI2cFields = (SRV_I2C_TEMPC | SRV_I2C_HUM | SRV_I2C_PRESS);
  g_mqttBleFields = g_prefs.getUInt("mqBleF", g_mqttBleFields);
  if (g_mqttBleFields == 0) g_mqttBleFields = (SRV_BLE_TEMPC | SRV_BLE_HUM | SRV_BLE_BATT);
  g_mqttI2cFields = g_prefs.getUInt("mqI2cF", g_mqttI2cFields);
  if (g_mqttI2cFields == 0) g_mqttI2cFields = (SRV_I2C_TEMPC | SRV_I2C_HUM | SRV_I2C_PRESS);
  bleManualDecode(g_prefs.getString("bleMan", ""));

  g_cfg.dsEnabled = g_prefs.getBool("dsEn", g_cfg.dsEnabled);
  g_cfg.dsPin = (int)g_prefs.getInt("dsPin", g_cfg.dsPin);
  g_cfg.dhtEnabled = g_prefs.getBool("dhtEn", g_cfg.dhtEnabled);
  g_cfg.dhtPin = (int)g_prefs.getInt("dhtPin", g_cfg.dhtPin);
  g_cfg.dhtType = (int)g_prefs.getInt("dhtType", g_cfg.dhtType);
  g_cfg.soilEnabled = g_prefs.getBool("soilEn", g_cfg.soilEnabled);
  g_cfg.soilAdcPin = (int)g_prefs.getInt("soilPin", g_cfg.soilAdcPin);
  g_cfg.soilDryRaw = (int)g_prefs.getInt("soilDry", g_cfg.soilDryRaw);
  g_cfg.soilWetRaw = (int)g_prefs.getInt("soilWet", g_cfg.soilWetRaw);
  g_cfg.soilSendRaw = g_prefs.getBool("soilRaw", g_cfg.soilSendRaw);
  g_cfg.soilSendPct = g_prefs.getBool("soilPct", g_cfg.soilSendPct);
  g_cfg.mqttSoilSendRaw = g_prefs.getBool("mqSoilRaw", g_cfg.mqttSoilSendRaw);
  g_cfg.mqttSoilSendPct = g_prefs.getBool("mqSoilPct", g_cfg.mqttSoilSendPct);
  g_cfg.hcsr04Enabled = g_prefs.getBool("hcEn", g_cfg.hcsr04Enabled);
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

  g_cfg.vbatMode = (uint8_t)g_prefs.getUChar("vbMode", g_cfg.vbatMode);
  g_cfg.vbatAdcPin = (int)g_prefs.getInt("vbPin", g_cfg.vbatAdcPin);
  g_cfg.vbatDividerX1000 = (uint16_t)g_prefs.getUShort("vbDiv", g_cfg.vbatDividerX1000);
  g_cfg.vbatFullMv = (uint16_t)g_prefs.getUShort("vbFull", g_cfg.vbatFullMv);
  g_cfg.vbatEmptyMv = (uint16_t)g_prefs.getUShort("vbEmp", g_cfg.vbatEmptyMv);
  g_cfg.vbatSendVolt = g_prefs.getBool("vbSV", g_cfg.vbatSendVolt);
  if (g_cfg.vbatMode > 2) g_cfg.vbatMode = 0;
  if (g_cfg.vbatDividerX1000 < 500) g_cfg.vbatDividerX1000 = 500;
  if (g_cfg.vbatDividerX1000 > 5000) g_cfg.vbatDividerX1000 = 5000;
  if (g_cfg.vbatFullMv < 3600) g_cfg.vbatFullMv = 4200;
  if (g_cfg.vbatFullMv > 5000) g_cfg.vbatFullMv = 4200;
  if (g_cfg.vbatEmptyMv < 2500) g_cfg.vbatEmptyMv = 3200;
  if (g_cfg.vbatEmptyMv > 4200) g_cfg.vbatEmptyMv = 3200;

  g_activeScan = g_prefs.getBool("bleAct", false);
  {
    const uint32_t v = g_prefs.getUInt("bleMax", (uint32_t)g_bleMaxResults);
    if (v < 5) g_bleMaxResults = 5;
    else if (v > 200) g_bleMaxResults = 200;
    else g_bleMaxResults = (uint8_t)v;
  }
  g_bleParseMfg = g_prefs.getBool("bleMfg", g_bleParseMfg);
#if NETTEMP_HEADLESS
  g_headlessStatusLogEnabled = g_prefs.getBool("hlLogEn", g_headlessStatusLogEnabled);
  {
    const uint32_t v = g_prefs.getUInt("hlLogInt", g_headlessStatusLogIntervalMs);
    g_headlessStatusLogIntervalMs = std::min<uint32_t>(300'000, std::max<uint32_t>(1000, v));
  }
  g_headlessBleAutoScan = g_prefs.getBool("bleAuto", g_headlessBleAutoScan);
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
  g_prefs.putBool("whEn", g_cfg.webhookEnabled);
  g_prefs.putString("whUrl", g_cfg.webhookUrl);
  g_prefs.putUInt("whInt", g_cfg.webhookIntervalMs);
  g_prefs.putBool("bleSrv", g_cfg.bleSendServer);
  g_prefs.putBool("bleMq", g_cfg.bleSendMqtt);
  g_prefs.putBool("bleWh", g_cfg.bleSendWebhook);
  g_prefs.putBool("i2cSrv", g_cfg.i2cSendServer);
  g_prefs.putBool("i2cMq", g_cfg.i2cSendMqtt);
  g_prefs.putBool("i2cWh", g_cfg.i2cSendWebhook);
  g_prefs.putBool("gpioSrv", g_cfg.gpioSendServer);
  g_prefs.putBool("gpioMq", g_cfg.gpioSendMqtt);
  g_prefs.putBool("gpioWh", g_cfg.gpioSendWebhook);
  g_prefs.putUInt("srvBleF", g_srvBleFields);
  g_prefs.putUInt("srvI2cF", g_srvI2cFields);
  g_prefs.putUInt("mqBleF", g_mqttBleFields);
  g_prefs.putUInt("mqI2cF", g_mqttI2cFields);
  g_prefs.putString("bleMan", bleManualEncode());

  g_prefs.putBool("dsEn", g_cfg.dsEnabled);
  g_prefs.putInt("dsPin", g_cfg.dsPin);
  g_prefs.putBool("dhtEn", g_cfg.dhtEnabled);
  g_prefs.putInt("dhtPin", g_cfg.dhtPin);
  g_prefs.putInt("dhtType", g_cfg.dhtType);
  g_prefs.putBool("soilEn", g_cfg.soilEnabled);
  g_prefs.putInt("soilPin", g_cfg.soilAdcPin);
  g_prefs.putInt("soilDry", g_cfg.soilDryRaw);
  g_prefs.putInt("soilWet", g_cfg.soilWetRaw);
  g_prefs.putBool("soilRaw", g_cfg.soilSendRaw);
  g_prefs.putBool("soilPct", g_cfg.soilSendPct);
  g_prefs.putBool("mqSoilRaw", g_cfg.mqttSoilSendRaw);
  g_prefs.putBool("mqSoilPct", g_cfg.mqttSoilSendPct);
  g_prefs.putBool("hcEn", g_cfg.hcsr04Enabled);
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

  g_prefs.putUChar("vbMode", g_cfg.vbatMode);
  g_prefs.putInt("vbPin", g_cfg.vbatAdcPin);
  g_prefs.putUShort("vbDiv", g_cfg.vbatDividerX1000);
  g_prefs.putUShort("vbFull", g_cfg.vbatFullMv);
  g_prefs.putUShort("vbEmp", g_cfg.vbatEmptyMv);
  g_prefs.putBool("vbSV", g_cfg.vbatSendVolt);

  g_prefs.putBool("bleAct", g_activeScan);
  g_prefs.putUInt("bleMax", (uint32_t)g_bleMaxResults);
  g_prefs.putBool("bleMfg", g_bleParseMfg);
#if NETTEMP_HEADLESS
  g_prefs.putBool("hlLogEn", g_headlessStatusLogEnabled);
  g_prefs.putUInt("hlLogInt", g_headlessStatusLogIntervalMs);
  g_prefs.putBool("bleAuto", g_headlessBleAutoScan);
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

static void i2cInitBus() {
  // If pins are not configured, use the board defaults.
  if (g_i2cBegun) {
#if defined(ARDUINO_ARCH_ESP32)
    Wire.end();
#endif
    g_i2cBegun = false;
  }
  if (g_cfg.i2cSdaPin >= 0 && g_cfg.i2cSclPin >= 0) {
    Wire.begin(g_cfg.i2cSdaPin, g_cfg.i2cSclPin);
  } else {
    Wire.begin();
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
static String prefKeyDsName(const String& romHex) { return String("dn") + keyTail10(romHex); }
static String prefKeyI2cName(const String& addrHex) { return String("in") + addrHex; }

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
