## Nettemp ESP32 (Cardputer): BLE thermometer viewer (Xiaomi LYWSD03MMC)

Status: works OK on a single ESP32 device; **Cardputer support is still in progress**.

More info: https://nettemp.pl

Arduino sketch for **any ESP32** as a Nettemp client (also works great with **M5Stack Cardputer / ESP32-S3**) that:
- scans BLE in passive/active mode and decodes **LYWSD03MMC** (ATC/PVVX advertising),
- reads I2C sensors (**BMP280**, **BME280**, **TMP102**, **SHT3x**, **SHT21/HTU21D/SI7021**),
- reads GPIO sensors like **DHT11/DHT22**, **DS18B20**, **HC-SR04** (ultrasonic), and **capacitive soil (ADC)**,
- supports **VBAT** reading and **deep sleep** (duty-cycle),
- shows readings on **Cardputer display** or **SSD1306 OLED**,
- has **captive portal + web UI** (Basic Auth) for config,
- can send data to **MQTT**, **Nettemp Cloud API**, or **Webhook (JSON)**,
- supports **OTA firmware upload** from the web UI,
- can show a QR to import its API token into `app.nettemp.pl`.

### Quick start (Arduino IDE)
1. Install ESP32 boards support (Espressif).
2. Install libraries: `M5Cardputer`, `NimBLE-Arduino`, `Adafruit_GFX`, `Adafruit_SSD1306` (OLED), `PubSubClient` (optional), `OneWire` (optional).
3. Board: **ESP32S3 Dev Module** (or Cardputer profile).
4. Flash `nettemp_esp32.ino`.

If you see NimBLE compile errors, make sure you installed **NimBLE-Arduino** (not the built-in ESP32 BLE lib).

### Usage
- First boot starts an open AP + captive portal:
  - SSID: `nettemp-setup-<chipid>`
  - URL: `http://192.168.4.1/`
- After WiFi connects: open `http://<device-ip>/` (Basic Auth) to configure BLE/I2C/MQTT/Server.
- On device: `BtnA/BtnB` move, `Enter` select, `Esc` back.

### Headless mode (no display)
- Auto-enabled when Cardputer is not detected.
- Force with `#define NETTEMP_HEADLESS 1` (or build flag `-DNETTEMP_HEADLESS=1`).
- Configure via Serial Monitor (`115200`).

### Compile-time toggles (size/features)
In `nettemp_esp32.ino`:
- `NETTEMP_ENABLE_SERVER` (HTTP/HTTPS to Cloud)
- `NETTEMP_ENABLE_MQTT`
- `NETTEMP_ENABLE_I2C`
- `NETTEMP_ENABLE_OLED`
- `NETTEMP_ENABLE_PORTAL` (web UI + OTA)

If the sketch is too big, switch to **Huge APP (3MB No OTA)** partition scheme (note: OTA upload needs a partition scheme with OTA).

### BLE notes
- This sketch decodes **advertising payloads** (ATC/PVVX).
- Stock Xiaomi firmware “active mode” (GATT read) needs bindkeys and a different implementation.
- To parse manufacturer data (more noise), set `#define NETTEMP_BLE_PARSE_MFG 1`.

### Cloud send (default)
Endpoint: `https://api.nettemp.pl/api/v1/data`  
Auth: `Authorization: Bearer ntk_...`

### Troubleshooting
- `esp_bt.h: No such file or directory` → wrong board/core; install **esp32 by Espressif Systems**.
- BLE sees nothing → try `ble active`, ensure you flashed this repo version.

### Code layout
- `nettemp_esp32.ino` (main)
- `nettemp_core.inc`, `nettemp_web.inc`, `nettemp_headless.inc`, `nettemp_power.inc`

---
