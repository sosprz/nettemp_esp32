#pragma once

#include <vector>
#include <Arduino.h>
#include <Wire.h>

#ifndef NETTEMP_ENABLE_I2C
#define NETTEMP_ENABLE_I2C 1
#endif

#if NETTEMP_ENABLE_I2C

enum class I2cSensorType {
  Unknown,
  BMP180,
  BMP280,
  BME280,
  TMP102,
  SHT3X,
  SHT2X,
  AHTxx,  // AHT10/20/21/30
  TSL2561,
  VL53L0X,
};

struct I2cSensorReading {
  bool ok = false;
  float temperature_c = NAN;
  float humidity_pct = NAN;     // SHT3x only
  float pressure_hpa = NAN;     // BMP180/BMP280 only
  float light_lux = NAN;        // TSL2561
  float distance_mm = NAN;      // VL53L0X
};

struct I2cSensorInfo {
  I2cSensorType type = I2cSensorType::Unknown;
  uint8_t address = 0;
  I2cSensorReading reading;
  uint32_t last_seen_ms = 0;
  uint32_t last_mqtt_sent_ms = 0;
  bool selected = false;
};

// Detect known I2C sensors and return a list. Uses best-effort heuristics:
// - BMP180 at 0x76/0x77, chip_id 0x55 (reg 0xD0)
// - BMP280 at 0x76/0x77, chip_id 0x58 (reg 0xD0)
// - TMP102 at 0x48-0x4B
// - SHT3x at 0x44/0x45 (no chip-id; detected by successful measurement + CRC)
// - HTU21D/SHT21/SI7021 at 0x40 (detected by successful measurement + CRC)
// - AHT10/20/21/30 at 0x38 (detected by successful measurement)
// - TSL2561 at 0x29/0x39/0x49 (ID register)
// - VL53L0X at 0x29 (requires Adafruit_VL53L0X library)
std::vector<I2cSensorInfo> i2cDetectKnownSensors(TwoWire& wire);
std::vector<uint8_t> i2cScanAllAddresses(TwoWire& wire);

// Update readings for sensors (best-effort; keeps type/address).
void i2cUpdateReadings(TwoWire& wire, std::vector<I2cSensorInfo>& sensors);

const char* i2cSensorTypeName(I2cSensorType t);

#else
// Stubs for builds that disable I2C to reduce flash usage.
enum class I2cSensorType { Disabled };
struct I2cSensorReading { bool ok = false; };
struct I2cSensorInfo {};
inline std::vector<I2cSensorInfo> i2cDetectKnownSensors(TwoWire&) { return {}; }
inline std::vector<uint8_t> i2cScanAllAddresses(TwoWire&) { return {}; }
inline void i2cUpdateReadings(TwoWire&, std::vector<I2cSensorInfo>&) {}
inline const char* i2cSensorTypeName(I2cSensorType) { return "DISABLED"; }
#endif
