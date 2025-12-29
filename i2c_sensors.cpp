#include "i2c_sensors.h"

#if NETTEMP_ENABLE_I2C

// VL53L0X library - use Adafruit implementation
// Note: __has_include doesn't work with arduino-cli auto-discovery,
// so we include directly and let the build fail if library is missing
#include <Adafruit_VL53L0X.h>
#define NETTEMP_HAVE_VL53L0X 1

namespace {

bool i2cReadBytes(TwoWire& wire, uint8_t addr, uint8_t reg, uint8_t* out, size_t n) {
  wire.beginTransmission(addr);
  wire.write(reg);
  if (wire.endTransmission(false) != 0) return false;
  const size_t got = wire.requestFrom((int)addr, (int)n);
  if (got != n) return false;
  for (size_t i = 0; i < n; i++) out[i] = (uint8_t)wire.read();
  return true;
}

bool i2cWriteBytes(TwoWire& wire, uint8_t addr, const uint8_t* data, size_t n) {
  wire.beginTransmission(addr);
  for (size_t i = 0; i < n; i++) wire.write(data[i]);
  return wire.endTransmission(true) == 0;
}

// -------------------- TSL2561 --------------------
bool tsl2561ReadLux(TwoWire& wire, uint8_t addr, float& outLux) {
  const uint8_t cmd = 0x80;  // command bit
  uint8_t id = 0;
  if (!i2cReadBytes(wire, addr, cmd | 0x0A, &id, 1)) return false;
  // TSL2561 part number is 0x5 in the upper nibble.
  if ((id & 0xF0) != 0x50) return false;

  const uint8_t pwrOn[2] = {cmd | 0x00, 0x03};
  if (!i2cWriteBytes(wire, addr, pwrOn, 2)) return false;
  const uint8_t timing[2] = {cmd | 0x01, 0x02};  // 402ms, 1x gain
  if (!i2cWriteBytes(wire, addr, timing, 2)) return false;
  delay(450);

  uint8_t ch0b[2]{};
  uint8_t ch1b[2]{};
  if (!i2cReadBytes(wire, addr, cmd | 0x0C, ch0b, 2)) return false;
  if (!i2cReadBytes(wire, addr, cmd | 0x0E, ch1b, 2)) return false;
  const uint16_t ch0 = (uint16_t)ch0b[0] | ((uint16_t)ch0b[1] << 8);
  const uint16_t ch1 = (uint16_t)ch1b[0] | ((uint16_t)ch1b[1] << 8);
  if (ch0 == 0) return false;
  const float ratio = (float)ch1 / (float)ch0;
  float lux = 0.0f;
  if (ratio <= 0.50f) {
    lux = 0.0304f * ch0 - 0.062f * ch0 * powf(ratio, 1.4f);
  } else if (ratio <= 0.61f) {
    lux = 0.0224f * ch0 - 0.031f * ch1;
  } else if (ratio <= 0.80f) {
    lux = 0.0128f * ch0 - 0.0153f * ch1;
  } else if (ratio <= 1.30f) {
    lux = 0.00146f * ch0 - 0.00112f * ch1;
  } else {
    lux = 0.0f;
  }
  if (lux < 0.0f) lux = 0.0f;
  outLux = lux;
  return true;
}

// -------------------- VL53L0X --------------------
bool vl53l0xDetect(TwoWire& wire, uint8_t addr) {
  uint8_t id = 0;
  if (!i2cReadBytes(wire, addr, 0xC0, &id, 1)) return false;
  // VL53L0X model ID is typically 0xEE.
  return id == 0xEE;
}

bool vl53l0xReadDistanceMm(TwoWire& wire, uint8_t addr, float& outMm) {
#if NETTEMP_HAVE_VL53L0X
  // Adafruit VL53L0X implementation
  static Adafruit_VL53L0X* sensor = nullptr;
  static bool sensorReady = false;
  static uint8_t sensorAddr = 0;
  static TwoWire* sensorWire = nullptr;
  static uint32_t lastInitMs = 0;
  static uint32_t lastSuccessMs = 0;
  const uint32_t nowMs = millis();

  // Reinitialize if:
  // - First time (sensor == nullptr)
  // - Address/wire changed
  // - Init failed and 10s passed
  // - No successful read in 60s (sensor might have reset)
  if (sensor == nullptr || sensorAddr != addr || sensorWire != &wire ||
      (!sensorReady && (nowMs - lastInitMs > 10000)) ||
      (sensorReady && lastSuccessMs > 0 && (nowMs - lastSuccessMs > 60000))) {

    // Clean up old sensor
    if (sensor != nullptr) {
      delete sensor;
      sensor = nullptr;
    }

    // Create new sensor instance
    sensor = new Adafruit_VL53L0X();
    sensorReady = false;
    sensorAddr = addr;
    sensorWire = &wire;
    lastInitMs = nowMs;

    // Try to initialize - use debug=false to avoid serial output
    if (sensor->begin(addr, false, &wire)) {
      sensorReady = true;
      lastSuccessMs = nowMs;
      delay(100);  // Give sensor time to stabilize
    } else {
      // Init failed - will retry in 10s
      delete sensor;
      sensor = nullptr;
      return false;
    }
  }

  if (!sensorReady) return false;

  // Perform ranging measurement
  VL53L0X_RangingMeasurementData_t measure{};
  sensor->rangingTest(&measure, false);

  // Accept readings based on status code
  // Status: 0=good, 1=sigma fail, 2=signal fail, 3=min range, 4=phase fail
  if (measure.RangeMilliMeter < 8190) {
    // Accept status 0 (good measurement)
    if (measure.RangeStatus == 0 && measure.RangeMilliMeter >= 0) {
      outMm = (float)measure.RangeMilliMeter;
      lastSuccessMs = nowMs;
      return true;
    }
    // Try accepting degraded but potentially usable readings
    if (measure.RangeStatus >= 1 && measure.RangeStatus <= 2 && measure.RangeMilliMeter > 20) {
      outMm = (float)measure.RangeMilliMeter;
      lastSuccessMs = nowMs;
      return true;
    }
  }

  return false;

#else
  (void)wire;
  (void)addr;
  (void)outMm;
  return false;
#endif
}

// -------------------- TMP102 --------------------
bool tmp102ReadTempC(TwoWire& wire, uint8_t addr, float& outC) {
  uint8_t buf[2]{};
  if (!i2cReadBytes(wire, addr, 0x00, buf, 2)) return false;
  // Default TMP102: 12-bit, left-justified in 16-bit register (bits 15..4)
  int16_t raw = ((int16_t)buf[0] << 8) | buf[1];
  raw >>= 4;
  // sign extend 12-bit
  if (raw & 0x0800) raw |= 0xF000;
  outC = (float)raw * 0.0625f;
  return true;
}

// -------------------- SHT3x --------------------
uint8_t sht3xCrc8(const uint8_t* data, size_t len) {
  // Polynomial 0x31, init 0xFF (Sensirion)
  uint8_t crc = 0xFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

bool sht3xReadTempHum(TwoWire& wire, uint8_t addr, float& outC, float& outH) {
  // Single shot, high repeatability, no clock stretching: 0x2400
  const uint8_t cmd[2] = {0x24, 0x00};
  if (!i2cWriteBytes(wire, addr, cmd, 2)) return false;
  delay(15);
  wire.requestFrom((int)addr, 6);
  if (wire.available() != 6) return false;
  uint8_t b[6];
  for (int i = 0; i < 6; i++) b[i] = (uint8_t)wire.read();

  if (sht3xCrc8(b, 2) != b[2]) return false;
  if (sht3xCrc8(b + 3, 2) != b[5]) return false;

  const uint16_t rawT = ((uint16_t)b[0] << 8) | b[1];
  const uint16_t rawH = ((uint16_t)b[3] << 8) | b[4];

  outC = -45.0f + (175.0f * ((float)rawT / 65535.0f));
  outH = 100.0f * ((float)rawH / 65535.0f);
  return true;
}

// -------------------- SHT2x / HTU21D / SI70xx --------------------
uint8_t sht2xCrc8(const uint8_t* data, size_t len) {
  // Polynomial 0x31, init 0x00 (Sensirion)
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

bool sht2xReadValue(TwoWire& wire, uint8_t addr, uint8_t cmd, uint16_t& raw) {
  wire.beginTransmission(addr);
  wire.write(cmd);
  if (wire.endTransmission(true) != 0) return false;
  delay(85);
  wire.requestFrom((int)addr, 3);
  if (wire.available() != 3) return false;
  uint8_t b[3];
  for (int i = 0; i < 3; i++) b[i] = (uint8_t)wire.read();
  if (sht2xCrc8(b, 2) != b[2]) return false;
  raw = ((uint16_t)b[0] << 8) | b[1];
  raw &= 0xFFFC; // clear status bits
  return true;
}

bool sht2xReadTempHum(TwoWire& wire, uint8_t addr, float& outC, float& outH) {
  // Soft reset to avoid stuck sensors.
  {
    wire.beginTransmission(addr);
    wire.write(0xFE);
    wire.endTransmission(true);
    delay(15);
  }
  uint16_t rawT = 0;
  uint16_t rawH = 0;
  // Use no-hold commands for better compatibility.
  if (!sht2xReadValue(wire, addr, 0xF3, rawT)) return false; // Temp, no hold master
  if (!sht2xReadValue(wire, addr, 0xF5, rawH)) return false; // Humidity, no hold master
  outC = -46.85f + (175.72f * ((float)rawT / 65536.0f));
  outH = -6.0f + (125.0f * ((float)rawH / 65536.0f));
  if (outH < 0.0f) outH = 0.0f;
  if (outH > 100.0f) outH = 100.0f;
  return true;
}

// -------------------- AHT10/20/21/30 --------------------
bool ahtxxInit(TwoWire& wire, uint8_t addr) {
  // Initialize AHTxx: send 0xE1 calibration command with parameters 0x08 0x00
  // AHT10 requires this before each measurement; AHT20/21/30 only need it once at startup
  // but it's safe to send before each measurement for compatibility with all models
  const uint8_t cmd[3] = {0xE1, 0x08, 0x00};
  return i2cWriteBytes(wire, addr, cmd, 3);
}

bool ahtxxReadTempHum(TwoWire& wire, uint8_t addr, float& outC, float& outH) {
  // Initialize sensor (required for AHT10, safe for AHT20/21/30)
  if (!ahtxxInit(wire, addr)) return false;
  delay(10);

  // Trigger measurement: 0xAC 0x33 0x00 (same for all AHTxx models)
  const uint8_t cmd[3] = {0xAC, 0x33, 0x00};
  if (!i2cWriteBytes(wire, addr, cmd, 3)) return false;
  delay(80); // Measurement takes ~75ms

  // Read 6 bytes: status + 5 data bytes
  wire.requestFrom((int)addr, 6);
  if (wire.available() != 6) return false;
  uint8_t b[6];
  for (int i = 0; i < 6; i++) b[i] = (uint8_t)wire.read();

  // Check if sensor is busy (bit 7 of status byte should be 0 when ready)
  if (b[0] & 0x80) return false;

  // Extract 20-bit humidity and temperature values
  uint32_t rawH = ((uint32_t)b[1] << 12) | ((uint32_t)b[2] << 4) | ((uint32_t)b[3] >> 4);
  uint32_t rawT = (((uint32_t)b[3] & 0x0F) << 16) | ((uint32_t)b[4] << 8) | (uint32_t)b[5];

  // Convert to physical values (same formula for all AHTxx models)
  outH = ((float)rawH / 1048576.0f) * 100.0f;
  outC = (((float)rawT / 1048576.0f) * 200.0f) - 50.0f;

  // Clamp humidity to valid range
  if (outH < 0.0f) outH = 0.0f;
  if (outH > 100.0f) outH = 100.0f;

  return true;
}

// -------------------- BMP180 --------------------
struct Bmp180Calib {
  int16_t ac1;
  int16_t ac2;
  int16_t ac3;
  uint16_t ac4;
  uint16_t ac5;
  uint16_t ac6;
  int16_t b1;
  int16_t b2;
  int16_t mb;
  int16_t mc;
  int16_t md;
};

bool bmp180ReadCalib(TwoWire& wire, uint8_t addr, Bmp180Calib& c) {
  uint8_t b[22]{};
  if (!i2cReadBytes(wire, addr, 0xAA, b, sizeof(b))) return false;
  auto u16 = [&](int off) -> uint16_t { return (uint16_t)b[off] << 8 | b[off + 1]; };
  auto s16 = [&](int off) -> int16_t { return (int16_t)u16(off); };
  c.ac1 = s16(0);
  c.ac2 = s16(2);
  c.ac3 = s16(4);
  c.ac4 = u16(6);
  c.ac5 = u16(8);
  c.ac6 = u16(10);
  c.b1 = s16(12);
  c.b2 = s16(14);
  c.mb = s16(16);
  c.mc = s16(18);
  c.md = s16(20);
  return true;
}

bool bmp180ReadRawTemp(TwoWire& wire, uint8_t addr, int32_t& raw) {
  const uint8_t cmd[2] = {0xF4, 0x2E};
  if (!i2cWriteBytes(wire, addr, cmd, 2)) return false;
  delay(5);
  uint8_t b[2]{};
  if (!i2cReadBytes(wire, addr, 0xF6, b, 2)) return false;
  raw = ((int32_t)b[0] << 8) | b[1];
  return true;
}

bool bmp180ReadRawPress(TwoWire& wire, uint8_t addr, int32_t& raw) {
  const uint8_t cmd[2] = {0xF4, 0x34};  // OSS=0
  if (!i2cWriteBytes(wire, addr, cmd, 2)) return false;
  delay(8);
  uint8_t b[3]{};
  if (!i2cReadBytes(wire, addr, 0xF6, b, 3)) return false;
  raw = (((int32_t)b[0] << 16) | ((int32_t)b[1] << 8) | b[2]) >> 8;
  return true;
}

bool bmp180ReadTempPress(TwoWire& wire, uint8_t addr, float& outC, float& outHpa) {
  uint8_t id = 0;
  if (!i2cReadBytes(wire, addr, 0xD0, &id, 1)) return false;
  if (id != 0x55) return false;

  Bmp180Calib c{};
  if (!bmp180ReadCalib(wire, addr, c)) return false;

  int32_t ut = 0;
  int32_t up = 0;
  if (!bmp180ReadRawTemp(wire, addr, ut)) return false;
  if (!bmp180ReadRawPress(wire, addr, up)) return false;

  int32_t x1 = ((ut - (int32_t)c.ac6) * (int32_t)c.ac5) >> 15;
  int32_t x2 = ((int32_t)c.mc << 11) / (x1 + c.md);
  int32_t b5 = x1 + x2;
  int32_t t = (b5 + 8) >> 4;
  outC = (float)t / 10.0f;

  int32_t b6 = b5 - 4000;
  x1 = ((int32_t)c.b2 * ((b6 * b6) >> 12)) >> 11;
  x2 = ((int32_t)c.ac2 * b6) >> 11;
  int32_t x3 = x1 + x2;
  int32_t b3 = (((int32_t)c.ac1 * 4 + x3) + 2) >> 2;
  x1 = ((int32_t)c.ac3 * b6) >> 13;
  x2 = ((int32_t)c.b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  uint32_t b4 = ((uint32_t)c.ac4 * (uint32_t)(x3 + 32768)) >> 15;
  uint32_t b7 = ((uint32_t)up - b3) * 50000;
  int32_t p = (b7 < 0x80000000) ? (int32_t)((b7 * 2) / b4) : (int32_t)((b7 / b4) * 2);
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  p = p + ((x1 + x2 + 3791) >> 4);

  outHpa = (float)p / 100.0f;
  return true;
}

// -------------------- BMP280 --------------------
struct Bmp280Calib {
  uint16_t dig_T1;
  int16_t dig_T2;
  int16_t dig_T3;
  uint16_t dig_P1;
  int16_t dig_P2;
  int16_t dig_P3;
  int16_t dig_P4;
  int16_t dig_P5;
  int16_t dig_P6;
  int16_t dig_P7;
  int16_t dig_P8;
  int16_t dig_P9;
};

bool bmp280ReadCalib(TwoWire& wire, uint8_t addr, Bmp280Calib& c) {
  uint8_t b[24]{};
  if (!i2cReadBytes(wire, addr, 0x88, b, sizeof(b))) return false;

  auto u16 = [&](int off) -> uint16_t { return (uint16_t)b[off] | ((uint16_t)b[off + 1] << 8); };
  auto s16 = [&](int off) -> int16_t { return (int16_t)u16(off); };

  c.dig_T1 = u16(0);
  c.dig_T2 = s16(2);
  c.dig_T3 = s16(4);
  c.dig_P1 = u16(6);
  c.dig_P2 = s16(8);
  c.dig_P3 = s16(10);
  c.dig_P4 = s16(12);
  c.dig_P5 = s16(14);
  c.dig_P6 = s16(16);
  c.dig_P7 = s16(18);
  c.dig_P8 = s16(20);
  c.dig_P9 = s16(22);
  return true;
}

bool bmp280Configure(TwoWire& wire, uint8_t addr) {
  // ctrl_meas (0xF4): temp oversampling x1, press oversampling x1, normal mode
  // config (0xF5): standby 250ms, filter off
  const uint8_t cfg1[2] = {0xF5, 0b01000000};
  const uint8_t cfg2[2] = {0xF4, 0b00100111};
  return i2cWriteBytes(wire, addr, cfg1, 2) && i2cWriteBytes(wire, addr, cfg2, 2);
}

bool bmp280ReadRaw(TwoWire& wire, uint8_t addr, int32_t& adc_T, int32_t& adc_P) {
  uint8_t b[6]{};
  if (!i2cReadBytes(wire, addr, 0xF7, b, sizeof(b))) return false;
  adc_P = ((int32_t)b[0] << 12) | ((int32_t)b[1] << 4) | ((int32_t)b[2] >> 4);
  adc_T = ((int32_t)b[3] << 12) | ((int32_t)b[4] << 4) | ((int32_t)b[5] >> 4);
  return true;
}

bool bmp280Compensate(const Bmp280Calib& c, int32_t adc_T, int32_t adc_P, float& outC, float& outHpa) {
  // From BMP280 datasheet (integer compensation)
  int32_t var1, var2;
  int32_t t_fine;
  var1 = ((((adc_T >> 3) - ((int32_t)c.dig_T1 << 1))) * ((int32_t)c.dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)c.dig_T1)) * ((adc_T >> 4) - ((int32_t)c.dig_T1))) >> 12) * ((int32_t)c.dig_T3)) >> 14;
  t_fine = var1 + var2;
  int32_t T = (t_fine * 5 + 128) >> 8;
  outC = (float)T / 100.0f;

  int64_t var1p, var2p, p;
  var1p = ((int64_t)t_fine) - 128000;
  var2p = var1p * var1p * (int64_t)c.dig_P6;
  var2p = var2p + ((var1p * (int64_t)c.dig_P5) << 17);
  var2p = var2p + (((int64_t)c.dig_P4) << 35);
  var1p = ((var1p * var1p * (int64_t)c.dig_P3) >> 8) + ((var1p * (int64_t)c.dig_P2) << 12);
  var1p = (((((int64_t)1) << 47) + var1p) * (int64_t)c.dig_P1) >> 33;
  if (var1p == 0) return false;
  p = 1048576 - adc_P;
  p = (((p << 31) - var2p) * 3125) / var1p;
  var1p = (((int64_t)c.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2p = (((int64_t)c.dig_P8) * p) >> 19;
  p = ((p + var1p + var2p) >> 8) + (((int64_t)c.dig_P7) << 4);
  // p is in Q24.8 Pa
  const float pa = (float)p / 256.0f;
  outHpa = pa / 100.0f;
  return true;
}

bool bmp280ReadTempPress(TwoWire& wire, uint8_t addr, float& outC, float& outHpa) {
  uint8_t id = 0;
  if (!i2cReadBytes(wire, addr, 0xD0, &id, 1)) return false;
  if (id != 0x58) return false;

  Bmp280Calib calib{};
  if (!bmp280ReadCalib(wire, addr, calib)) return false;
  (void)bmp280Configure(wire, addr);
  delay(20);

  int32_t adc_T = 0, adc_P = 0;
  if (!bmp280ReadRaw(wire, addr, adc_T, adc_P)) return false;
  return bmp280Compensate(calib, adc_T, adc_P, outC, outHpa);
}

} // namespace

const char* i2cSensorTypeName(I2cSensorType t) {
  switch (t) {
    case I2cSensorType::BMP180: return "BMP180";
    case I2cSensorType::BMP280: return "BMP280";
    case I2cSensorType::BME280: return "BME280";
    case I2cSensorType::TMP102: return "TMP102";
    case I2cSensorType::SHT3X: return "SHT3X";
    case I2cSensorType::SHT2X: return "SHT21/HTU21D";
    case I2cSensorType::AHTxx: return "AHT10/20/21/30";
    case I2cSensorType::TSL2561: return "TSL2561";
    case I2cSensorType::VL53L0X: return "VL53L0X";
    default: return "UNKNOWN";
  }
}

// -------------------- BME280 (temp + pressure + humidity) --------------------
struct Bme280Calib {
  uint16_t dig_T1;
  int16_t dig_T2;
  int16_t dig_T3;
  uint16_t dig_P1;
  int16_t dig_P2;
  int16_t dig_P3;
  int16_t dig_P4;
  int16_t dig_P5;
  int16_t dig_P6;
  int16_t dig_P7;
  int16_t dig_P8;
  int16_t dig_P9;
  uint8_t dig_H1;
  int16_t dig_H2;
  uint8_t dig_H3;
  int16_t dig_H4;
  int16_t dig_H5;
  int8_t dig_H6;
};

bool bme280ReadCalib(TwoWire& wire, uint8_t addr, Bme280Calib& c) {
  uint8_t b1[26]{};
  if (!i2cReadBytes(wire, addr, 0x88, b1, sizeof(b1))) return false;
  uint8_t h1 = 0;
  if (!i2cReadBytes(wire, addr, 0xA1, &h1, 1)) return false;
  uint8_t b2[7]{};
  if (!i2cReadBytes(wire, addr, 0xE1, b2, sizeof(b2))) return false;

  auto u16 = [&](int off) -> uint16_t { return (uint16_t)b1[off] | ((uint16_t)b1[off + 1] << 8); };
  auto s16 = [&](int off) -> int16_t { return (int16_t)u16(off); };

  c.dig_T1 = u16(0);
  c.dig_T2 = s16(2);
  c.dig_T3 = s16(4);
  c.dig_P1 = u16(6);
  c.dig_P2 = s16(8);
  c.dig_P3 = s16(10);
  c.dig_P4 = s16(12);
  c.dig_P5 = s16(14);
  c.dig_P6 = s16(16);
  c.dig_P7 = s16(18);
  c.dig_P8 = s16(20);
  c.dig_P9 = s16(22);

  c.dig_H1 = h1;
  c.dig_H2 = (int16_t)((uint16_t)b2[0] | ((uint16_t)b2[1] << 8));
  c.dig_H3 = b2[2];
  // H4 and H5 are packed
  c.dig_H4 = (int16_t)((int16_t)b2[3] << 4 | (b2[4] & 0x0F));
  c.dig_H5 = (int16_t)((int16_t)b2[5] << 4 | (b2[4] >> 4));
  c.dig_H6 = (int8_t)b2[6];
  return true;
}

bool bme280Configure(TwoWire& wire, uint8_t addr) {
  // ctrl_hum (0xF2): oversampling x1
  // ctrl_meas (0xF4): temp x1, press x1, normal mode
  // config (0xF5): standby 250ms, filter off
  const uint8_t hum[2] = {0xF2, 0x01};
  const uint8_t cfg[2] = {0xF5, 0b01000000};
  const uint8_t meas[2] = {0xF4, 0b00100111};
  return i2cWriteBytes(wire, addr, hum, 2) && i2cWriteBytes(wire, addr, cfg, 2) && i2cWriteBytes(wire, addr, meas, 2);
}

bool bme280ReadRaw(TwoWire& wire, uint8_t addr, int32_t& adc_T, int32_t& adc_P, int32_t& adc_H) {
  uint8_t b[8]{};
  if (!i2cReadBytes(wire, addr, 0xF7, b, sizeof(b))) return false;
  adc_P = ((int32_t)b[0] << 12) | ((int32_t)b[1] << 4) | ((int32_t)b[2] >> 4);
  adc_T = ((int32_t)b[3] << 12) | ((int32_t)b[4] << 4) | ((int32_t)b[5] >> 4);
  adc_H = ((int32_t)b[6] << 8) | (int32_t)b[7];
  return true;
}

bool bme280Compensate(const Bme280Calib& c, int32_t adc_T, int32_t adc_P, int32_t adc_H, float& outC, float& outHpa, float& outHum) {
  // Bosch reference integer compensation
  int32_t var1, var2;
  int32_t t_fine;
  var1 = ((((adc_T >> 3) - ((int32_t)c.dig_T1 << 1))) * ((int32_t)c.dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)c.dig_T1)) * ((adc_T >> 4) - ((int32_t)c.dig_T1))) >> 12) * ((int32_t)c.dig_T3)) >> 14;
  t_fine = var1 + var2;
  int32_t T = (t_fine * 5 + 128) >> 8;
  outC = (float)T / 100.0f;

  int64_t var1p, var2p, p;
  var1p = ((int64_t)t_fine) - 128000;
  var2p = var1p * var1p * (int64_t)c.dig_P6;
  var2p = var2p + ((var1p * (int64_t)c.dig_P5) << 17);
  var2p = var2p + (((int64_t)c.dig_P4) << 35);
  var1p = ((var1p * var1p * (int64_t)c.dig_P3) >> 8) + ((var1p * (int64_t)c.dig_P2) << 12);
  var1p = (((((int64_t)1) << 47) + var1p) * (int64_t)c.dig_P1) >> 33;
  if (var1p == 0) return false;
  p = 1048576 - adc_P;
  p = (((p << 31) - var2p) * 3125) / var1p;
  var1p = (((int64_t)c.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2p = (((int64_t)c.dig_P8) * p) >> 19;
  p = ((p + var1p + var2p) >> 8) + (((int64_t)c.dig_P7) << 4);
  const float pa = (float)p / 256.0f;
  outHpa = pa / 100.0f;

  // Humidity
  int32_t v_x1_u32r;
  v_x1_u32r = (t_fine - ((int32_t)76800));
  v_x1_u32r = (((((adc_H << 14) - (((int32_t)c.dig_H4) << 20) - (((int32_t)c.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
              (((((((v_x1_u32r * ((int32_t)c.dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)c.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
                 ((int32_t)c.dig_H2) + 8192) >> 14));
  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)c.dig_H1)) >> 4));
  if (v_x1_u32r < 0) v_x1_u32r = 0;
  if (v_x1_u32r > 419430400) v_x1_u32r = 419430400;
  uint32_t H = (uint32_t)(v_x1_u32r >> 12);
  outHum = (float)H / 1024.0f;

  return true;
}

bool bme280ReadTempPressHum(TwoWire& wire, uint8_t addr, float& outC, float& outHpa, float& outHum) {
  uint8_t id = 0;
  if (!i2cReadBytes(wire, addr, 0xD0, &id, 1)) return false;
  if (id != 0x60) return false;

  Bme280Calib calib{};
  if (!bme280ReadCalib(wire, addr, calib)) return false;
  (void)bme280Configure(wire, addr);
  delay(20);

  int32_t adc_T = 0, adc_P = 0, adc_H = 0;
  if (!bme280ReadRaw(wire, addr, adc_T, adc_P, adc_H)) return false;
  return bme280Compensate(calib, adc_T, adc_P, adc_H, outC, outHpa, outHum);
}

std::vector<I2cSensorInfo> i2cDetectKnownSensors(TwoWire& wire) {
  std::vector<I2cSensorInfo> found;

  for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
    wire.beginTransmission(addr);
    const uint8_t err = wire.endTransmission();
    if (err != 0) continue;

    I2cSensorInfo info{};
    info.address = addr;
    info.last_seen_ms = millis();

    // BMP180/BMP280/BME280
    if (addr == 0x76 || addr == 0x77) {
      uint8_t id = 0;
      if (i2cReadBytes(wire, addr, 0xD0, &id, 1)) {
        if (id == 0x55) {
          info.type = I2cSensorType::BMP180;
          found.push_back(info);
          continue;
        }
        if (id == 0x60) {
          info.type = I2cSensorType::BME280;
          found.push_back(info);
          continue;
        }
        if (id == 0x58) {
          info.type = I2cSensorType::BMP280;
          found.push_back(info);
          continue;
        }
      }
    }

    // TMP102 typical addresses
    if (addr >= 0x48 && addr <= 0x4B) {
      info.type = I2cSensorType::TMP102;
      found.push_back(info);
      continue;
    }

    // VL53L0X typical address (0x29). Prefer it over TSL2561 when both use 0x29.
    if (addr == 0x29 && vl53l0xDetect(wire, addr)) {
      info.type = I2cSensorType::VL53L0X;
      float mm = NAN;
      if (vl53l0xReadDistanceMm(wire, addr, mm)) {
        info.reading.ok = true;
        info.reading.distance_mm = mm;
      }
      found.push_back(info);
      continue;
    }

    // TSL2561 typical addresses
    if (addr == 0x29 || addr == 0x39 || addr == 0x49) {
      float lux = NAN;
      if (tsl2561ReadLux(wire, addr, lux)) {
        info.type = I2cSensorType::TSL2561;
        info.reading.ok = true;
        info.reading.light_lux = lux;
        found.push_back(info);
        continue;
      }
    }

    // SHT3x typical addresses (probe by doing a measurement)
    if (addr == 0x44 || addr == 0x45) {
      float t = NAN, h = NAN;
      if (sht3xReadTempHum(wire, addr, t, h)) {
        info.type = I2cSensorType::SHT3X;
        info.reading.ok = true;
        info.reading.temperature_c = t;
        info.reading.humidity_pct = h;
        found.push_back(info);
        continue;
      }
    }

    // SHT2x / HTU21D / SI70xx typical address
    if (addr == 0x40) {
      float t = NAN, h = NAN;
      if (sht2xReadTempHum(wire, addr, t, h)) {
        info.type = I2cSensorType::SHT2X;
        info.reading.ok = true;
        info.reading.temperature_c = t;
        info.reading.humidity_pct = h;
        found.push_back(info);
        continue;
      }
    }

    // AHT10/20/21/30 typical address (probe by doing a measurement)
    if (addr == 0x38) {
      float t = NAN, h = NAN;
      if (ahtxxReadTempHum(wire, addr, t, h)) {
        info.type = I2cSensorType::AHTxx;
        info.reading.ok = true;
        info.reading.temperature_c = t;
        info.reading.humidity_pct = h;
        found.push_back(info);
        continue;
      }
    }

    // Unknown device; ignore for now (we only show known drivers).
  }

  return found;
}

std::vector<uint8_t> i2cScanAllAddresses(TwoWire& wire) {
  std::vector<uint8_t> found;

  for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
    wire.beginTransmission(addr);
    const uint8_t err = wire.endTransmission();
    if (err == 0) found.push_back(addr);
  }

  return found;
}

void i2cUpdateReadings(TwoWire& wire, std::vector<I2cSensorInfo>& sensors) {
  for (auto& s : sensors) {
    I2cSensorReading r{};
    r.ok = false;

    if (s.type == I2cSensorType::TMP102) {
      float t = NAN;
      if (tmp102ReadTempC(wire, s.address, t)) {
        r.ok = true;
        r.temperature_c = t;
      }
    } else if (s.type == I2cSensorType::SHT3X) {
      float t = NAN, h = NAN;
      if (sht3xReadTempHum(wire, s.address, t, h)) {
        r.ok = true;
        r.temperature_c = t;
        r.humidity_pct = h;
      }
    } else if (s.type == I2cSensorType::SHT2X) {
      float t = NAN, h = NAN;
      if (sht2xReadTempHum(wire, s.address, t, h)) {
        r.ok = true;
        r.temperature_c = t;
        r.humidity_pct = h;
      }
    } else if (s.type == I2cSensorType::AHTxx) {
      float t = NAN, h = NAN;
      if (ahtxxReadTempHum(wire, s.address, t, h)) {
        r.ok = true;
        r.temperature_c = t;
        r.humidity_pct = h;
      }
    } else if (s.type == I2cSensorType::TSL2561) {
      float lux = NAN;
      if (tsl2561ReadLux(wire, s.address, lux)) {
        r.ok = true;
        r.light_lux = lux;
      }
    } else if (s.type == I2cSensorType::VL53L0X) {
      float mm = NAN;
      if (vl53l0xReadDistanceMm(wire, s.address, mm)) {
        r.ok = true;
        r.distance_mm = mm;
      }
    } else if (s.type == I2cSensorType::BMP180) {
      float t = NAN, p = NAN;
      if (bmp180ReadTempPress(wire, s.address, t, p)) {
        r.ok = true;
        r.temperature_c = t;
        r.pressure_hpa = p;
      }
    } else if (s.type == I2cSensorType::BMP280) {
      float t = NAN, p = NAN;
      if (bmp280ReadTempPress(wire, s.address, t, p)) {
        r.ok = true;
        r.temperature_c = t;
        r.pressure_hpa = p;
      }
    } else if (s.type == I2cSensorType::BME280) {
      float t = NAN, p = NAN, h = NAN;
      if (bme280ReadTempPressHum(wire, s.address, t, p, h)) {
        r.ok = true;
        r.temperature_c = t;
        r.pressure_hpa = p;
        r.humidity_pct = h;
      }
    }

    if (r.ok) {
      s.reading = r;
      s.last_seen_ms = millis();
    } else {
      s.reading.ok = false;
    }
  }
}

#endif // NETTEMP_ENABLE_I2C
