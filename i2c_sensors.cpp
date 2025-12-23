#include "i2c_sensors.h"

#if NETTEMP_ENABLE_I2C

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
  delay(50);
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
    case I2cSensorType::BMP280: return "BMP280";
    case I2cSensorType::BME280: return "BME280";
    case I2cSensorType::TMP102: return "TMP102";
    case I2cSensorType::SHT3X: return "SHT3X";
    case I2cSensorType::SHT2X: return "SHT21/HTU21D";
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

    // BMP280
    if (addr == 0x76 || addr == 0x77) {
      uint8_t id = 0;
      if (i2cReadBytes(wire, addr, 0xD0, &id, 1)) {
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
