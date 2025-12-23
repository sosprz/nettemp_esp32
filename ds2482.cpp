#include "ds2482.h"

#include <OneWire.h>

namespace ds2482 {

namespace {

constexpr uint8_t CMD_DEVICE_RESET = 0xF0;
constexpr uint8_t CMD_SET_READ_PTR = 0xE1;
constexpr uint8_t CMD_WRITE_CONFIG = 0xD2;
constexpr uint8_t CMD_1W_RESET = 0xB4;
constexpr uint8_t CMD_1W_WRITE_BYTE = 0xA5;
constexpr uint8_t CMD_1W_READ_BYTE = 0x96;
constexpr uint8_t CMD_1W_TRIPLET = 0x78;
constexpr uint8_t CMD_CHANNEL_SELECT = 0xC3;

constexpr uint8_t PTR_STATUS = 0xF0;
constexpr uint8_t PTR_DATA = 0xE1;

constexpr uint8_t STATUS_1WB = 0x01;
constexpr uint8_t STATUS_PPD = 0x02;
constexpr uint8_t STATUS_SBR = 0x20;
constexpr uint8_t STATUS_TSB = 0x40;
constexpr uint8_t STATUS_DIR = 0x80;

bool writeBytes(TwoWire& wire, uint8_t addr, const uint8_t* data, size_t len) {
  wire.beginTransmission(addr);
  for (size_t i = 0; i < len; i++) wire.write(data[i]);
  return wire.endTransmission(true) == 0;
}

bool readBytes(TwoWire& wire, uint8_t addr, uint8_t* out, size_t len) {
  const size_t got = wire.requestFrom((int)addr, (int)len);
  if (got != len) return false;
  for (size_t i = 0; i < len; i++) out[i] = (uint8_t)wire.read();
  return true;
}

bool setReadPtr(TwoWire& wire, uint8_t addr, uint8_t ptr) {
  const uint8_t cmd[2] = {CMD_SET_READ_PTR, ptr};
  return writeBytes(wire, addr, cmd, 2);
}

bool readStatus(TwoWire& wire, uint8_t addr, uint8_t& out) {
  if (!setReadPtr(wire, addr, PTR_STATUS)) return false;
  return readBytes(wire, addr, &out, 1);
}

bool waitReady(TwoWire& wire, uint8_t addr, uint8_t& statusOut) {
  for (int i = 0; i < 50; i++) {
    if (!readStatus(wire, addr, statusOut)) return false;
    if ((statusOut & STATUS_1WB) == 0) return true;
    delay(1);
  }
  return false;
}

bool readData(TwoWire& wire, uint8_t addr, uint8_t& out) {
  if (!setReadPtr(wire, addr, PTR_DATA)) return false;
  return readBytes(wire, addr, &out, 1);
}

bool writeConfig(TwoWire& wire, uint8_t addr, uint8_t cfg) {
  const uint8_t cfgByte = (uint8_t)(cfg | ((~cfg) << 4));
  const uint8_t cmd[2] = {CMD_WRITE_CONFIG, cfgByte};
  return writeBytes(wire, addr, cmd, 2);
}

} // namespace

bool begin(TwoWire& wire, uint8_t addr) {
  const uint8_t cmd = CMD_DEVICE_RESET;
  if (!writeBytes(wire, addr, &cmd, 1)) return false;
  delay(2);
  uint8_t status = 0;
  if (!readStatus(wire, addr, status)) return false;
  // Enable active pull-up, disable strong pull-up/1-wire speed boost.
  return writeConfig(wire, addr, 0x01);
}

bool setConfig(TwoWire& wire, uint8_t addr, uint8_t cfg) {
  return writeConfig(wire, addr, cfg);
}

bool selectChannel(TwoWire& wire, uint8_t addr, uint8_t channel) {
  if (channel == 0) return true;
  struct ChannelMap { uint8_t code; uint8_t resp; };
  const ChannelMap map[] = {
    {0xF0, 0xB8}, {0xE1, 0xB1}, {0xD2, 0xAA}, {0xC3, 0xA3},
    {0xB4, 0x9C}, {0xA5, 0x95}, {0x96, 0x8E}, {0x87, 0x87},
  };
  if (channel >= (sizeof(map) / sizeof(map[0]))) return false;
  const uint8_t cmd[2] = {CMD_CHANNEL_SELECT, map[channel].code};
  if (!writeBytes(wire, addr, cmd, 2)) return false;
  uint8_t resp = 0;
  if (!readBytes(wire, addr, &resp, 1)) return false;
  return resp == map[channel].resp;
}

bool oneWireReset(TwoWire& wire, uint8_t addr) {
  const uint8_t cmd = CMD_1W_RESET;
  if (!writeBytes(wire, addr, &cmd, 1)) return false;
  uint8_t status = 0;
  if (!waitReady(wire, addr, status)) return false;
  return (status & STATUS_PPD) != 0;
}

bool oneWireWriteByte(TwoWire& wire, uint8_t addr, uint8_t data) {
  const uint8_t cmd[2] = {CMD_1W_WRITE_BYTE, data};
  if (!writeBytes(wire, addr, cmd, 2)) return false;
  uint8_t status = 0;
  return waitReady(wire, addr, status);
}

bool oneWireReadByte(TwoWire& wire, uint8_t addr, uint8_t& out) {
  const uint8_t cmd = CMD_1W_READ_BYTE;
  if (!writeBytes(wire, addr, &cmd, 1)) return false;
  uint8_t status = 0;
  if (!waitReady(wire, addr, status)) return false;
  return readData(wire, addr, out);
}

bool oneWireTriplet(TwoWire& wire, uint8_t addr, bool dir, uint8_t& status) {
  const uint8_t cmd[2] = {CMD_1W_TRIPLET, (uint8_t)(dir ? 0x80 : 0x00)};
  if (!writeBytes(wire, addr, cmd, 2)) return false;
  return waitReady(wire, addr, status);
}

bool searchRoms(TwoWire& wire, uint8_t addr, std::vector<std::array<uint8_t, 8>>& outRoms) {
  outRoms.clear();
  std::array<uint8_t, 8> lastRom{};
  uint8_t lastDiscrepancy = 0;
  bool lastDevice = false;

  while (!lastDevice) {
    if (!oneWireReset(wire, addr)) break;
    if (!oneWireWriteByte(wire, addr, 0xF0)) break; // SEARCH ROM

    std::array<uint8_t, 8> rom{};
    uint8_t discrepancy = 0;

    for (uint8_t bitIndex = 1; bitIndex <= 64; bitIndex++) {
      const uint8_t byteIndex = (uint8_t)((bitIndex - 1) >> 3);
      const uint8_t bitMask = (uint8_t)(1 << ((bitIndex - 1) & 7));
      bool dir = false;

      if (bitIndex < lastDiscrepancy) {
        dir = (lastRom[byteIndex] & bitMask) != 0;
      } else if (bitIndex == lastDiscrepancy) {
        dir = true;
      }

      uint8_t status = 0;
      if (!oneWireTriplet(wire, addr, dir, status)) return !outRoms.empty();
      const bool sbr = (status & STATUS_SBR) != 0;
      const bool tsb = (status & STATUS_TSB) != 0;
      const bool dirBit = (status & STATUS_DIR) != 0;

      if (sbr && tsb) return !outRoms.empty();
      if (!sbr && !tsb && !dirBit) discrepancy = bitIndex;
      if (dirBit) rom[byteIndex] |= bitMask;
    }

    lastDiscrepancy = discrepancy;
    if (lastDiscrepancy == 0) lastDevice = true;

    if (OneWire::crc8(rom.data(), 7) != rom[7]) {
      lastRom = rom;
      continue;
    }
    if (rom[0] == 0x28 || rom[0] == 0x22 || rom[0] == 0x10) outRoms.push_back(rom);
    lastRom = rom;
  }

  return !outRoms.empty();
}

bool startConvertAll(TwoWire& wire, uint8_t addr) {
  if (!oneWireReset(wire, addr)) return false;
  if (!oneWireWriteByte(wire, addr, 0xCC)) return false; // SKIP ROM
  return oneWireWriteByte(wire, addr, 0x44); // CONVERT T
}

bool readTempC(TwoWire& wire, uint8_t addr, const std::array<uint8_t, 8>& rom, float& outTempC) {
  outTempC = NAN;
  if (!oneWireReset(wire, addr)) return false;
  if (!oneWireWriteByte(wire, addr, 0x55)) return false; // MATCH ROM
  for (size_t i = 0; i < rom.size(); i++) {
    if (!oneWireWriteByte(wire, addr, rom[i])) return false;
  }
  if (!oneWireWriteByte(wire, addr, 0xBE)) return false; // READ SCRATCHPAD

  uint8_t data[9]{};
  for (int i = 0; i < 9; i++) {
    if (!oneWireReadByte(wire, addr, data[i])) return false;
  }
  if (OneWire::crc8(data, 8) != data[8]) return false;

  const int16_t raw = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
  const uint8_t family = rom[0];
  if (family == 0x28 || family == 0x22) {
    outTempC = (float)raw / 16.0f;
  } else if (family == 0x10) {
    outTempC = (float)raw / 2.0f;
  } else {
    return false;
  }
  return true;
}

} // namespace ds2482
