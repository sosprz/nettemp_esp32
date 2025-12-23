#pragma once

#include <Arduino.h>
#include <Wire.h>

#include <array>
#include <vector>

namespace ds2482 {

bool begin(TwoWire& wire, uint8_t addr);
bool setConfig(TwoWire& wire, uint8_t addr, uint8_t cfg);
bool selectChannel(TwoWire& wire, uint8_t addr, uint8_t channel);
bool oneWireReset(TwoWire& wire, uint8_t addr);
bool oneWireWriteByte(TwoWire& wire, uint8_t addr, uint8_t data);
bool oneWireReadByte(TwoWire& wire, uint8_t addr, uint8_t& out);
bool oneWireTriplet(TwoWire& wire, uint8_t addr, bool dir, uint8_t& status);

bool searchRoms(TwoWire& wire, uint8_t addr, std::vector<std::array<uint8_t, 8>>& outRoms);
bool startConvertAll(TwoWire& wire, uint8_t addr);
bool readTempC(TwoWire& wire, uint8_t addr, const std::array<uint8_t, 8>& rom, float& outTempC);

} // namespace ds2482
