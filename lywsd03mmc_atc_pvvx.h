#pragma once

#include <cmath>
#include <cstdint>
#include <string>

struct LYWSD03Reading {
  bool has_mac = false;
  uint8_t mac[6]{};
  float temperature_c = NAN;
  float humidity_pct = NAN;
  int battery_pct = -1;
  int voltage_mv = -1;
  int counter = -1;
  int flags = -1;
};

// Parses common ATC/PVVX-style frames for LYWSD03MMC.
// These firmwares typically broadcast data either as:
// - Service Data for UUID 0x181A (Environmental Sensing)
// - Manufacturer Specific Data (varies by firmware)
//
// The goal here is pragmatic compatibility, not strict format coverage.
bool parseLywsd03FromServiceData(const std::string& serviceData, LYWSD03Reading& out);
bool parseLywsd03FromServiceDataAtc1441(const std::string& serviceData, LYWSD03Reading& out);
bool parseLywsd03FromManufacturerData(const std::string& mfgData, LYWSD03Reading& out);
