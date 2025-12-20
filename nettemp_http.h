#pragma once

#ifndef NETTEMP_ENABLE_SERVER
#define NETTEMP_ENABLE_SERVER 1
#endif

#if NETTEMP_ENABLE_SERVER

#include <vector>
#include <Arduino.h>
#include <WiFiClientSecure.h>

struct NettempReading {
  String sensorId;
  float value;
  String sensorType;
  String unit;
  uint32_t timestamp;
  String friendlyName;
};

struct NettempBatch {
  String baseUrl;  // e.g. https://api.nettemp.pl
  String apiKey;   // ntk_...
  String deviceId; // device_id in Nettemp
  std::vector<NettempReading> readings;
};

// Best-effort POST to Nettemp Cloud API (/api/v1/data).
// Returns true if HTTP response code is 2xx.
bool nettempPostBatch(WiFiClientSecure& client, const NettempBatch& batch);

#else
// Stubs for builds that disable server sending to reduce flash usage.
struct NettempReading {};
struct NettempBatch {};
inline bool nettempPostBatch(void*, const NettempBatch&) { return false; }
#endif
