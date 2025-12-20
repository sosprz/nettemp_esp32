#include "nettemp_http.h"

#if NETTEMP_ENABLE_SERVER

#include <HTTPClient.h>

namespace {

static String joinUrl(const String& base, const String& path) {
  if (base.endsWith("/")) return base + path.substring(1);
  if (path.startsWith("/")) return base + path;
  return base + "/" + path;
}

static void jsonAppendEscaped(String& out, const String& s) {
  for (size_t i = 0; i < s.length(); i++) {
    const char c = s[i];
    if (c == '\\' || c == '"') {
      out += '\\';
      out += c;
    } else if (c == '\n') {
      out += "\\n";
    } else if (c == '\r') {
      out += "\\r";
    } else if (c == '\t') {
      out += "\\t";
    } else {
      out += c;
    }
  }
}

static String buildPayload(const NettempBatch& batch) {
  String json;
  json.reserve(512 + batch.readings.size() * 160);

  json += "{\"device_id\":\"";
  jsonAppendEscaped(json, batch.deviceId);
  json += "\",\"readings\":[";

  for (size_t i = 0; i < batch.readings.size(); i++) {
    const auto& r = batch.readings[i];
    if (i) json += ",";
    json += "{";
    json += "\"sensor_id\":\"";
    jsonAppendEscaped(json, r.sensorId);
    json += "\",\"value\":";
    json += String(r.value, 3);

    if (r.sensorType.length()) {
      json += ",\"sensor_type\":\"";
      jsonAppendEscaped(json, r.sensorType);
      json += "\"";
    }
    if (r.friendlyName.length()) {
      json += ",\"metadata\":{\"name\":\"";
      jsonAppendEscaped(json, r.friendlyName);
      json += "\"}";
    }
    json += "}";
  }

  json += "]}";
  return json;
}

} // namespace

bool nettempPostBatch(WiFiClientSecure& client, const NettempBatch& batch) {
  if (!batch.baseUrl.length() || !batch.apiKey.length() || !batch.deviceId.length()) return false;
  if (batch.readings.empty()) return true;

  const String endpoint = joinUrl(batch.baseUrl, "/api/v1/data");
  const String body = buildPayload(batch);

  HTTPClient http;
  if (!http.begin(client, endpoint)) return false;

  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", String("Bearer ") + batch.apiKey);

  const int code = http.POST((uint8_t*)body.c_str(), body.length());
  http.end();

  return code >= 200 && code < 300;
}

#endif // NETTEMP_ENABLE_SERVER
