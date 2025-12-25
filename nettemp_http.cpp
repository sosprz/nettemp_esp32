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
  if (!batch.endpoint.length() && !batch.baseUrl.length()) {
    Serial.println("Server send failed: baseUrl not configured");
    return false;
  }
  if (batch.requireApiKey && !batch.apiKey.length()) {
    Serial.println("Server send failed: apiKey not configured");
    return false;
  }
  if (!batch.deviceId.length()) {
    Serial.println("Server send failed: deviceId not configured");
    return false;
  }
  if (batch.readings.empty()) {
    Serial.println("Server send skipped: no readings to send");
    return true;
  }

  const String endpoint = batch.endpoint.length() ? batch.endpoint : joinUrl(batch.baseUrl, "/api/v1/data");
  const String body = buildPayload(batch);

  Serial.printf("Sending %u reading(s) to server: %s\n", (unsigned)batch.readings.size(), endpoint.c_str());

  // Auto-detect HTTP vs HTTPS
  const bool isHttps = endpoint.startsWith("https://");

  HTTPClient http;
  bool beginOk = false;
  if (isHttps) {
    // HTTPS - use secure client
    beginOk = http.begin(client, endpoint);
    if (!beginOk) {
      Serial.println("Server send failed: HTTPS begin failed");
    }
  } else {
    // HTTP - don't use secure client
    beginOk = http.begin(endpoint);
    if (!beginOk) {
      Serial.println("Server send failed: HTTP begin failed");
    }
  }

  if (!beginOk) {
    http.end();  // Ensure cleanup even on begin failure
    return false;
  }

  // Set timeouts to prevent indefinite hanging (critical for async operation)
  http.setConnectTimeout(5000);  // 5 second connection timeout
  http.setTimeout(10000);         // 10 second total timeout

  http.addHeader("Content-Type", "application/json");
  if (batch.apiKey.length()) {
    http.addHeader("Authorization", String("Bearer ") + batch.apiKey);
  }

  const int code = http.POST((uint8_t*)body.c_str(), body.length());
  http.end();

  if (code >= 200 && code < 300) {
    Serial.printf("Server send OK: HTTP %d\n", code);
  } else {
    Serial.printf("Server send failed: HTTP %d\n", code);
  }

  return code >= 200 && code < 300;
}

#endif // NETTEMP_ENABLE_SERVER
