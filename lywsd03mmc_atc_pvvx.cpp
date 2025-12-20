#include "lywsd03mmc_atc_pvvx.h"

namespace {

static uint16_t readU16LE(const uint8_t* p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint16_t readU16BE(const uint8_t* p) {
  return ((uint16_t)p[0] << 8) | (uint16_t)p[1];
}

static int16_t readI16LE(const uint8_t* p) {
  return (int16_t)readU16LE(p);
}

static bool looksSane(float tC, float hPct, int batt) {
  if (std::isnan(tC) || std::isnan(hPct)) return false;
  if (tC < -50.0f || tC > 85.0f) return false;
  if (hPct < 0.0f || hPct > 100.0f) return false;
  if (batt < -1 || batt > 100) return false;
  return true;
}

static bool looksSaneVoltageMv(int mv) {
  return mv < 0 || (mv >= 1500 && mv <= 4000);
}

// ATC1441/PVVX "custom" 0x181A service data is commonly:
// [MAC(6)][temp_c*100 int16 LE][hum uint8][batt uint8][volt_mv uint16 LE][cnt uint8][flags uint8]
// Total: 6 + 2 + 1 + 1 + 2 + 1 + 1 = 14 bytes
static bool parseAtc181AFrame(const uint8_t* buf, size_t len, LYWSD03Reading& out) {
  if (len < 11) return false; // need at least mac+temp+hum+batt

  out.has_mac = true;
  for (int i = 0; i < 6; i++) out.mac[i] = buf[i];

  // Variant A (PVVX/ATC legacy): temp_x100 int16 LE, volt_mv u16 LE, optional cnt/flags
  if (len >= 11) {
    const int16_t tempX100 = readI16LE(buf + 6);
    const float tempC = tempX100 / 100.0f;
    const float hum = (float)buf[8];
    const int batt = (int)buf[9];
    int mv = -1;
    if (len >= 12) mv = (int)readU16LE(buf + 10);

    if (looksSane(tempC, hum, batt) && looksSaneVoltageMv(mv)) {
      out.temperature_c = tempC;
      out.humidity_pct = hum;
      out.battery_pct = batt;
      out.voltage_mv = mv;
      if (len >= 13) out.counter = (int)buf[12];
      if (len >= 14) out.flags = (int)buf[13];
      return true;
    }
  }

  // Variant B (ATC1441 with AdFlags as seen in TheengsGateway):
  // [MAC(6)][temp_x10 u16 BE][hum u8][batt u8][volt_mv u16 BE][cnt u8] -> 13 bytes
  if (len >= 13) {
    const uint16_t tempX10 = readU16BE(buf + 6);
    const float tempC = (float)tempX10 / 10.0f;
    const float hum = (float)buf[8];
    const int batt = (int)buf[9];
    const int mv = (int)readU16BE(buf + 10);

    if (!looksSane(tempC, hum, batt)) return false;
    if (!looksSaneVoltageMv(mv)) return false;

    out.temperature_c = tempC;
    out.humidity_pct = hum;
    out.battery_pct = batt;
    out.voltage_mv = mv;
    out.counter = (int)buf[12];
    out.flags = -1;
    return true;
  }

  return false;
}

} // namespace

bool parseLywsd03FromServiceData(const std::string& serviceData, LYWSD03Reading& out) {
  // NimBLE returns the raw serviceData payload bytes (not including the UUID).
  // For ATC/PVVX 0x181A service data, the payload begins with the MAC.
  const auto* b = reinterpret_cast<const uint8_t*>(serviceData.data());
  const size_t n = serviceData.size();
  if (!b || n < 11) return false;
  if (parseAtc181AFrame(b, n, out)) return true;

  // Some stacks return service data that still includes the 16-bit UUID (0x181A) at the start.
  // Try skipping it.
  if (n >= 13 && b[0] == 0x1A && b[1] == 0x18) {
    return parseAtc181AFrame(b + 2, n - 2, out);
  }

  return false;
}

bool parseLywsd03FromManufacturerData(const std::string& mfgData, LYWSD03Reading& out) {
  // Manufacturer data is less standardized across forks; we try a couple of heuristics.
  const auto* b = reinterpret_cast<const uint8_t*>(mfgData.data());
  const size_t n = mfgData.size();
  if (!b || n < 8) return false;

  // Many manufacturer payloads start with 2 bytes company ID (LE).
  // Try skipping 2 bytes and parse as if it was 0x181A payload.
  if (n >= 13) {
    if (parseAtc181AFrame(b + 2, n - 2, out)) return true;
  }

  // Some variants embed the frame without a company ID (rare).
  if (parseAtc181AFrame(b, n, out)) return true;

  return false;
}
