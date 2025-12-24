
#if NETTEMP_ENABLE_PORTAL
enum class WebMode {
  None,
  ApSetup,
  StaConfig,
};

WebMode g_webMode = WebMode::None;

static String portalSsid() {
  String ssid = "nettemp-setup-";
  ssid += String((uint32_t)(ESP.getEfuseMac() & 0xFFFFFFFFu), HEX);
  ssid.toLowerCase();
  return ssid;
}

static String htmlEscape(const String& s) {
  String out;
  out.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    const char c = s[i];
    if (c == '&') out += "&amp;";
    else if (c == '<') out += "&lt;";
    else if (c == '>') out += "&gt;";
    else if (c == '"') out += "&quot;";
    else out += c;
  }
  return out;
}

static String dsRomLinuxId(const std::array<uint8_t, 8>& rom, uint8_t bus, bool includeBus) {
  char out[32]{};
  char serial[13]{};
  // Linux w1 serial is bytes 1..6 reversed (no CRC).
  snprintf(serial, sizeof(serial), "%02X%02X%02X%02X%02X%02X",
           rom[6], rom[5], rom[4], rom[3], rom[2], rom[1]);
  if (includeBus) {
    snprintf(out, sizeof(out), "%02x-%02X_%s", (unsigned)bus, (unsigned)rom[0], serial);
  } else {
    snprintf(out, sizeof(out), "%02X_%s", (unsigned)rom[0], serial);
  }
  String s(out);
  s.toLowerCase();
  return s;
}

static String formatSendStatus(uint32_t lastSendMs, uint32_t intervalMs) {
  if (lastSendMs == 0) return String("Sent never");
  const uint32_t secAgo = (millis() - lastSendMs) / 1000UL;
  if (intervalMs == 0) return String("Sent ") + String(secAgo) + " sec ago";
  const uint32_t intervalS = intervalMs / 1000UL;
  if (intervalS == 0) return String("Sent ") + String(secAgo) + " sec ago";
  if (secAgo < intervalS) {
    const uint32_t nextIn = intervalS - secAgo;
    return String("Sent ") + String(secAgo) + " sec ago Next in " + String(nextIn) + " sec";
  }
  return String("Sent ") + String(secAgo) + " sec ago Ready";
}

static uint32_t parseU32Or(const String& s, uint32_t fallback) {
  if (!s.length()) return fallback;
  long v = s.toInt();
  if (v < 0) return fallback;
  return (uint32_t)v;
}

static void portalStop() {
  if (!g_portalRunning) return;
  g_dnsServer.stop();
  g_webServer.stop();
  WiFi.softAPdisconnect(true);
  g_portalRunning = false;
  g_webMode = WebMode::None;
}

static bool webRequireAuth() {
  if (g_webMode != WebMode::StaConfig) return true;
  if (g_cfg.webUser.length() == 0 || g_cfg.webPass.length() == 0) {
    g_webServer.send(500, "text/plain", "Web auth not configured");
    return false;
  }
  if (g_webServer.authenticate(g_cfg.webUser.c_str(), g_cfg.webPass.c_str())) return true;
  g_webServer.requestAuthentication();
  return false;
}

static void portalHandleRoot() {
  // AP captive portal: WiFi + admin credentials only
  const String ssid = htmlEscape(g_cfg.wifiSsid);
  const String body =
    String("<!doctype html><html><head><meta charset='utf-8'/>"
           "<meta name='viewport' content='width=device-width,initial-scale=1'/>"
           "<title>Nettemp Setup (WiFi)</title>"
           "<style>"
           "*,*:before,*:after{box-sizing:border-box}"
           ":root{--bg:#0b1220;--card:#111b2e;--muted:#9fb0cc;--text:#e8eefc;--accent:#2f6bff;--border:#243251}"
           "body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial;margin:0;background:var(--bg);color:var(--text)}"
           ".wrap{max-width:860px;margin:0 auto;padding:16px}"
           ".top{display:flex;align-items:center;gap:10px;margin:6px 0 14px}"
           ".logo{width:34px;height:34px;border-radius:10px;background:var(--accent);display:flex;align-items:center;justify-content:center;font-weight:700}"
           "h2{margin:0;font-size:20px}"
           "p{color:var(--muted);line-height:1.35}"
           ".card{background:var(--card);border:1px solid var(--border);border-radius:14px;padding:14px;margin:12px 0}"
           "label{display:block;margin:10px 0 6px;color:var(--muted);font-size:13px}"
           "input{max-width:100%;padding:8px 10px;border-radius:10px;border:1px solid var(--border);background:#0d1628;color:var(--text)}"
           "input[type=checkbox]{width:auto}"
           ".row{display:flex;gap:10px} .row>div{flex:1}"
           "code{background:#0d1628;border:1px solid var(--border);padding:2px 6px;border-radius:8px;color:var(--text)}"
           "button{background:var(--accent);border:none;color:white;padding:10px 14px;border-radius:10px;font-weight:600}"
           "</style>"
           "</head><body>"
           "<div class='wrap'>"
           "<div class='top'><div class='logo'>nt</div><div>"
           "<h2>Nettemp Setup</h2><p>Step 1/2: WiFi + Web Admin (Basic Auth)</p>"
           "</div></div>"
           "<div class='card'>"
           "<form method='POST' action='/save'>"
           "<h3 style='margin:0 0 10px'>WiFi</h3>"
           "<label>SSID</label><input name='wifiSsid' value='") + ssid + "'/>"
           "<label>Password</label><input name='wifiPass' type='password' value='' placeholder='(leave empty to keep existing)'/>"
           "<h3 style='margin:16px 0 10px'>Web Admin</h3>"
           "<label>Username</label><input name='webUser' value='" + htmlEscape(g_cfg.webUser) + "'/>"
           "<label>Password</label><input name='webPass' type='password' value='' placeholder='(required for web config server)'/>"
           "<label style='margin-top:14px'><input type='checkbox' name='apAuto' value='1' " + String(g_portalAuto ? "checked" : "") + "> Auto-start portal if WiFi not configured</label>"
           "<p style='margin:14px 0 0'><button type='submit'>Save & Reboot</button></p>"
           "</form>"
           "</div>"
           "<p>Portal SSID: <code>" + htmlEscape(portalSsid()) + "</code> (open network)</p>"
           "<p>Step 2/2: after WiFi connects, open <code>http://&lt;device-ip&gt;/</code> to configure MQTT/Server (Basic Auth).</p>"
           "</div></body></html>";
  g_webServer.send(200, "text/html; charset=utf-8", body);
}

static void portalHandleSave() {
  const String wifiSsid = g_webServer.arg("wifiSsid");
  const String wifiPass = g_webServer.arg("wifiPass");
  if (wifiSsid.length()) g_cfg.wifiSsid = wifiSsid;
  if (wifiPass.length()) g_cfg.wifiPass = wifiPass;

  const String webUser = g_webServer.arg("webUser");
  const String webPass = g_webServer.arg("webPass");
  if (webUser.length()) g_cfg.webUser = webUser;
  if (webPass.length()) g_cfg.webPass = webPass;

  g_portalAuto = g_webServer.hasArg("apAuto");

  prefsSave();
  g_webServer.send(200, "text/html; charset=utf-8",
                   "<!doctype html><html><head><meta charset='utf-8'/>"
                   "<meta name='viewport' content='width=device-width,initial-scale=1'/>"
                   "<title>Saved</title></head><body><h3>Saved.</h3><p>Rebooting...</p></body></html>");
  delay(400);
  ESP.restart();
}

static void portalStart() {
  if (g_portalRunning) return;
  WiFi.mode(WIFI_AP_STA);
  const IPAddress ip(192, 168, 4, 1);
  const IPAddress mask(255, 255, 255, 0);
  WiFi.softAPConfig(ip, ip, mask);
  WiFi.softAP(portalSsid().c_str()); // open network

  g_dnsServer.start(53, "*", ip);
  g_webServer.on("/", HTTP_GET, portalHandleRoot);
  g_webServer.on("/save", HTTP_POST, portalHandleSave);
  g_webServer.onNotFound([]() {
    g_webServer.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
    g_webServer.send(302, "text/plain", "");
  });
  g_webServer.begin();
  g_portalRunning = true;
  g_webMode = WebMode::ApSetup;
}

static void portalTick() {
  if (!g_portalRunning) return;
  if (g_webMode == WebMode::ApSetup) {
    g_dnsServer.processNextRequest();
  }
  g_webServer.handleClient();
}

static void configHandleRoot() {
  if (!webRequireAuth()) return;
  g_prefs.begin("nettemp", true);
  auto prefGet = [&](const String& key) -> String {
    return g_prefs.getString(key.c_str(), "");
  };
  auto prefGetBool = [&](const String& key, bool defaultVal = false) -> bool {
    return g_prefs.getBool(key.c_str(), defaultVal);
  };
  auto prefGetName = [&](const String& key, const String& fallback) -> String {
    const String v = prefGet(key);
    return v.length() ? v : fallback;
  };
  auto prefGetDsName = [&](const String& romHex, const String& fallback) -> String {
    String v = prefGet(prefKeyDsName(romHex));
    if (!v.length()) v = prefGet(prefKeyDsNameLegacy(romHex));
    return v.length() ? v : fallback;
  };
  auto sortByNameThenId = [&](const String& nameA, const String& idA, const String& nameB, const String& idB) -> bool {
    if (nameA != nameB) return nameA < nameB;
    return idA < idB;
  };
  String body = "<!doctype html><html><head><meta charset='utf-8'/>"
                "<meta name='viewport' content='width=device-width,initial-scale=1'/>"
                "<title>nettemp.pl</title>"
                "<style>"
                "*,*:before,*:after{box-sizing:border-box}"
                ":root{--bg:#0b1220;--card:#111b2e;--muted:#9fb0cc;--text:#e8eefc;--accent:#2f6bff;--border:#243251}"
                "body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial;margin:0;background:var(--bg);color:var(--text)}"
                ".wrap{max-width:980px;margin:0 auto;padding:16px}"
                ".top{display:flex;align-items:center;gap:10px;margin:6px 0 14px}"
                ".logo{width:34px;height:34px;border-radius:10px;background:var(--accent);display:flex;align-items:center;justify-content:center;font-weight:700}"
                "h2{margin:0;font-size:20px} h3{margin:0 0 10px}"
                "p{color:var(--muted);line-height:1.35}"
                ".tabs{display:flex;gap:8px;flex-wrap:wrap;margin:0 0 12px}"
                ".tabbtn{background:#0d1628;border:1px solid var(--border);color:var(--text);padding:8px 10px;border-radius:10px;font-weight:600}"
                ".tabbtn.active{background:var(--accent);border-color:var(--accent)}"
                ".card{background:var(--card);border:1px solid var(--border);border-radius:14px;padding:14px;margin:0 0 12px}"
                "label{display:block;margin:10px 0 6px;color:var(--muted);font-size:13px}"
                "input{max-width:100%;padding:8px 10px;border-radius:10px;border:1px solid var(--border);background:#0d1628;color:var(--text)}"
                "input[type=checkbox]{width:auto}"
                ".row{display:flex;gap:10px} .row>div{flex:1}"
                "button{background:var(--accent);border:none;color:white;padding:10px 14px;border-radius:10px;font-weight:600}"
                ".tablewrap{overflow-x:auto;-webkit-overflow-scrolling:touch}"
                "table{border-collapse:collapse;width:100%;margin:8px 0;border:1px solid var(--border)}"
                "td,th{border-top:1px solid var(--border);padding:8px;font-size:14px;white-space:nowrap}"
                "th{background:#0d1628;text-align:left;color:var(--muted)}"
                "code{background:#0d1628;border:1px solid var(--border);padding:2px 6px;border-radius:8px;color:var(--text);word-break:break-all;white-space:normal}"
                ".romcode{font-size:10px}"
                "ul{margin:8px 0 0 18px;color:var(--muted)}"
                "select{max-width:100%;padding:8px 10px;border-radius:10px;border:1px solid var(--border);background:#0d1628;color:var(--text)}"
                ".grid2{display:grid;grid-template-columns:1fr 1fr;gap:10px}"
                ".tile-grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}"
                ".tile{background:#0d1628;border:1px solid var(--border);border-radius:12px;padding:12px;margin:0}"
                ".tile.full{grid-column:1 / -1}"
                "@media(max-width:720px){.grid2{grid-template-columns:1fr}}"
                "@media(max-width:720px){.tile-grid{grid-template-columns:1fr}}"
                ".hidden{display:none}"
                "</style>"
                "<script>"
                "function showTab(id){"
                "document.querySelectorAll('.tab').forEach(el=>el.classList.add('hidden'));"
                "document.querySelectorAll('.tabbtn').forEach(el=>el.classList.remove('active'));"
                "document.getElementById(id).classList.remove('hidden');"
                "document.getElementById('btn-'+id).classList.add('active');"
                "localStorage.setItem('nettemp_tab', id);"
                "}"
                "function otaOnSubmit(ev){"
                "ev.preventDefault();"
                "const form=ev.target;"
                "const fileInput=form.querySelector('input[type=file]');"
                "const status=document.getElementById('ota-status');"
                "const progress=document.getElementById('ota-progress');"
                "const btn=document.getElementById('ota-submit');"
                "if(!fileInput||!fileInput.files||!fileInput.files.length){"
                "if(status) status.textContent='Select a .bin file first.';"
                "return;"
                "}"
                "const maxBytes=parseInt(fileInput.dataset.max||'0',10);"
                "if(maxBytes>0&&fileInput.files[0].size>maxBytes){"
                "if(status) status.textContent='Partition too small for upload.';"
                "return;"
                "}"
                "if(btn) btn.disabled=true;"
                "if(status) status.textContent='Uploading firmware...';"
                "if(progress) progress.textContent='0%';"
                "const xhr=new XMLHttpRequest();"
                "xhr.open('POST','/ota',true);"
                "xhr.upload.onprogress=function(e){"
                "if(!e.lengthComputable) return;"
                "const pct=Math.round((e.loaded/e.total)*100);"
                "if(progress) progress.textContent=pct+'%';"
                "};"
                "xhr.onload=function(){"
                "if(status) status.textContent=xhr.status===200?xhr.responseText:'Update failed.';"
                "if(xhr.status===200){setTimeout(()=>window.location.reload(),2000);}"
                "if(btn) btn.disabled=false;"
                "};"
                "xhr.onerror=function(){"
                "if(status) status.textContent='Upload error.';"
                "if(btn) btn.disabled=false;"
                "};"
                "const data=new FormData();"
                "data.append('firmware',fileInput.files[0]);"
                "xhr.send(data);"
                "}"
                "window.addEventListener('load',()=>{"
                "const id=localStorage.getItem('nettemp_tab')||'tab-status';"
                "if(document.getElementById(id)) showTab(id);"
                "});"
                "</script>"
                "</head><body><div class='wrap'>";

  body += "<div class='top'><div class='logo'>nt</div><div>";
  body += "<h2><a href='https://nettemp.pl' target='_blank' rel='noopener' style='color:inherit;text-decoration:none'>nettemp</a></h2>";
  const String deviceId = g_cfg.deviceId.length() ? g_cfg.deviceId : String("nettemp_esp32");
  body += "<p>Device IP: <code>" + WiFi.localIP().toString() + "</code> | Device ID: <code>" + deviceId + "</code></p>";
  body += "</div></div>";

  body += "<div class='tabs'>"
          "<button type='button' class='tabbtn' id='btn-tab-status' onclick=\"showTab('tab-status')\">Status</button>"
          "<button type='button' class='tabbtn' id='btn-tab-ble' onclick=\"showTab('tab-ble')\">BLE</button>"
          "<button type='button' class='tabbtn' id='btn-tab-i2c' onclick=\"showTab('tab-i2c')\">I2C</button>"
          "<button type='button' class='tabbtn' id='btn-tab-gpio' onclick=\"showTab('tab-gpio')\">GPIO</button>"
          "<button type='button' class='tabbtn' id='btn-tab-oled' onclick=\"showTab('tab-oled')\">OLED</button>"
          "<button type='button' class='tabbtn' id='btn-tab-channels' onclick=\"showTab('tab-channels')\">Channels</button>"
          "<button type='button' class='tabbtn' id='btn-tab-device' onclick=\"showTab('tab-device')\">Device</button>"
          "<button type='button' class='tabbtn' id='btn-tab-admin' onclick=\"showTab('tab-admin')\">Admin</button>"
          "<button type='button' class='tabbtn' id='btn-tab-system' onclick=\"showTab('tab-system')\">System</button>"
          "</div>";

  body += "<div class='card tab' id='tab-status'>";
  body += "<h3>Status</h3>";
  {
    const bool showTempc = (g_srvBleFields & SRV_BLE_TEMPC)
#if NETTEMP_ENABLE_I2C
      || (g_srvI2cFields & SRV_I2C_TEMPC)
#endif
      ;
    const bool showTempf = (g_srvBleFields & SRV_BLE_TEMPF);
    const bool showHum = (g_srvBleFields & SRV_BLE_HUM)
#if NETTEMP_ENABLE_I2C
      || (g_srvI2cFields & SRV_I2C_HUM)
#endif
      ;
    const bool showVolt = (g_srvBleFields & SRV_BLE_VOLT);
    const bool showBatt = (g_srvBleFields & SRV_BLE_BATT);
    const bool showRssi = (g_srvBleFields & SRV_BLE_RSSI);
#if NETTEMP_ENABLE_I2C
    const bool showPress = (g_srvI2cFields & SRV_I2C_PRESS);
#else
    const bool showPress = false;
#endif
    const bool showSoilRaw = g_cfg.soilEnabled;
    const bool showSoilPct = g_cfg.soilEnabled;
    const bool showHcDist = g_cfg.hcsr04Enabled;
    const uint32_t nowMs = millis();
    body += "<h4 style='margin:8px 0 6px'>Sensors</h4>";

    bool anyStatusTable = false;
    {
      const auto idx = buildBleSortedIndex();
      bool anyBle = false;
      for (size_t k = 0; k < idx.size(); k++) {
        const auto& s = g_sensors[idx[k]];
        if (!s.selected) continue;
        const bool hasValues = !isnan(s.temperatureC) || !isnan(s.humidityPct) || s.batteryPct >= 0 || s.voltageMv >= 0 || s.rssi != 0;
        if (!hasValues) continue;
        anyBle = true;
        break;
      }

      if (anyBle) {
        body += "<div class='tablewrap'><table><tr><th>Name</th><th>BLE</th>";
        if (showTempc) body += "<th>tempc</th>";
        if (showTempf) body += "<th>tempf</th>";
        if (showHum) body += "<th>hum</th>";
        if (showVolt) body += "<th>volt</th>";
        if (showBatt) body += "<th>batt</th>";
        if (showRssi) body += "<th>rssi</th>";
        body += "</tr>";

        std::vector<size_t> bleIdx;
        bleIdx.reserve(idx.size());
        for (size_t k = 0; k < idx.size(); k++) {
          const auto& s = g_sensors[idx[k]];
          if (!s.selected) continue;
          const bool hasValues = !isnan(s.temperatureC) || !isnan(s.humidityPct) || s.batteryPct >= 0 || s.voltageMv >= 0 || s.rssi != 0;
          if (!hasValues) continue;
          bleIdx.push_back(idx[k]);
        }
        std::sort(bleIdx.begin(), bleIdx.end(), [&](size_t a, size_t b) {
          const auto& sa = g_sensors[a];
          const auto& sb = g_sensors[b];
          const String macNoA = macNoColonsUpper(sa.mac);
          const String macNoB = macNoColonsUpper(sb.mac);
          const String nameA = prefGetName(prefKeyBleName(macNoA), "");
          const String nameB = prefGetName(prefKeyBleName(macNoB), "");
          return sortByNameThenId(nameA, macNoA, nameB, macNoB);
        });

        for (const size_t k : bleIdx) {
          const auto& s = g_sensors[k];
          body += "<tr>";
          const String macNo = macNoColonsUpper(s.mac);
          const String nameVal = prefGetName(prefKeyBleName(macNo), "");
          body += "<td>" + htmlEscape(nameVal) + "</td>";
          body += "<td><code>" + macWithColonsUpper(s.mac) + "</code></td>";
          if (showTempc) body += "<td>" + (isnan(s.temperatureC) ? String("-") : String(s.temperatureC, 2)) + "</td>";
          if (showTempf) body += "<td>" + (isnan(s.temperatureC) ? String("-") : String((s.temperatureC * 9.0f / 5.0f) + 32.0f, 2)) + "</td>";
          if (showHum) body += "<td>" + (isnan(s.humidityPct) ? String("-") : String(s.humidityPct, 1)) + "</td>";
          if (showVolt) body += "<td>" + (s.voltageMv < 0 ? String("-") : String((float)s.voltageMv / 1000.0f, 3)) + "</td>";
          if (showBatt) body += "<td>" + (s.batteryPct < 0 ? String("-") : String(s.batteryPct)) + "</td>";
          if (showRssi) body += "<td>" + (s.rssi == 0 ? String("-") : String(s.rssi)) + "</td>";
          body += "</tr>";
        }
        body += "</table></div>";
        anyStatusTable = true;
      }
    }

    {
#if NETTEMP_ENABLE_I2C
      bool anyI2c = false;
      for (const auto& s : g_i2cSensors) {
        if (!s.selected) continue;
        if (!s.reading.ok) continue;
        const bool hasValues = !isnan(s.reading.temperature_c) || !isnan(s.reading.humidity_pct) || !isnan(s.reading.pressure_hpa);
        if (!hasValues) continue;
        anyI2c = true;
        break;
      }

      if (anyI2c) {
        body += "<div class='tablewrap' style='margin-top:8px'><table><tr><th>Name</th><th>I2C</th>";
        if (showTempc) body += "<th>tempc</th>";
        if (showTempf) body += "<th>tempf</th>";
        if (showHum) body += "<th>hum</th>";
        if (showPress) body += "<th>press_hpa</th>";
        body += "</tr>";

        std::vector<size_t> i2cIdx;
        i2cIdx.reserve(g_i2cSensors.size());
        for (size_t i = 0; i < g_i2cSensors.size(); i++) {
          const auto& s = g_i2cSensors[i];
          if (!s.selected) continue;
          if (!s.reading.ok) continue;
          const bool hasValues = !isnan(s.reading.temperature_c) || !isnan(s.reading.humidity_pct) || !isnan(s.reading.pressure_hpa);
          if (!hasValues) continue;
          i2cIdx.push_back(i);
        }
        std::sort(i2cIdx.begin(), i2cIdx.end(), [&](size_t a, size_t b) {
          const auto& sa = g_i2cSensors[a];
          const auto& sb = g_i2cSensors[b];
          String addrA = String(sa.address, HEX);
          String addrB = String(sb.address, HEX);
          addrA.toLowerCase();
          addrB.toLowerCase();
          if (addrA.length() == 1) addrA = "0" + addrA;
          if (addrB.length() == 1) addrB = "0" + addrB;
          const String nameA = prefGetName(
            prefKeyI2cName(addrA),
            prefGetName(String("i2cname_") + addrA, String(i2cSensorTypeName(sa.type)) + " 0x" + addrA)
          );
          const String nameB = prefGetName(
            prefKeyI2cName(addrB),
            prefGetName(String("i2cname_") + addrB, String(i2cSensorTypeName(sb.type)) + " 0x" + addrB)
          );
          const String idA = String(i2cSensorTypeName(sa.type)) + " 0x" + addrA;
          const String idB = String(i2cSensorTypeName(sb.type)) + " 0x" + addrB;
          return sortByNameThenId(nameA, idA, nameB, idB);
        });

        for (const size_t i : i2cIdx) {
          const auto& s = g_i2cSensors[i];
          String addrHex = String(s.address, HEX);
          addrHex.toLowerCase();
          if (addrHex.length() == 1) addrHex = "0" + addrHex;
          body += "<tr>";
          const String nameVal = prefGetName(
            prefKeyI2cName(addrHex),
            prefGetName(String("i2cname_") + addrHex, String(i2cSensorTypeName(s.type)) + " 0x" + addrHex)
          );
          body += "<td>" + htmlEscape(nameVal) + "</td>";
          body += "<td><code>0x" + addrHex + " " + String(i2cSensorTypeName(s.type)) + "</code></td>";
          if (showTempc) body += "<td>" + (isnan(s.reading.temperature_c) ? String("-") : String(s.reading.temperature_c, 2)) + "</td>";
          if (showTempf) body += "<td>" + (isnan(s.reading.temperature_c) ? String("-") : String((s.reading.temperature_c * 9.0f / 5.0f) + 32.0f, 2)) + "</td>";
          if (showHum) body += "<td>" + (isnan(s.reading.humidity_pct) ? String("-") : String(s.reading.humidity_pct, 1)) + "</td>";
          if (showPress) body += "<td>" + (isnan(s.reading.pressure_hpa) ? String("-") : String(s.reading.pressure_hpa, 1)) + "</td>";
          body += "</tr>";
        }
        body += "</table></div>";
        anyStatusTable = true;
      }
#else
      body += "<p style='margin-top:8px'>I2C support is disabled in this build.</p>";
#endif
    }

    {
      const bool anyGpio = g_cfg.dsEnabled || g_cfg.dhtEnabled || g_cfg.vbatMode != 0 || g_cfg.soilEnabled || g_cfg.hcsr04Enabled;

      if (anyGpio) {
        body += "<div class='tablewrap' style='margin-top:8px'><table><tr><th>Name</th><th>GPIO</th>";
        if (showTempc) body += "<th>tempc</th>";
        if (showTempf) body += "<th>tempf</th>";
        if (showHum) body += "<th>hum</th>";
        if (showVolt) body += "<th>volt</th>";
        if (showBatt) body += "<th>batt</th>";
        if (showSoilRaw) body += "<th>soil_raw</th>";
        if (showSoilPct) body += "<th>soil_pct</th>";
        if (showHcDist) body += "<th>dist_cm</th>";
        body += "</tr>";

        if (g_cfg.dsEnabled) {
          if (g_dsRoms.empty()) {
            body += "<tr>";
            body += "<td></td>";
            body += "<td><code>DS18B20 (no devices)</code></td>";
            if (showTempc) body += "<td>-</td>";
            if (showTempf) body += "<td>-</td>";
            if (showHum) body += "<td>-</td>";
            if (showVolt) body += "<td>-</td>";
            if (showBatt) body += "<td>-</td>";
            if (showSoilRaw) body += "<td>-</td>";
            if (showSoilPct) body += "<td>-</td>";
            if (showHcDist) body += "<td>-</td>";
            body += "</tr>";
          } else {
            std::vector<size_t> dsIdx;
            dsIdx.reserve(g_dsRoms.size());
            for (size_t i = 0; i < g_dsRoms.size(); i++) dsIdx.push_back(i);
            std::sort(dsIdx.begin(), dsIdx.end(), [&](size_t a, size_t b) {
              char romHexA[17]{};
              char romHexB[17]{};
              for (int b0 = 0; b0 < 8; b0++) sprintf(romHexA + b0 * 2, "%02X", g_dsRoms[a][b0]);
              for (int b0 = 0; b0 < 8; b0++) sprintf(romHexB + b0 * 2, "%02X", g_dsRoms[b][b0]);
              const String idA = dsRomLinuxId(g_dsRoms[a], 0, false);
              const String idB = dsRomLinuxId(g_dsRoms[b], 0, false);
              const String nameA = prefGetDsName(romHexA, String("DS18B20 ") + idA);
              const String nameB = prefGetDsName(romHexB, String("DS18B20 ") + idB);
              return sortByNameThenId(nameA, idA, nameB, idB);
            });

            for (size_t idx : dsIdx) {
              char romHex[17]{};
              for (int b = 0; b < 8; b++) sprintf(romHex + b * 2, "%02X", g_dsRoms[idx][b]);
              body += "<tr>";
              const String id = dsRomLinuxId(g_dsRoms[idx], 0, false);
              const String nameVal = prefGetDsName(romHex, String("DS18B20 ") + id);
              body += "<td>" + htmlEscape(nameVal) + "</td>";
              body += "<td><code class='romcode'>DS18B20 " + id + "</code></td>";
              const float tempc = (idx < g_dsTempsC.size()) ? g_dsTempsC[idx] : NAN;
              if (showTempc) body += "<td>" + (isnan(tempc) ? String("-") : String(tempc, 2)) + "</td>";
              if (showTempf) body += "<td>" + (isnan(tempc) ? String("-") : String((tempc * 9.0f / 5.0f) + 32.0f, 2)) + "</td>";
              if (showHum) body += "<td>-</td>";
              if (showVolt) body += "<td>-</td>";
              if (showBatt) body += "<td>-</td>";
              if (showSoilRaw) body += "<td>-</td>";
              if (showSoilPct) body += "<td>-</td>";
              if (showHcDist) body += "<td>-</td>";
              body += "</tr>";
            }
          }
        }

        if (g_cfg.dhtEnabled) {
          body += "<tr>";
          body += "<td>" + htmlEscape(prefGetName("name_dht", "")) + "</td>";
          body += "<td><code>DHT" + String(g_cfg.dhtType) + " GPIO" + String(g_cfg.dhtPin) + "</code></td>";
          if (showTempc) body += "<td>" + (isnan(g_dhtTempC) ? String("-") : String(g_dhtTempC, 2)) + "</td>";
          if (showTempf) body += "<td>" + (isnan(g_dhtTempC) ? String("-") : String((g_dhtTempC * 9.0f / 5.0f) + 32.0f, 2)) + "</td>";
          if (showHum) body += "<td>" + (isnan(g_dhtHumPct) ? String("-") : String(g_dhtHumPct, 1)) + "</td>";
          if (showVolt) body += "<td>-</td>";
          if (showBatt) body += "<td>-</td>";
          if (showSoilRaw) body += "<td>-</td>";
          if (showSoilPct) body += "<td>-</td>";
          if (showHcDist) body += "<td>-</td>";
          body += "</tr>";
        }

        if (g_cfg.vbatMode != 0) {
          body += "<tr>";
          body += "<td>" + htmlEscape(prefGetName("name_vbat", "")) + "</td>";
          body += "<td><code>VBAT</code></td>";
          if (showTempc) body += "<td>-</td>";
          if (showTempf) body += "<td>-</td>";
          if (showHum) body += "<td>-</td>";
          if (showVolt) body += "<td>" + (isnan(g_vbatVolts) ? String("-") : String(g_vbatVolts, 2)) + "</td>";
          if (showBatt) body += "<td>" + (g_vbatPct < 0 ? String("-") : String(g_vbatPct)) + "</td>";
          if (showSoilRaw) body += "<td>-</td>";
          if (showSoilPct) body += "<td>-</td>";
          if (showHcDist) body += "<td>-</td>";
          body += "</tr>";
        }

        if (g_cfg.soilEnabled) {
          body += "<tr>";
          body += "<td>" + htmlEscape(prefGetName("name_soil", "")) + "</td>";
          body += "<td><code>SOIL ADC" + String(g_cfg.soilAdcPin) + "</code></td>";
          if (showTempc) body += "<td>-</td>";
          if (showTempf) body += "<td>-</td>";
          if (showHum) body += "<td>-</td>";
          if (showVolt) body += "<td>-</td>";
          if (showBatt) body += "<td>-</td>";
          if (showSoilRaw) body += "<td>" + (g_soilRaw < 0 ? String("-") : String(g_soilRaw)) + "</td>";
          if (showSoilPct) body += "<td>" + (isnan(g_soilPct) ? String("-") : String(g_soilPct, 1)) + "</td>";
          if (showHcDist) body += "<td>-</td>";
          body += "</tr>";
        }

        if (g_cfg.hcsr04Enabled) {
          body += "<tr>";
          body += "<td>" + htmlEscape(prefGetName("name_hcsr04", "")) + "</td>";
          body += "<td><code>HC-SR04 GPIO" + String(g_cfg.hcsr04TrigPin) + "/" + String(g_cfg.hcsr04EchoPin) + "</code></td>";
          if (showTempc) body += "<td>-</td>";
          if (showTempf) body += "<td>-</td>";
          if (showHum) body += "<td>-</td>";
          if (showVolt) body += "<td>-</td>";
          if (showBatt) body += "<td>-</td>";
          if (showSoilRaw) body += "<td>-</td>";
          if (showSoilPct) body += "<td>-</td>";
          if (showHcDist) body += "<td>" + (isnan(g_hcsr04Cm) ? String("-") : String(g_hcsr04Cm, 1)) + "</td>";
          body += "</tr>";
        }

        body += "</table></div>";
        anyStatusTable = true;
      }
    }

    if (!anyStatusTable) {
      body += "<p>Enable sensors to show data.</p>";
    }
  }

  body += "</div>";

  body += "<div class='card tab hidden' id='tab-ble'>";
  body += "<h3>BLE</h3>";
  body += "<form method='POST' action='/save_ble'>";
#if NETTEMP_HEADLESS
  body += "<label><input type='checkbox' name='bleAuto' value='1' " + String(g_headlessBleAutoScan ? "checked" : "") + "> Autoscan</label>";
#else
  body += "<p>BLE scanning is always on in UI mode (Cardputer).</p>";
#endif
  body += "<p style='margin:12px 0 0'><button type='submit'>Save</button></p>";
  body += "</form>";

  body += "<h3 style='margin-top:16px'>BLE devices</h3>";
  body += "<p>Manual overrides persist across reboot (use <code>Manual</code> checkbox).</p>";
  body += "<form method='POST' action='/save_sensors'>";
  body += "<div class='tablewrap'><table><tr><th>Send</th><th>Manual</th><th>MAC</th><th>Name</th><th>Age(s)</th><th>tempc</th><th>hum</th><th>batt</th></tr>";
  const uint32_t nowMs = millis();
  const auto idx = buildBleSortedIndex();
  std::vector<size_t> bleCfgIdx;
  bleCfgIdx.reserve(idx.size());
  for (size_t k = 0; k < idx.size(); k++) {
    const auto& s = g_sensors[idx[k]];
    const bool hasValues = !isnan(s.temperatureC) || !isnan(s.humidityPct) || s.batteryPct >= 0;
    if (!hasValues) continue;
    bleCfgIdx.push_back(idx[k]);
  }
  std::sort(bleCfgIdx.begin(), bleCfgIdx.end(), [&](size_t a, size_t b) {
    const auto& sa = g_sensors[a];
    const auto& sb = g_sensors[b];
    const String macNoA = macNoColonsUpper(sa.mac);
    const String macNoB = macNoColonsUpper(sb.mac);
    const String nameA = prefGetName(prefKeyBleName(macNoA), "");
    const String nameB = prefGetName(prefKeyBleName(macNoB), "");
    return sortByNameThenId(nameA, macNoA, nameB, macNoB);
  });

  for (const size_t k : bleCfgIdx) {
    const auto& s = g_sensors[k];
    const String mac = macWithColonsUpper(s.mac);
    const String macNo = macNoColonsUpper(mac);
    bool manualSelected = false;
    const bool manual = bleManualGetSelected(macNo, manualSelected);
    const bool selected = manual ? manualSelected : s.selected;
    const uint32_t ageS = (s.lastSeenMs > 0 && nowMs >= s.lastSeenMs) ? ((nowMs - s.lastSeenMs) / 1000UL) : 0;

    body += "<tr>";
    body += "<td><input type='checkbox' name='sel_" + macNo + "' value='1' " + String(selected ? "checked" : "") + "></td>";
    body += "<td><input type='checkbox' name='man_" + macNo + "' value='1' " + String(manual ? "checked" : "") + "></td>";
    const String nameVal = prefGetName(prefKeyBleName(macNo), "");
    body += "<td><code>" + mac + "</code></td>";
    body += "<td><input name='bleName_" + macNo + "' value='" + htmlEscape(nameVal) + "' placeholder='name'/></td>";
    body += "<td>" + String((unsigned long)ageS) + "</td>";
    body += "<td>" + (isnan(s.temperatureC) ? String("-") : String(s.temperatureC, 2)) + "</td>";
    body += "<td>" + (isnan(s.humidityPct) ? String("-") : String(s.humidityPct, 1)) + "</td>";
    body += "<td>" + (s.batteryPct < 0 ? String("-") : String(s.batteryPct)) + "</td>";
    body += "</tr>";
  }
  body += "</table></div>";
  body += "<p style='margin:12px 0 0'><button type='submit'>Apply</button></p>";
  body += "</form>";
  body += "</div>";

  body += "<div class='card tab hidden' id='tab-i2c'>";
  body += "<h3>I2C</h3>";
#if NETTEMP_ENABLE_I2C
  body += "<datalist id='i2cPins'>"
          "<option value='-1'></option>"
          "<option value='21'></option><option value='22'></option>"
          "<option value='18'></option><option value='19'></option>"
          "<option value='4'></option><option value='5'></option>"
          "<option value='8'></option><option value='9'></option>"
          "</datalist>";
  body += "<form method='POST' action='/i2c_pins' style='margin-top:12px'>";
  body += "<div class='grid2'><div><label>SDA GPIO (-1 = default)</label><input name='i2cSda' list='i2cPins' value='" + String(g_cfg.i2cSdaPin) + "'/></div>";
  body += "<div><label>SCL GPIO (-1 = default)</label><input name='i2cScl' list='i2cPins' value='" + String(g_cfg.i2cSclPin) + "'/></div></div>";
  body += "<p style='margin:12px 0 0'><button type='submit'>Apply pins + Rescan</button></p>";
  body += "</form>";

  std::vector<uint8_t> filteredAddrs;
  filteredAddrs.reserve(g_i2cDetectedAddrs.size());
  for (const auto addr : g_i2cDetectedAddrs) {
    bool isSupported = false;
    for (const auto& s : g_i2cSensors) {
      if (s.address == addr) {
        isSupported = true;
        break;
      }
    }
    if (isSupported) filteredAddrs.push_back(addr);
  }
  const size_t i2cCount = filteredAddrs.size();
  body += "<p style='margin-top:12px'>Detected I2C devices: <code>" + String((unsigned long)i2cCount) + "</code></p>";

  // Debug: Show all I2C addresses detected
  if (!g_i2cDetectedAddrs.empty()) {
    body += "<details><summary style='cursor:pointer;color:var(--muted);font-size:0.9em'>🔍 Debug: I2C scan details</summary>";
    body += "<div style='background:var(--card);border:1px solid var(--border);padding:8px;margin:8px 0;font-family:monospace;font-size:0.85em'>";

    // Bus health check
    const size_t addrCount = g_i2cDetectedAddrs.size();
    if (addrCount > 10) {
      body += "<div style='background:#2a1f0d;border-left:4px solid #6b4b1a;padding:8px;margin-bottom:8px'>";
      body += "⚠️ <b>BUS FAULT DETECTED!</b><br>";
      body += String((unsigned)addrCount) + " devices detected - this indicates:<br>";
      body += "• <b>Missing I2C pull-up resistors</b> (most likely)<br>";
      body += "• Floating SDA/SCL lines<br>";
      body += "• Wrong GPIO pins configured<br>";
      body += "<br><b>FIX:</b> Add 4.7kΩ pull-ups from SDA→3.3V and SCL→3.3V<br>";
      body += "Current pins: SDA=" + String(g_cfg.i2cSdaPin) + ", SCL=" + String(g_cfg.i2cSclPin);
      body += "</div>";
    }

    body += "<b>All I2C devices on bus:</b> (" + String((unsigned)addrCount) + " total)<br>";
    for (const auto addr : g_i2cDetectedAddrs) {
      body += "• 0x" + String(addr < 16 ? "0" : "") + String(addr, HEX) + "<br>";
    }
    body += "</div></details>";
  }

  if (filteredAddrs.empty()) {
    body += "<p>No I2C devices found.</p>";
  } else {
    const uint32_t nowMs = millis();
    body += "<form method='POST' action='/save_i2c'>";
    body += "<div class='tablewrap'><table><tr><th>Send</th><th>Name</th><th>Type</th><th>Addr</th><th>Age(s)</th><th>tempc</th><th>hum</th><th>press_hpa</th></tr>";
    std::vector<uint8_t> sortedAddrs = filteredAddrs;
    std::sort(sortedAddrs.begin(), sortedAddrs.end(), [&](uint8_t a, uint8_t b) {
      const I2cSensorInfo* sa = nullptr;
      const I2cSensorInfo* sb = nullptr;
      for (const auto& s : g_i2cSensors) {
        if (s.address == a) sa = &s;
        if (s.address == b) sb = &s;
      }
      const char* typeA = sa ? i2cSensorTypeName(sa->type) : "UNKNOWN";
      const char* typeB = sb ? i2cSensorTypeName(sb->type) : "UNKNOWN";
      String addrA = String(a, HEX);
      String addrB = String(b, HEX);
      addrA.toLowerCase();
      addrB.toLowerCase();
      if (addrA.length() == 1) addrA = "0" + addrA;
      if (addrB.length() == 1) addrB = "0" + addrB;
      const String nameA = prefGetName(prefKeyI2cName(addrA), prefGetName(String("i2cname_") + addrA, String(typeA) + " 0x" + addrA));
      const String nameB = prefGetName(prefKeyI2cName(addrB), prefGetName(String("i2cname_") + addrB, String(typeB) + " 0x" + addrB));
      const String idA = String(typeA) + " 0x" + addrA;
      const String idB = String(typeB) + " 0x" + addrB;
      return sortByNameThenId(nameA, idA, nameB, idB);
    });

    for (const auto addr : sortedAddrs) {
      const I2cSensorInfo* match = nullptr;
      for (const auto& s : g_i2cSensors) {
        if (s.address == addr) {
          match = &s;
          break;
        }
      }
      const char* typeName = match ? i2cSensorTypeName(match->type) : "UNKNOWN";
      char addrHexBuf[5]{};
      snprintf(addrHexBuf, sizeof(addrHexBuf), "0x%02X", (unsigned)addr);
      const String addrHex = (addr < 16 ? String("0") : String("")) + String(addr, HEX);
      const String key = "i2c_" + addrHex;
      const uint32_t ageS = (match && match->last_seen_ms > 0 && nowMs >= match->last_seen_ms) ? ((nowMs - match->last_seen_ms) / 1000UL) : 0;
      body += "<tr>";
      if (match) {
        body += "<td><input type='checkbox' name='" + key + "' value='1' " + String(match->selected ? "checked" : "") + "></td>";
      } else {
        body += "<td>-</td>";
      }
      if (match) {
        const String nameVal = prefGetName(prefKeyI2cName(addrHex), prefGetName(String("i2cname_") + addrHex, ""));
        body += "<td><input name='i2cName_" + addrHex + "' value='" + htmlEscape(nameVal) + "' placeholder='" + htmlEscape(String(typeName) + " 0x" + addrHex) + "'/></td>";
      } else {
        body += "<td>-</td>";
      }
      body += "<td><code>" + String(typeName) + "</code></td>";
      body += "<td><code>" + String(addrHexBuf) + "</code></td>";
      body += "<td>" + (match ? String((unsigned long)ageS) : String("-")) + "</td>";
      body += "<td>" + (match && match->reading.ok && !isnan(match->reading.temperature_c) ? String(match->reading.temperature_c, 2) : String("-")) + "</td>";
      body += "<td>" + (match && match->reading.ok && !isnan(match->reading.humidity_pct) ? String(match->reading.humidity_pct, 1) : String("-")) + "</td>";
      body += "<td>" + (match && match->reading.ok && !isnan(match->reading.pressure_hpa) ? String(match->reading.pressure_hpa, 1) : String("-")) + "</td>";
      body += "</tr>";
    }
    body += "</table></div>";
    body += "<p style='margin:12px 0 0'><button type='submit'>Save</button></p>";
    body += "</form>";
  }
  body += "<p style='margin-top:16px'>Supported drivers:</p>";
  body += "<ul>"
          "<li><code>BMP280</code> (0x76/0x77)</li>"
          "<li><code>BME280</code> (0x76/0x77)</li>"
          "<li><code>TMP102</code> (0x48-0x4B)</li>"
          "<li><code>SHT3X</code> (0x44/0x45)</li>"
          "<li><code>HTU21D/SHT21/SI7021</code> (0x40)</li>"
          "</ul>";
#else
  body += "<p>I2C support is disabled in this build (<code>NETTEMP_ENABLE_I2C=0</code>).</p>";
#endif
  body += "</div>";

  body += "<div class='card tab hidden' id='tab-gpio'>";
  body += "<h3>GPIO sensors</h3>";
  body += "<div class='tile-grid'>";
  body += "<div class='tile full'>";
  body += "<h4 style='margin:0 0 8px'>DS18B20</h4>";
  body += "<form method='POST' action='/save_gpio'>";
  body += "<input type='hidden' name='sensor' value='ds18b20'/>";
  body += "<label><input type='checkbox' name='dsEn' value='1' " + String(g_cfg.dsEnabled ? "checked" : "") + "> Enable DS18B20 (1-Wire)</label>";
  body += "<label>DS18B20 GPIO</label><input name='dsPin' value='" + String(g_cfg.dsPin) + "'/>";
  body += "<p>Detected DS18B20: <code>" + String((unsigned long)g_dsRoms.size()) + "</code> ";
  body += "<button type='submit' name='dsScan' value='1' style='padding:2px 8px;font-size:0.9em'>Scan Now</button> ";
  body += "<small style='color:var(--muted)'>(Scans without saving)</small></p>";
  if (!g_dsRoms.empty()) {
    Serial.printf("Web: Building GPIO form with %u DS18B20 sensors\n", (unsigned)g_dsRoms.size());
    body += "<div class='tablewrap'><table><tr><th>Send</th><th>Name</th><th>#</th><th>ROM</th><th>tempc</th></tr>";
    std::vector<size_t> dsIdx;
    dsIdx.reserve(g_dsRoms.size());
    for (size_t i = 0; i < g_dsRoms.size(); i++) dsIdx.push_back(i);
    std::sort(dsIdx.begin(), dsIdx.end(), [&](size_t a, size_t b) {
      char romHexA[17]{};
      char romHexB[17]{};
      for (int b0 = 0; b0 < 8; b0++) sprintf(romHexA + b0 * 2, "%02X", g_dsRoms[a][b0]);
      for (int b0 = 0; b0 < 8; b0++) sprintf(romHexB + b0 * 2, "%02X", g_dsRoms[b][b0]);
      const String idA = dsRomLinuxId(g_dsRoms[a], 0, false);
      const String idB = dsRomLinuxId(g_dsRoms[b], 0, false);
      const String nameA = prefGetDsName(romHexA, String("DS18B20 ") + idA);
      const String nameB = prefGetDsName(romHexB, String("DS18B20 ") + idB);
      return sortByNameThenId(nameA, idA, nameB, idB);
    });

    for (size_t idx : dsIdx) {
      char romHex[17]{};
      for (int b = 0; b < 8; b++) sprintf(romHex + b * 2, "%02X", g_dsRoms[idx][b]);
      const String nameKey = prefKeyDsName(romHex);
      const String nameVal = prefGetDsName(romHex, "");
      const String linuxId = dsRomLinuxId(g_dsRoms[idx], 0, false);
      const String selKey = String("dsSel_") + romHex;
      const bool selected = prefGetBool(selKey, true); // Default: selected
      Serial.printf("  [%u] ROM=%s, nameKey=%s, name='%s', selKey=%s, sel=%s\n", (unsigned)idx, romHex, nameKey.c_str(), nameVal.c_str(), selKey.c_str(), selected ? "YES" : "NO");
      body += "<tr>";
      body += "<td><input type='checkbox' name='" + selKey + "' value='1' " + String(selected ? "checked" : "") + "></td>";
      body += "<td><input name='dsName_" + String(romHex) + "' value='" + htmlEscape(nameVal) + "' placeholder='" + htmlEscape(String("DS18B20 ") + linuxId) + "'/></td>";
      body += "<td>" + String((unsigned long)idx) + "</td><td><code class='romcode'>" + linuxId + "</code></td><td>";
      body += (idx < g_dsTempsC.size() && !isnan(g_dsTempsC[idx]) ? String(g_dsTempsC[idx], 2) : String("-"));
      body += "</td></tr>";
    }
    body += "</table></div>";
  }
  body += "<p style='margin:12px 0 0'><button type='submit'>Save</button></p>";
  body += "</form>";
  body += "</div>";
  body += "<div class='tile'>";
  body += "<h4 style='margin:0 0 8px'>DHT</h4>";
  body += "<form method='POST' action='/save_gpio'>";
  body += "<input type='hidden' name='sensor' value='dht'/>";
  body += "<label><input type='checkbox' name='dhtEn' value='1' " + String(g_cfg.dhtEnabled ? "checked" : "") + "> Enable DHT</label>";
  body += "<label>Name</label><input name='name_dht' value='" + htmlEscape(prefGetName("name_dht", "")) + "' placeholder='name'/>";
  body += "<div class='grid2'><div><label>DHT GPIO</label><input name='dhtPin' value='" + String(g_cfg.dhtPin) + "'/></div>";
  body += "<div><label>DHT type</label><input name='dhtType' value='" + String(g_cfg.dhtType) + "' placeholder='11 or 22'/></div></div>";
  body += "<p>Last DHT: tempc=<code>" + (isnan(g_dhtTempC) ? String("-") : String(g_dhtTempC, 2)) + "</code> hum=<code>" +
          (isnan(g_dhtHumPct) ? String("-") : String(g_dhtHumPct, 1)) + "</code></p>";
  body += "<p style='margin:12px 0 0'><button type='submit'>Save</button></p>";
  body += "</form>";
  body += "</div>";
  body += "<div class='tile'>";
  body += "<h4 style='margin:0 0 8px'>Soil</h4>";
  body += "<form method='POST' action='/save_gpio'>";
  body += "<input type='hidden' name='sensor' value='soil'/>";
  body += "<label><input type='checkbox' name='soilEn' value='1' " + String(g_cfg.soilEnabled ? "checked" : "") + "> Enable capacitive soil moisture sensor v1.2 (ADC)</label>";
  body += "<label>Name</label><input name='name_soil' value='" + htmlEscape(prefGetName("name_soil", "")) + "' placeholder='name'/>";
  body += "<label>Soil ADC pin</label><input name='soilPin' value='" + String(g_cfg.soilAdcPin) + "'/>";
  body += "<div class='grid2'><div><label>Dry raw (0%)</label><input name='soilDry' value='" + String(g_cfg.soilDryRaw) + "'/></div>";
  body += "<div><label>Wet raw (100%)</label><input name='soilWet' value='" + String(g_cfg.soilWetRaw) + "'/></div></div>";
  body += "<p>Last soil: raw=<code>" + (g_soilRaw < 0 ? String("-") : String(g_soilRaw)) + "</code> pct=<code>" +
          (isnan(g_soilPct) ? String("-") : String(g_soilPct, 1)) + "</code></p>";
  body += "<p style='margin:12px 0 0'><button type='submit'>Save</button></p>";
  body += "</form>";
  body += "</div>";
  body += "<div class='tile'>";
  body += "<h4 style='margin:0 0 8px'>HC-SR04</h4>";
  body += "<form method='POST' action='/save_gpio'>";
  body += "<input type='hidden' name='sensor' value='hcsr04'/>";
  body += "<label><input type='checkbox' name='hcEn' value='1' " + String(g_cfg.hcsr04Enabled ? "checked" : "") + "> Enable HC-SR04 distance sensor</label>";
  body += "<label>Name</label><input name='name_hcsr04' value='" + htmlEscape(prefGetName("name_hcsr04", "")) + "' placeholder='name'/>";
  body += "<div class='grid2'><div><label>Trig GPIO</label><input name='hcTrig' value='" + String(g_cfg.hcsr04TrigPin) + "'/></div>";
  body += "<div><label>Echo GPIO</label><input name='hcEcho' value='" + String(g_cfg.hcsr04EchoPin) + "'/></div></div>";
  body += "<p>Last HC-SR04: cm=<code>" + (isnan(g_hcsr04Cm) ? String("-") : String(g_hcsr04Cm, 1)) + "</code></p>";
  body += "<p style='margin:12px 0 0'><button type='submit'>Save</button></p>";
  body += "</form>";
  body += "</div>";
  body += "<div class='tile'>";
  body += "<h4 style='margin:0 0 8px'>Battery (VBAT)</h4>";
  body += "<form method='POST' action='/save_gpio'>";
  body += "<input type='hidden' name='sensor' value='vbat'/>";
  body += "<label><input type='checkbox' name='vbEn' value='1' " + String(g_cfg.vbatMode != 0 ? "checked" : "") + "> Enable VBAT</label>";
  body += "<label>Name</label><input name='name_vbat' value='" + htmlEscape(prefGetName("name_vbat", "")) + "' placeholder='name'/>";
  body += "<label>Mode</label><select name='vbMode'>"
          "<option value='1' " + String(g_cfg.vbatMode == 1 ? "selected" : "") + ">adc</option>"
          "<option value='2' " + String(g_cfg.vbatMode == 2 ? "selected" : "") + ">pmic</option>"
          "</select>";
  body += "<label>ADC pin (adc mode)</label><input name='vbPin' value='" + String(g_cfg.vbatAdcPin) + "'/>";
  body += "<label>Divider ratio (x1000)</label><input name='vbDiv' value='" + String((unsigned)g_cfg.vbatDividerX1000) + "' placeholder='2000 = 2.0'/>";
  body += "<p class='muted' style='margin-top:-8px'>Tip: VBAT = ADC × ratio. Current ratio: <code>" +
          String(((float)g_cfg.vbatDividerX1000) / 1000.0f, 3) +
          "</code>. Typical range: <code>1000</code>–<code>5000</code>.</p>";
  body += "<div class='grid2'><div><label>Full (mV)</label><input name='vbFull' value='" + String((unsigned)g_cfg.vbatFullMv) + "'/></div>";
  body += "<div><label>Empty (mV)</label><input name='vbEmp' value='" + String((unsigned)g_cfg.vbatEmptyMv) + "'/></div></div>";
  body += "<label><input type='checkbox' name='vbSV' value='1' " + String(g_cfg.vbatSendVolt ? "checked" : "") + "> Send voltage (vbat) in addition to percent</label>";
  body += "<p>Last VBAT: ";
  if (g_cfg.vbatMode == 0) {
    body += "<code>off</code>";
  } else if (g_vbatLastSeenMs == 0) {
    body += "<code>not measured yet</code>";
  } else {
    const uint32_t nowMs = millis();
    const uint32_t ageS = (nowMs >= g_vbatLastSeenMs) ? ((nowMs - g_vbatLastSeenMs) / 1000UL) : 0;
    bool first = true;
    if (!isnan(g_vbatVolts)) {
      body += "v=<code>" + String(g_vbatVolts, 3) + "</code>";
      first = false;
    }
    if (g_vbatPct >= 0) {
      body += String(first ? "" : " ") + "batt=<code>" + String(g_vbatPct) + "%</code>";
      first = false;
    }
    body += String(first ? "" : " ") + "age=<code>" + String((unsigned long)ageS) + "s</code>";
  }
  body += "</p>";
  if (g_cfg.vbatMode != 0 && g_vbatLastSeenMs == 0) {
    body += "<p>Tip: if this stays empty, verify <code>Mode</code> and <code>ADC pin</code>. "
            "On ESP32-S3, GPIO36 does not exist; on Cardputer prefer <code>pmic</code>. "
            "If your board has a built-in divider that already scales ADC to VBAT, try <code>Divider ratio</code> = <code>1000</code>.</p>";
  }
  body += "<p style='margin:12px 0 0'><button type='submit'>Save</button></p>";
  body += "</form>";
  body += "</div>";
  body += "</div>";
  body += "</div>";

  body += "<div class='card tab hidden' id='tab-oled'>";
  body += "<h3>OLED SSD1306</h3>";
  body += "<form method='POST' action='/save_oled'>";
  body += "<label><input type='checkbox' name='oledEn' value='1' " + String(g_cfg.oledEnabled ? "checked" : "") + "> Enable OLED output</label>";
  body += "<label>Text size</label><select name='oledSz'>"
          "<option value='1' " + String(g_cfg.oledTextSize == 1 ? "selected" : "") + ">Small (1)</option>"
          "<option value='2' " + String(g_cfg.oledTextSize == 2 ? "selected" : "") + ">Medium (2)</option>"
          "<option value='3' " + String(g_cfg.oledTextSize == 3 ? "selected" : "") + ">Large (3)</option>"
          "</select>";
  body += "<p class='muted'>I2C address default: <code>0x3C</code></p>";

  body += "<h4 style='margin:12px 0 6px'>BLE sensors</h4>";
  body += "<div class='tablewrap'><table><tr><th>Show</th><th>Name</th><th>tempc</th><th>hum</th><th>volt</th><th>batt</th></tr>";
  for (const auto& s : g_sensors) {
    const String macNo = macNoColonsUpper(s.mac);
    const String nameVal = prefGetName(prefKeyBleName(macNo), String("BLE ") + macWithColonsUpper(s.mac));
    const String selKey = prefKeyBleOledSel(macNo);
    const String tcKey = prefKeyBleOledTempc(macNo);
    const String humKey = prefKeyBleOledHum(macNo);
    const String voltKey = prefKeyBleOledVolt(macNo);
    const String battKey = prefKeyBleOledBatt(macNo);
    body += "<tr>";
    body += "<td><input type='checkbox' name='" + selKey + "' value='1' " + String(prefGetBool(selKey) ? "checked" : "") + "></td>";
    body += "<td>" + htmlEscape(nameVal) + "</td>";
    body += "<td><input type='checkbox' name='" + tcKey + "' value='1' " + String(prefGetBool(tcKey) ? "checked" : "") + "></td>";
    body += "<td><input type='checkbox' name='" + humKey + "' value='1' " + String(prefGetBool(humKey) ? "checked" : "") + "></td>";
    body += "<td><input type='checkbox' name='" + voltKey + "' value='1' " + String(prefGetBool(voltKey) ? "checked" : "") + "></td>";
    body += "<td><input type='checkbox' name='" + battKey + "' value='1' " + String(prefGetBool(battKey) ? "checked" : "") + "></td>";
    body += "</tr>";
  }
  body += "</table></div>";

#if NETTEMP_ENABLE_I2C
  body += "<h4 style='margin:12px 0 6px'>I2C sensors</h4>";
  body += "<div class='tablewrap'><table><tr><th>Show</th><th>Name</th><th>tempc</th><th>hum</th><th>press_hpa</th></tr>";
  for (const auto& s : g_i2cSensors) {
    const String addrHex = (s.address < 16 ? String("0") : String("")) + String(s.address, HEX);
    const String nameVal = prefGetName(prefKeyI2cName(addrHex), prefGetName(String("i2cname_") + addrHex, String("I2C 0x") + addrHex + " " + i2cSensorTypeName(s.type)));
    const String selKey = prefKeyI2cOledSel(addrHex);
    const String tcKey = prefKeyI2cOledTempc(addrHex);
    const String humKey = prefKeyI2cOledHum(addrHex);
    const String pressKey = prefKeyI2cOledPress(addrHex);
    body += "<tr>";
    body += "<td><input type='checkbox' name='" + selKey + "' value='1' " + String(prefGetBool(selKey) ? "checked" : "") + "></td>";
    body += "<td>" + htmlEscape(nameVal) + "</td>";
    body += "<td><input type='checkbox' name='" + tcKey + "' value='1' " + String(prefGetBool(tcKey) ? "checked" : "") + "></td>";
    body += "<td><input type='checkbox' name='" + humKey + "' value='1' " + String(prefGetBool(humKey) ? "checked" : "") + "></td>";
    body += "<td><input type='checkbox' name='" + pressKey + "' value='1' " + String(prefGetBool(pressKey) ? "checked" : "") + "></td>";
    body += "</tr>";
  }
  body += "</table></div>";
#endif

  body += "<h4 style='margin:12px 0 6px'>GPIO sensors</h4>";
  body += "<div class='tablewrap'><table><tr><th>Show</th><th>Name</th><th>tempc</th><th>tempf</th><th>hum</th><th>volt</th><th>batt</th><th>soil_raw</th><th>soil_pct</th><th>dist_cm</th></tr>";
  if (g_cfg.dhtEnabled) {
    const String nameVal = prefGetName("name_dht", String("DHT") + String(g_cfg.dhtType) + " GPIO" + String(g_cfg.dhtPin));
    body += "<tr>";
    body += "<td><input type='checkbox' name='" + String(PREF_OLED_DHT) + "' value='1' " + String(prefGetBool(PREF_OLED_DHT) ? "checked" : "") + "></td>";
    body += "<td>" + htmlEscape(nameVal) + "</td>";
    body += "<td><input type='checkbox' name='" + String(PREF_OLED_DHT_T) + "' value='1' " + String(prefGetBool(PREF_OLED_DHT_T) ? "checked" : "") + "></td>";
    body += "<td><input type='checkbox' name='" + String(PREF_OLED_DHT_F) + "' value='1' " + String(prefGetBool(PREF_OLED_DHT_F) ? "checked" : "") + "></td>";
    body += "<td><input type='checkbox' name='" + String(PREF_OLED_DHT_H) + "' value='1' " + String(prefGetBool(PREF_OLED_DHT_H) ? "checked" : "") + "></td>";
    body += "<td>-</td><td>-</td><td>-</td><td>-</td><td>-</td>";
    body += "</tr>";
  }
  if (g_cfg.dsEnabled) {
    for (size_t i = 0; i < g_dsRoms.size(); i++) {
      char romHex[17]{};
      for (int b = 0; b < 8; b++) sprintf(romHex + b * 2, "%02X", g_dsRoms[i][b]);
      const String nameVal = prefGetDsName(romHex, String("DS18B20 ") + romHex);
      const String selKey = prefKeyDsOledSel(romHex);
      const String tcKey = prefKeyDsOledTempc(romHex);
      const String tfKey = prefKeyDsOledTempf(romHex);
      body += "<tr>";
      body += "<td><input type='checkbox' name='" + selKey + "' value='1' " + String(prefGetBool(selKey) ? "checked" : "") + "></td>";
      body += "<td>" + htmlEscape(nameVal) + "</td>";
      body += "<td><input type='checkbox' name='" + tcKey + "' value='1' " + String(prefGetBool(tcKey) ? "checked" : "") + "></td>";
      body += "<td><input type='checkbox' name='" + tfKey + "' value='1' " + String(prefGetBool(tfKey) ? "checked" : "") + "></td>";
      body += "<td>-</td><td>-</td><td>-</td><td>-</td><td>-</td><td>-</td>";
      body += "</tr>";
    }
  }
  if (g_cfg.soilEnabled) {
    const String nameVal = prefGetName("name_soil", String("SOIL ADC") + String(g_cfg.soilAdcPin));
    body += "<tr>";
    body += "<td><input type='checkbox' name='" + String(PREF_OLED_SOIL) + "' value='1' " + String(prefGetBool(PREF_OLED_SOIL) ? "checked" : "") + "></td>";
    body += "<td>" + htmlEscape(nameVal) + "</td>";
    body += "<td>-</td><td>-</td><td>-</td><td>-</td><td>-</td>";
    body += "<td><input type='checkbox' name='" + String(PREF_OLED_SOIL_RAW) + "' value='1' " + String(prefGetBool(PREF_OLED_SOIL_RAW) ? "checked" : "") + "></td>";
    body += "<td><input type='checkbox' name='" + String(PREF_OLED_SOIL_PCT) + "' value='1' " + String(prefGetBool(PREF_OLED_SOIL_PCT) ? "checked" : "") + "></td>";
    body += "<td>-</td>";
    body += "</tr>";
  }
  if (g_cfg.hcsr04Enabled) {
    const String nameVal = prefGetName("name_hcsr04", String("HC-SR04 GPIO") + String(g_cfg.hcsr04TrigPin) + "/" + String(g_cfg.hcsr04EchoPin));
    body += "<tr>";
    body += "<td><input type='checkbox' name='" + String(PREF_OLED_HC) + "' value='1' " + String(prefGetBool(PREF_OLED_HC) ? "checked" : "") + "></td>";
    body += "<td>" + htmlEscape(nameVal) + "</td>";
    body += "<td>-</td><td>-</td><td>-</td><td>-</td><td>-</td><td>-</td><td>-</td>";
    body += "<td><input type='checkbox' name='" + String(PREF_OLED_HC_DIST) + "' value='1' " + String(prefGetBool(PREF_OLED_HC_DIST) ? "checked" : "") + "></td>";
    body += "</tr>";
  }
  if (g_cfg.vbatMode != 0) {
    const String nameVal = prefGetName("name_vbat", "VBAT");
    body += "<tr>";
    body += "<td><input type='checkbox' name='" + String(PREF_OLED_VBAT) + "' value='1' " + String(prefGetBool(PREF_OLED_VBAT) ? "checked" : "") + "></td>";
    body += "<td>" + htmlEscape(nameVal) + "</td>";
    body += "<td>-</td><td>-</td><td>-</td>";
    body += "<td><input type='checkbox' name='" + String(PREF_OLED_VBAT_V) + "' value='1' " + String(prefGetBool(PREF_OLED_VBAT_V) ? "checked" : "") + "></td>";
    body += "<td><input type='checkbox' name='" + String(PREF_OLED_VBAT_B) + "' value='1' " + String(prefGetBool(PREF_OLED_VBAT_B) ? "checked" : "") + "></td>";
    body += "<td>-</td><td>-</td><td>-</td>";
    body += "</tr>";
  }
  body += "</table></div>";

  body += "<p style='margin:12px 0 0'><button type='submit'>Save</button></p>";
  body += "</form>";
  body += "</div>";

  body += "<div class='card tab hidden' id='tab-channels'>";
  body += "<h3>Channels</h3>";
  body += "<div class='tile-grid'>";
  body += "<div class='tile'>";
  body += "<h4 style='margin:0 0 4px'>MQTT <span style='color:var(--muted);font-size:12px;font-weight:400'>(" + htmlEscape(formatSendStatus(g_lastMqttLocalSentMs, g_cfg.mqttIntervalMs)) + ")</span></h4>";
  body += "<form method='POST' action='/send_mqtt' style='margin:0 0 8px'><button type='submit'>Send now</button></form>";
  body += "<form method='POST' action='/save_mqtt'>";
  body += "<label><input type='checkbox' name='mqttEn' value='1' " + String(g_cfg.mqttEnabled ? "checked" : "") + "> Enable MQTT</label>";
  body += "<div class='grid2'><div><label>Host</label><input name='mqttHost' value='" + htmlEscape(g_cfg.mqttHost) + "'/></div>";
  body += "<div><label>Port</label><input name='mqttPort' value='" + String((unsigned)g_cfg.mqttPort) + "'/></div></div>";
  body += "<div class='grid2'><div><label>Username</label><input name='mqttUser' value='" + htmlEscape(g_cfg.mqttUser) + "'/></div>";
  body += "<div><label>Password</label><input name='mqttPass' type='password' value='' placeholder='(leave empty to keep existing)'/></div></div>";
  body += "<label>Interval (seconds)</label><input name='mqttInt' value='" + String((unsigned long)(g_cfg.mqttIntervalMs / 1000UL)) + "'/>";
  body += "<label style='margin-top:12px'>Send data from</label>";
  body += "<div style='display:flex;gap:12px;flex-wrap:wrap'>";
  body += "<label><input type='checkbox' name='mqttBle' value='1' " + String(g_cfg.bleSendMqtt ? "checked" : "") + "> BLE</label>";
  body += "<label><input type='checkbox' name='mqttGpio' value='1' " + String(g_cfg.gpioSendMqtt ? "checked" : "") + "> GPIO</label>";
  body += "<label><input type='checkbox' name='mqttI2c' value='1' " + String(g_cfg.i2cSendMqtt ? "checked" : "") + "> I2C</label>";
  body += "</div>";
  body += "<label style='margin-top:12px'>MQTT JSON fields (BLE)</label>";
  body += "<label><input type='checkbox' name='m_f_tempc' value='1' " + String((g_mqttBleFields & SRV_BLE_TEMPC) ? "checked" : "") + "> tempc</label>";
  body += "<label><input type='checkbox' name='m_f_tempf' value='1' " + String((g_mqttBleFields & SRV_BLE_TEMPF) ? "checked" : "") + "> tempf</label>";
  body += "<label><input type='checkbox' name='m_f_hum' value='1' " + String((g_mqttBleFields & SRV_BLE_HUM) ? "checked" : "") + "> hum</label>";
  body += "<label><input type='checkbox' name='m_f_batt' value='1' " + String((g_mqttBleFields & SRV_BLE_BATT) ? "checked" : "") + "> batt</label>";
  body += "<label><input type='checkbox' name='m_f_volt' value='1' " + String((g_mqttBleFields & SRV_BLE_VOLT) ? "checked" : "") + "> volt</label>";
  body += "<label><input type='checkbox' name='m_f_rssi' value='1' " + String((g_mqttBleFields & SRV_BLE_RSSI) ? "checked" : "") + "> rssi</label>";
#if NETTEMP_ENABLE_I2C
  body += "<label style='margin-top:12px'>MQTT JSON fields (I2C)</label>";
  body += "<label><input type='checkbox' name='m_f_i2c_tempc' value='1' " + String((g_mqttI2cFields & SRV_I2C_TEMPC) ? "checked" : "") + "> tempc</label>";
  body += "<label><input type='checkbox' name='m_f_i2c_hum' value='1' " + String((g_mqttI2cFields & SRV_I2C_HUM) ? "checked" : "") + "> hum</label>";
  body += "<label><input type='checkbox' name='m_f_i2c_press' value='1' " + String((g_mqttI2cFields & SRV_I2C_PRESS) ? "checked" : "") + "> press_hpa</label>";
#endif
  body += "<label style='margin-top:12px'>MQTT JSON fields (Soil)</label>";
  body += "<label><input type='checkbox' name='m_f_soil_raw' value='1' " + String(g_cfg.mqttSoilSendRaw ? "checked" : "") + "> soil_raw</label>";
  body += "<label><input type='checkbox' name='m_f_soil_pct' value='1' " + String(g_cfg.mqttSoilSendPct ? "checked" : "") + "> soil_pct</label>";
  body += "<p style='margin:12px 0 0'><button type='submit'>Save</button></p>";
  body += "</form>";
  body += "</div>";

  body += "<div class='tile'>";
  body += "<h4 style='margin:0 0 4px'>Server <span style='color:var(--muted);font-size:12px;font-weight:400'>(" + htmlEscape(formatSendStatus(g_lastServerSendMs, g_cfg.serverIntervalMs)) + ")</span></h4>";
  body += "<form method='POST' action='/send_server' style='margin:0 0 8px'><button type='submit'>Send now</button></form>";
  body += "<form method='POST' action='/save_server'>";
  body += "<label><input type='checkbox' name='srvEn' value='1' " + String(g_cfg.serverEnabled ? "checked" : "") + "> Enable Server</label>";
  body += "<label>Base URL</label><input name='srvUrl' value='" + htmlEscape(g_cfg.serverBaseUrl) + "'/>";
  body += "<label>API key (ntk_...)</label><input name='srvKey' type='password' value='' placeholder='(leave empty to keep existing)'/>";
  body += "<label>Interval (seconds)</label><input name='srvInt' value='" + String((unsigned long)(g_cfg.serverIntervalMs / 1000UL)) + "'/>";
  body += "<label style='margin-top:12px'>Send data from</label>";
  body += "<div style='display:flex;gap:12px;flex-wrap:wrap'>";
  body += "<label><input type='checkbox' name='srvBle' value='1' " + String(g_cfg.bleSendServer ? "checked" : "") + "> BLE</label>";
  body += "<label><input type='checkbox' name='srvGpio' value='1' " + String(g_cfg.gpioSendServer ? "checked" : "") + "> GPIO</label>";
  body += "<label><input type='checkbox' name='srvI2c' value='1' " + String(g_cfg.i2cSendServer ? "checked" : "") + "> I2C</label>";
  body += "</div>";
  body += "<label style='margin-top:12px'>Server JSON fields (BLE)</label>";
  body += "<label><input type='checkbox' name='f_tempc' value='1' " + String((g_srvBleFields & SRV_BLE_TEMPC) ? "checked" : "") + "> tempc</label>";
  body += "<label><input type='checkbox' name='f_tempf' value='1' " + String((g_srvBleFields & SRV_BLE_TEMPF) ? "checked" : "") + "> tempf</label>";
  body += "<label><input type='checkbox' name='f_hum' value='1' " + String((g_srvBleFields & SRV_BLE_HUM) ? "checked" : "") + "> hum</label>";
  body += "<label><input type='checkbox' name='f_batt' value='1' " + String((g_srvBleFields & SRV_BLE_BATT) ? "checked" : "") + "> batt</label>";
  body += "<label><input type='checkbox' name='f_volt' value='1' " + String((g_srvBleFields & SRV_BLE_VOLT) ? "checked" : "") + "> volt</label>";
  body += "<label><input type='checkbox' name='f_rssi' value='1' " + String((g_srvBleFields & SRV_BLE_RSSI) ? "checked" : "") + "> rssi</label>";
#if NETTEMP_ENABLE_I2C
  body += "<label style='margin-top:12px'>Server JSON fields (I2C)</label>";
  body += "<label><input type='checkbox' name='f_i2c_tempc' value='1' " + String((g_srvI2cFields & SRV_I2C_TEMPC) ? "checked" : "") + "> tempc</label>";
  body += "<label><input type='checkbox' name='f_i2c_hum' value='1' " + String((g_srvI2cFields & SRV_I2C_HUM) ? "checked" : "") + "> hum</label>";
  body += "<label><input type='checkbox' name='f_i2c_press' value='1' " + String((g_srvI2cFields & SRV_I2C_PRESS) ? "checked" : "") + "> press_hpa</label>";
#endif
  body += "<label style='margin-top:12px'>Server JSON fields (Soil)</label>";
  body += "<label><input type='checkbox' name='f_soil_raw' value='1' " + String(g_cfg.soilSendRaw ? "checked" : "") + "> soil_raw</label>";
  body += "<label><input type='checkbox' name='f_soil_pct' value='1' " + String(g_cfg.soilSendPct ? "checked" : "") + "> soil_pct</label>";
  body += "<p style='margin:12px 0 0'><button type='submit'>Save</button></p>";
  body += "</form>";
  body += "</div>";

  body += "<div class='tile'>";
  body += "<h4 style='margin:0 0 4px'>Webhook <span style='color:var(--muted);font-size:12px;font-weight:400'>(" + htmlEscape(formatSendStatus(g_lastWebhookSendMs, g_cfg.webhookIntervalMs)) + ")</span></h4>";
  body += "<form method='POST' action='/send_webhook' style='margin:0 0 8px'><button type='submit'>Send now</button></form>";
  body += "<form method='POST' action='/save_webhook'>";
  body += "<label><input type='checkbox' name='whEn' value='1' " + String(g_cfg.webhookEnabled ? "checked" : "") + "> Enable Webhook</label>";
  body += "<label>Webhook URL</label><input name='whUrl' value='" + htmlEscape(g_cfg.webhookUrl) + "' placeholder='https://example.com/hook'/>";
  body += "<label>Interval (seconds)</label><input name='whInt' value='" + String((unsigned long)(g_cfg.webhookIntervalMs / 1000UL)) + "'/>";
  body += "<label style='margin-top:12px'>Webhook JSON fields (BLE)</label>";
  body += "<label><input type='checkbox' name='w_f_tempc' value='1' " + String((g_webhookBleFields & SRV_BLE_TEMPC) ? "checked" : "") + "> tempc</label>";
  body += "<label><input type='checkbox' name='w_f_tempf' value='1' " + String((g_webhookBleFields & SRV_BLE_TEMPF) ? "checked" : "") + "> tempf</label>";
  body += "<label><input type='checkbox' name='w_f_hum' value='1' " + String((g_webhookBleFields & SRV_BLE_HUM) ? "checked" : "") + "> hum</label>";
  body += "<label><input type='checkbox' name='w_f_batt' value='1' " + String((g_webhookBleFields & SRV_BLE_BATT) ? "checked" : "") + "> batt</label>";
  body += "<label><input type='checkbox' name='w_f_volt' value='1' " + String((g_webhookBleFields & SRV_BLE_VOLT) ? "checked" : "") + "> volt</label>";
  body += "<label><input type='checkbox' name='w_f_rssi' value='1' " + String((g_webhookBleFields & SRV_BLE_RSSI) ? "checked" : "") + "> rssi</label>";
#if NETTEMP_ENABLE_I2C
  body += "<label style='margin-top:12px'>Webhook JSON fields (I2C)</label>";
  body += "<label><input type='checkbox' name='w_f_i2c_tempc' value='1' " + String((g_webhookI2cFields & SRV_I2C_TEMPC) ? "checked" : "") + "> tempc</label>";
  body += "<label><input type='checkbox' name='w_f_i2c_hum' value='1' " + String((g_webhookI2cFields & SRV_I2C_HUM) ? "checked" : "") + "> hum</label>";
  body += "<label><input type='checkbox' name='w_f_i2c_press' value='1' " + String((g_webhookI2cFields & SRV_I2C_PRESS) ? "checked" : "") + "> press_hpa</label>";
#endif
  body += "<label style='margin-top:12px'>Send data from</label>";
  body += "<div style='display:flex;gap:12px;flex-wrap:wrap'>";
  body += "<label><input type='checkbox' name='whBle' value='1' " + String(g_cfg.bleSendWebhook ? "checked" : "") + "> BLE</label>";
  body += "<label><input type='checkbox' name='whGpio' value='1' " + String(g_cfg.gpioSendWebhook ? "checked" : "") + "> GPIO</label>";
  body += "<label><input type='checkbox' name='whI2c' value='1' " + String(g_cfg.i2cSendWebhook ? "checked" : "") + "> I2C</label>";
  body += "</div>";
  body += "<p style='margin:12px 0 0'><button type='submit'>Save</button></p>";
  body += "</form>";
  body += "</div>";
  body += "</div>";
  body += "</div>";

  body += "<div class='card tab hidden' id='tab-device'>";
  body += "<h3>Device</h3>";
  body += "<div class='tile-grid'>";
  body += "<div class='tile'>";
  body += "<h4 style='margin:0 0 8px'>Device ID</h4>";
  body += "<form method='POST' action='/save_device'>";
  body += "<label>Device ID (I2C/GPIO + sensor_id prefix)</label><input name='deviceId' value='" + htmlEscape(g_cfg.deviceId) + "'/>";
  body += "<p style='margin:12px 0 0'><button type='submit'>Save</button></p>";
  body += "</form>";
  body += "</div>";
  body += "<div class='tile'>";
  body += "<h4 style='margin:0 0 8px'>Sleep</h4>";
  body += "<form method='POST' action='/save_power'>";
  body += "<label><input type='checkbox' name='pwrSlp' value='1' " + String(g_cfg.powerSleepEnabled ? "checked" : "") + "> Enable deep sleep duty-cycle</label>";
  body += "<label>Boot grace (seconds; power-on/reset)</label><input name='pwrBootS' value='" + String((unsigned long)g_cfg.powerBootGraceSeconds) + "' placeholder='30 (0=never sleep on boot)'/>";
  body += "<label>Awake hold (seconds; after cycle)</label><input name='pwrHoldS' value='" + String((unsigned long)g_cfg.powerAwakeHoldSeconds) + "' placeholder='5 (0=immediate sleep)'/>";
  body += "<label>Sleep interval (seconds)</label><input name='pwrSlpS' value='" + String((unsigned long)g_cfg.powerSleepSeconds) + "'/>";
  body += "<label>BLE scan per cycle (seconds)</label><input name='pwrBleS' value='" + String((unsigned long)g_cfg.powerBleScanSeconds) + "'/>";
  body += "<label>WiFi wait per cycle (seconds)</label><input name='pwrWiS' value='" + String((unsigned long)g_cfg.powerWifiWaitSeconds) + "'/>";
  body += "<p style='margin:12px 0 0'><button type='submit'>Save</button></p>";
  body += "</form>";
  body += "</div>";
  body += "<div class='tile'>";
  body += "<h4 style='margin:0 0 8px'>Firmware update</h4>";
  const uint32_t otaFree = ESP.getFreeSketchSpace();
  const uint32_t otaMax = otaFree & 0xFFFFF000;
  body += "<form method='POST' action='/ota' enctype='multipart/form-data' onsubmit='otaOnSubmit(event)'>";
  body += "<label>Upload firmware (.bin)</label><input type='file' name='firmware' id='ota-file' accept='.bin' data-max='" + String(otaMax) + "'/>";
  body += "<p style='color:var(--muted);margin:4px 0 0'>OTA max size: <code>" + String(otaMax / 1024) + " KB</code></p>";
  body += "<p style='margin:12px 0 0'><button type='submit' id='ota-submit'>Upload & Update</button></p>";
  body += "<p id='ota-status' style='color:var(--muted);margin:4px 0 0'>Idle.</p>";
  body += "<p id='ota-progress' style='color:var(--muted);margin:4px 0 0'></p>";
  body += "</form>";
  body += "</div>";
  body += "</div>";
  body += "</div>";

  body += "<div class='card tab hidden' id='tab-admin'>";
  body += "<h3>Admin</h3>";
  body += "<form method='POST' action='/save_admin'>";
  body += "<label>Username</label><input name='webUser' value='" + htmlEscape(g_cfg.webUser) + "'/>";
  body += "<label>Password</label><input name='webPass' type='password' value='' placeholder='(leave empty to keep existing)'/>";
  body += "<p style='margin:12px 0 0'><button type='submit'>Save</button></p>";
  body += "</form>";
  body += "</div>";

  body += "<div class='card tab hidden' id='tab-system'>";
  body += "<h3>System</h3>";
  body += "<p><code>" + String(ESP.getChipModel()) + "</code> | rev <code>" + String(ESP.getChipRevision()) +
          "</code> | cores <code>" + String(ESP.getChipCores()) + "</code> | CPU <code>" +
          String((unsigned long)(ESP.getCpuFreqMHz())) + " MHz</code></p>";
  body += "<p>WiFi: <code>" + String(wifiConnected() ? "connected" : "not connected") + "</code>";
#if NETTEMP_HEADLESS
  body += " | BLE autoscan: <code>" + String(g_headlessBleAutoScan ? "on" : "off") + "</code>";
#else
  body += " | BLE scan: <code>on</code> (" + String(g_activeScan ? "active" : "passive") + ")";
#endif
  body += "</p>";
  body += "<p>Power sleep: <code>" + String(g_cfg.powerSleepEnabled ? "on" : "off") + "</code> | sleep_s <code>" + String((unsigned long)g_cfg.powerSleepSeconds) +
          "</code> | ble_scan_s <code>" + String((unsigned long)g_cfg.powerBleScanSeconds) + "</code></p>";
  {
    const auto cause = esp_sleep_get_wakeup_cause();
    const auto rr = esp_reset_reason();
    String causeStr = "other";
    if (cause == ESP_SLEEP_WAKEUP_TIMER) causeStr = "timer";
    else if (cause == ESP_SLEEP_WAKEUP_UNDEFINED) causeStr = "undef";
    else if (cause == ESP_SLEEP_WAKEUP_EXT0) causeStr = "ext0";
    else if (cause == ESP_SLEEP_WAKEUP_EXT1) causeStr = "ext1";
    else if (cause == ESP_SLEEP_WAKEUP_TOUCHPAD) causeStr = "touch";
    else if (cause == ESP_SLEEP_WAKEUP_ULP) causeStr = "ulp";
    String rrStr = String((int)rr);
    if (rr == ESP_RST_DEEPSLEEP) rrStr = "deepsleep";
    else if (rr == ESP_RST_POWERON) rrStr = "poweron";
    else if (rr == ESP_RST_SW) rrStr = "software";
    else if (rr == ESP_RST_EXT) rrStr = "external";
    else if (rr == ESP_RST_PANIC) rrStr = "panic";
    const uint32_t bootAgeS = (g_powerBootMs == 0 || millis() < g_powerBootMs) ? 0 : ((millis() - g_powerBootMs) / 1000UL);
    body += "<p class='muted'>Wakeup: <code>" + causeStr + "</code> | boot_age_s <code>" + String((unsigned long)bootAgeS) +
            "</code> | boot_grace_s <code>" + String((unsigned long)g_cfg.powerBootGraceSeconds) +
            "</code> | boot_cycle_done <code>" + String(g_powerBootCycleDone ? "yes" : "no") +
            "</code> | cycle_done_this_boot <code>" + String(g_powerCycleDoneThisBoot ? "yes" : "no") + "</code></p>";
    body += "<p class='muted'>Reset reason: <code>" + rrStr + "</code></p>";
#if NETTEMP_ENABLE_PORTAL
    String webModeStr = "none";
    if (g_webMode == WebMode::ApSetup) webModeStr = "ap_setup";
    else if (g_webMode == WebMode::StaConfig) webModeStr = "sta_config";
    body += "<p class='muted'>Web: <code>" + (g_portalRunning ? String("running") : String("stopped")) + "</code> | mode <code>" + webModeStr + "</code></p>";
#endif
  }
  body += "<div class='row' style='align-items:center;gap:10px'>"
          "<form method='POST' action='/reboot'><button type='submit'>Reboot</button></form>"
          "<form method='POST' action='/reset' onsubmit=\"return confirm('Reset all settings?');\"><button type='submit'>Factory Reset</button></form>"
          "</div>";
  body += "</div>";

  body += "</div></body></html>";
  g_prefs.end();
  g_webServer.send(200, "text/html; charset=utf-8", body);
}

static void configHandleSaveSensors() {
  if (!webRequireAuth()) return;
  // Apply per-device manual overrides (or clear them).
  g_prefs.begin("nettemp", false);
  for (const auto& s : g_sensors) {
    const bool hasValues = !isnan(s.temperatureC) || !isnan(s.humidityPct) || s.batteryPct >= 0;
    if (!hasValues) continue;
    const String macNo = macNoColonsUpper(s.mac);
    const String selKey = "sel_" + macNo;
    const String manKey = "man_" + macNo;
    const String nameKey = "bleName_" + macNo;
    if (g_webServer.hasArg(nameKey)) {
      const String prefKey = prefKeyBleName(macNo);
      g_prefs.putString(prefKey.c_str(), g_webServer.arg(nameKey));
    }
    if (g_webServer.hasArg(manKey)) {
      const bool selected = g_webServer.hasArg(selKey);
      bleManualSet(macNo, selected);
    } else {
      bleManualClear(macNo);
    }
  }
  g_prefs.end();
  prefsSave();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

#if NETTEMP_ENABLE_I2C
static void configHandleSaveI2c() {
  if (!webRequireAuth()) return;
  g_i2cSelDefined = true;
  g_i2cSelAddrs.clear();

  g_prefs.begin("nettemp", false);
  for (auto& s : g_i2cSensors) {
    const String addrHex = (s.address < 16 ? String("0") : String("")) + String(s.address, HEX);
    const String key = "i2c_" + addrHex;
    const String nameKey = "i2cName_" + addrHex;
    if (g_webServer.hasArg(nameKey)) {
      const String prefKey = prefKeyI2cName(addrHex);
      g_prefs.putString(prefKey.c_str(), g_webServer.arg(nameKey));
    }
    const bool selected = g_webServer.hasArg(key);
    s.selected = selected;
    if (selected) g_i2cSelAddrs.push_back(s.address);
  }
  g_prefs.end();

  prefsSave();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

static void configHandleI2cRescan() {
  if (!webRequireAuth()) return;

  // If there is no saved selection yet, treat current selection as the baseline and persist it.
  if (!g_i2cSelDefined) {
    g_i2cSelAddrs.clear();
    for (const auto& s : g_i2cSensors) {
      if (s.selected) g_i2cSelAddrs.push_back(s.address);
    }
    g_i2cSelDefined = true;
  }

  i2cInitBus();
  g_i2cDetectedAddrs = i2cScanAllAddresses(Wire);
  g_i2cSensors = i2cDetectKnownSensors(Wire);
  if (!g_i2cSensors.empty()) i2cUpdateReadings(Wire, g_i2cSensors);
  i2cApplySelectionToDetected();
  i2cPersistLastDetected();

  prefsSave();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

static void configHandleI2cPins() {
  if (!webRequireAuth()) return;

  const String sdaStr = g_webServer.arg("i2cSda");
  const String sclStr = g_webServer.arg("i2cScl");
  const int sda = sdaStr.length() ? sdaStr.toInt() : g_cfg.i2cSdaPin;
  const int scl = sclStr.length() ? sclStr.toInt() : g_cfg.i2cSclPin;

  // Allow -1 as "board default". Basic sanity range otherwise.
  g_cfg.i2cSdaPin = (sda < 0) ? -1 : std::min(48, sda);
  g_cfg.i2cSclPin = (scl < 0) ? -1 : std::min(48, scl);
  prefsSave();

  i2cInitBus();
  g_i2cDetectedAddrs = i2cScanAllAddresses(Wire);
  g_i2cSensors = i2cDetectKnownSensors(Wire);
  if (!g_i2cSensors.empty()) i2cUpdateReadings(Wire, g_i2cSensors);
  if (g_i2cSelDefined) i2cApplySelectionToDetected();
  i2cPersistLastDetected();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}
#endif

static void configHandleSaveBle() {
  if (!webRequireAuth()) return;

#if NETTEMP_HEADLESS
  g_headlessBleAutoScan = g_webServer.hasArg("bleAuto");
#endif
  prefsSave();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

static void configHandleSaveMqtt() {
  if (!webRequireAuth()) return;

  g_cfg.mqttEnabled = g_webServer.hasArg("mqttEn");
  const String mqttHost = g_webServer.arg("mqttHost");
  if (mqttHost.length()) g_cfg.mqttHost = mqttHost;
  const uint32_t mqttPort = parseU32Or(g_webServer.arg("mqttPort"), g_cfg.mqttPort);
  if (mqttPort > 0 && mqttPort <= 65535) g_cfg.mqttPort = (uint16_t)mqttPort;
  g_cfg.mqttUser = g_webServer.arg("mqttUser");
  const String mqttPass = g_webServer.arg("mqttPass");
  if (mqttPass.length()) g_cfg.mqttPass = mqttPass;
  const uint32_t mqttIntS = parseU32Or(g_webServer.arg("mqttInt"), (g_cfg.mqttIntervalMs / 1000UL));
  if (mqttIntS >= 5 && mqttIntS <= 3600) g_cfg.mqttIntervalMs = mqttIntS * 1000UL;
  g_cfg.bleSendMqtt = g_webServer.hasArg("mqttBle");
  g_cfg.gpioSendMqtt = g_webServer.hasArg("mqttGpio");
  g_cfg.i2cSendMqtt = g_webServer.hasArg("mqttI2c");
  g_cfg.mqttSoilSendRaw = g_webServer.hasArg("m_f_soil_raw");
  g_cfg.mqttSoilSendPct = g_webServer.hasArg("m_f_soil_pct");

  uint32_t mask = 0;
  if (g_webServer.hasArg("m_f_tempc")) mask |= SRV_BLE_TEMPC;
  if (g_webServer.hasArg("m_f_tempf")) mask |= SRV_BLE_TEMPF;
  if (g_webServer.hasArg("m_f_hum")) mask |= SRV_BLE_HUM;
  if (g_webServer.hasArg("m_f_batt")) mask |= SRV_BLE_BATT;
  if (g_webServer.hasArg("m_f_volt")) mask |= SRV_BLE_VOLT;
  if (g_webServer.hasArg("m_f_rssi")) mask |= SRV_BLE_RSSI;
  if (mask == 0) mask = (SRV_BLE_TEMPC | SRV_BLE_HUM | SRV_BLE_BATT);
  g_mqttBleFields = mask;

#if NETTEMP_ENABLE_I2C
  uint32_t i2cMask = 0;
  if (g_webServer.hasArg("m_f_i2c_tempc")) i2cMask |= SRV_I2C_TEMPC;
  if (g_webServer.hasArg("m_f_i2c_hum")) i2cMask |= SRV_I2C_HUM;
  if (g_webServer.hasArg("m_f_i2c_press")) i2cMask |= SRV_I2C_PRESS;
  if (i2cMask == 0) i2cMask = (SRV_I2C_TEMPC | SRV_I2C_HUM | SRV_I2C_PRESS);
  g_mqttI2cFields = i2cMask;
#endif

  prefsSave();
  mqttEnsureConnected();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

static void configHandleSaveServer() {
  if (!webRequireAuth()) return;

  g_cfg.serverEnabled = g_webServer.hasArg("srvEn");
  const String srvUrl = g_webServer.arg("srvUrl");
  if (srvUrl.length()) g_cfg.serverBaseUrl = srvUrl;
  const String srvKey = g_webServer.arg("srvKey");
  if (srvKey.length()) g_cfg.serverApiKey = srvKey;
  const uint32_t srvIntS = parseU32Or(g_webServer.arg("srvInt"), (g_cfg.serverIntervalMs / 1000UL));
  if (srvIntS >= 5 && srvIntS <= 3600) g_cfg.serverIntervalMs = srvIntS * 1000UL;
  g_cfg.bleSendServer = g_webServer.hasArg("srvBle");
  g_cfg.gpioSendServer = g_webServer.hasArg("srvGpio");
  g_cfg.i2cSendServer = g_webServer.hasArg("srvI2c");
  g_cfg.soilSendRaw = g_webServer.hasArg("f_soil_raw");
  g_cfg.soilSendPct = g_webServer.hasArg("f_soil_pct");

  uint32_t mask = 0;
  if (g_webServer.hasArg("f_tempc")) mask |= SRV_BLE_TEMPC;
  if (g_webServer.hasArg("f_tempf")) mask |= SRV_BLE_TEMPF;
  if (g_webServer.hasArg("f_hum")) mask |= SRV_BLE_HUM;
  if (g_webServer.hasArg("f_batt")) mask |= SRV_BLE_BATT;
  if (g_webServer.hasArg("f_volt")) mask |= SRV_BLE_VOLT;
  if (g_webServer.hasArg("f_rssi")) mask |= SRV_BLE_RSSI;
  if (mask == 0) mask = (SRV_BLE_TEMPC | SRV_BLE_HUM | SRV_BLE_BATT);
  g_srvBleFields = mask;

#if NETTEMP_ENABLE_I2C
  uint32_t i2cMask = 0;
  if (g_webServer.hasArg("f_i2c_tempc")) i2cMask |= SRV_I2C_TEMPC;
  if (g_webServer.hasArg("f_i2c_hum")) i2cMask |= SRV_I2C_HUM;
  if (g_webServer.hasArg("f_i2c_press")) i2cMask |= SRV_I2C_PRESS;
  if (i2cMask == 0) i2cMask = (SRV_I2C_TEMPC | SRV_I2C_HUM | SRV_I2C_PRESS);
  g_srvI2cFields = i2cMask;
#endif

  prefsSave();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

static void configHandleSaveDevice() {
  if (!webRequireAuth()) return;

  const String deviceId = g_webServer.arg("deviceId");
  if (deviceId.length()) g_cfg.deviceId = deviceId;

  prefsSave();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

static void configHandleSaveWebhook() {
  if (!webRequireAuth()) return;

  g_cfg.webhookEnabled = g_webServer.hasArg("whEn");
  const String whUrl = g_webServer.arg("whUrl");
  if (whUrl.length()) g_cfg.webhookUrl = whUrl;
  const uint32_t whIntS = parseU32Or(g_webServer.arg("whInt"), (g_cfg.webhookIntervalMs / 1000UL));
  if (whIntS >= 5 && whIntS <= 3600) g_cfg.webhookIntervalMs = whIntS * 1000UL;
  uint32_t mask = 0;
  if (g_webServer.hasArg("w_f_tempc")) mask |= SRV_BLE_TEMPC;
  if (g_webServer.hasArg("w_f_tempf")) mask |= SRV_BLE_TEMPF;
  if (g_webServer.hasArg("w_f_hum")) mask |= SRV_BLE_HUM;
  if (g_webServer.hasArg("w_f_batt")) mask |= SRV_BLE_BATT;
  if (g_webServer.hasArg("w_f_volt")) mask |= SRV_BLE_VOLT;
  if (g_webServer.hasArg("w_f_rssi")) mask |= SRV_BLE_RSSI;
  if (mask == 0) mask = (SRV_BLE_TEMPC | SRV_BLE_HUM | SRV_BLE_BATT);
  g_webhookBleFields = mask;
#if NETTEMP_ENABLE_I2C
  uint32_t i2cMask = 0;
  if (g_webServer.hasArg("w_f_i2c_tempc")) i2cMask |= SRV_I2C_TEMPC;
  if (g_webServer.hasArg("w_f_i2c_hum")) i2cMask |= SRV_I2C_HUM;
  if (g_webServer.hasArg("w_f_i2c_press")) i2cMask |= SRV_I2C_PRESS;
  if (i2cMask == 0) i2cMask = (SRV_I2C_TEMPC | SRV_I2C_HUM | SRV_I2C_PRESS);
  g_webhookI2cFields = i2cMask;
#endif
  g_cfg.bleSendWebhook = g_webServer.hasArg("whBle");
  g_cfg.gpioSendWebhook = g_webServer.hasArg("whGpio");
  g_cfg.i2cSendWebhook = g_webServer.hasArg("whI2c");

  prefsSave();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

static void configHandleSendMqtt() {
  if (!webRequireAuth()) return;
  for (auto& s : g_sensors) s.lastMqttSentMs = 0;
#if NETTEMP_ENABLE_I2C
  for (auto& s : g_i2cSensors) s.last_mqtt_sent_ms = 0;
#endif
  g_lastMqttLocalSentMs = 0;
  tickSendMqtt();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

static void configHandleSendServer() {
  if (!webRequireAuth()) return;
  g_lastServerSendMs = 0;
  tickSendServer();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

#if NETTEMP_ENABLE_SERVER
static void configHandleSendWebhook() {
  if (!webRequireAuth()) return;
  g_lastWebhookSendMs = 0;
  tickSendWebhook();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}
#endif

static bool g_otaFailed = false;
static String g_otaStatus;

static void configHandleOtaFinish() {
  if (!webRequireAuth()) return;
  const bool ok = !Update.hasError() && !g_otaFailed;
  String msg = ok ? "Update OK. Rebooting..." : "Update failed.";
  if (!ok && g_otaStatus.length()) {
    msg = g_otaStatus;
  }
  g_webServer.send(ok ? 200 : 500, "text/plain", msg);
  if (ok) {
    delay(300);
    ESP.restart();
  }
}

static void configHandleOtaUpload() {
  if (!webRequireAuth()) return;
  HTTPUpload& upload = g_webServer.upload();
  if (upload.status == UPLOAD_FILE_START) {
    g_otaFailed = false;
    g_otaStatus = "";
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      g_otaFailed = true;
      g_otaStatus = String("Update failed: ") + Update.errorString() + " (check OTA partition scheme)";
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (!g_otaFailed) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
        g_otaFailed = true;
        g_otaStatus = String("Update failed: ") + Update.errorString() + " (check OTA partition scheme)";
      }
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (!g_otaFailed) {
      if (!Update.end(true)) {
        g_otaFailed = true;
        g_otaStatus = String("Update failed: ") + Update.errorString() + " (check OTA partition scheme)";
      }
    }
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    Update.end();
    g_otaFailed = true;
    g_otaStatus = "Update aborted.";
  }
}

static void configHandleSaveGpio() {
  if (!webRequireAuth()) return;

  // Handle scan button - just rescan and redirect
  if (g_webServer.hasArg("dsScan")) {
    dsRescan();
    g_webServer.sendHeader("Location", "/", true);
    g_webServer.send(302, "text/plain", "");
    return;
  }

  // Detect which sensor is being saved
  const String sensor = g_webServer.arg("sensor");

  // Update only the sensor being saved
  if (sensor == "ds18b20") {
    g_cfg.dsEnabled = g_webServer.hasArg("dsEn");
    g_cfg.dsPin = (int)parseU32Or(g_webServer.arg("dsPin"), (uint32_t)g_cfg.dsPin);
    dsEnsureBus();
  } else if (sensor == "dht") {
    g_cfg.dhtEnabled = g_webServer.hasArg("dhtEn");
    g_cfg.dhtPin = (int)parseU32Or(g_webServer.arg("dhtPin"), (uint32_t)g_cfg.dhtPin);
    g_cfg.dhtType = (int)parseU32Or(g_webServer.arg("dhtType"), (uint32_t)g_cfg.dhtType);
    if (g_cfg.dhtType != 11 && g_cfg.dhtType != 22) g_cfg.dhtType = 22;
    g_dhtTempC = NAN;
    g_dhtHumPct = NAN;
    g_dhtLastSeenMs = 0;
    g_dhtLastReadMs = 0;
  } else if (sensor == "soil") {
    g_cfg.soilEnabled = g_webServer.hasArg("soilEn");
    g_cfg.soilAdcPin = (int)parseU32Or(g_webServer.arg("soilPin"), (uint32_t)g_cfg.soilAdcPin);
    g_cfg.soilDryRaw = (int)parseU32Or(g_webServer.arg("soilDry"), (uint32_t)g_cfg.soilDryRaw);
    g_cfg.soilWetRaw = (int)parseU32Or(g_webServer.arg("soilWet"), (uint32_t)g_cfg.soilWetRaw);
  } else if (sensor == "hcsr04") {
    g_cfg.hcsr04Enabled = g_webServer.hasArg("hcEn");
    g_cfg.hcsr04TrigPin = (int)parseU32Or(g_webServer.arg("hcTrig"), (uint32_t)g_cfg.hcsr04TrigPin);
    g_cfg.hcsr04EchoPin = (int)parseU32Or(g_webServer.arg("hcEcho"), (uint32_t)g_cfg.hcsr04EchoPin);
    g_hcsr04Cm = NAN;
    g_hcsr04LastSeenMs = 0;
    g_hcsr04LastReadMs = 0;
  } else if (sensor == "vbat") {
    const bool vbatEnabled = g_webServer.hasArg("vbEn");
    if (!vbatEnabled) {
      g_cfg.vbatMode = 0;
    } else {
      const uint32_t mode = parseU32Or(g_webServer.arg("vbMode"), (uint32_t)g_cfg.vbatMode);
      g_cfg.vbatMode = (mode == 2) ? 2 : 1;
    }
    g_cfg.vbatAdcPin = (int)parseU32Or(g_webServer.arg("vbPin"), (uint32_t)g_cfg.vbatAdcPin);
    g_cfg.vbatDividerX1000 = (uint16_t)parseU32Or(g_webServer.arg("vbDiv"), (uint32_t)g_cfg.vbatDividerX1000);
    g_cfg.vbatFullMv = (uint16_t)parseU32Or(g_webServer.arg("vbFull"), (uint32_t)g_cfg.vbatFullMv);
    g_cfg.vbatEmptyMv = (uint16_t)parseU32Or(g_webServer.arg("vbEmp"), (uint32_t)g_cfg.vbatEmptyMv);
    g_cfg.vbatSendVolt = g_webServer.hasArg("vbSV");
    g_vbatVolts = NAN;
    g_vbatPct = -1;
    g_vbatLastSeenMs = 0;
  }

  g_lastMqttLocalSentMs = 0;

  // Save sensor names
  g_prefs.begin("nettemp", false);
  if (sensor == "ds18b20") {
    Serial.printf("Saving DS18B20 config: %u sensors in list\n", (unsigned)g_dsRoms.size());
    for (size_t i = 0; i < g_dsRoms.size(); i++) {
      char romHex[17]{};
      for (int b = 0; b < 8; b++) sprintf(romHex + b * 2, "%02X", g_dsRoms[i][b]);
      const String key = prefKeyDsName(romHex);
      const String argKey = String("dsName_") + romHex;
      Serial.printf("  [%u] ROM=%s, nameKey=%s, argKey=%s\n", (unsigned)i, romHex, key.c_str(), argKey.c_str());
      if (g_webServer.hasArg(argKey)) {
        const String nameVal = g_webServer.arg(argKey);
        if (nameVal.length()) {
          g_prefs.putString(key.c_str(), nameVal);
          Serial.printf("       Saved name: '%s' (len=%u)\n", nameVal.c_str(), nameVal.length());
        } else {
          Serial.printf("       Name empty, keeping previous value\n");
        }
      } else {
        Serial.printf("       No name arg received from form\n");
      }
      // Save selection state
      const String selKey = String("dsSel_") + romHex;
      const bool selected = g_webServer.hasArg(selKey);
      g_prefs.putBool(selKey.c_str(), selected);
      Serial.printf("       Saved selection: %s (selKey=%s)\n", selected ? "checked" : "unchecked", selKey.c_str());
    }
  } else if (sensor == "dht") {
    g_prefs.putString("name_dht", g_webServer.arg("name_dht"));
  } else if (sensor == "soil") {
    g_prefs.putString("name_soil", g_webServer.arg("name_soil"));
  } else if (sensor == "hcsr04") {
    g_prefs.putString("name_hcsr04", g_webServer.arg("name_hcsr04"));
  } else if (sensor == "vbat") {
    g_prefs.putString("name_vbat", g_webServer.arg("name_vbat"));
  }
  g_prefs.end();

  prefsSave();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

static void configHandleSaveOled() {
  if (!webRequireAuth()) return;

  g_cfg.oledEnabled = g_webServer.hasArg("oledEn");
  g_cfg.oledTextSize = (uint8_t)parseU32Or(g_webServer.arg("oledSz"), g_cfg.oledTextSize);
  if (g_cfg.oledTextSize < 1 || g_cfg.oledTextSize > 3) g_cfg.oledTextSize = 1;
  g_prefs.begin("nettemp", false);
  for (const auto& s : g_sensors) {
    const String macNo = macNoColonsUpper(s.mac);
    const String selKey = prefKeyBleOledSel(macNo);
    const String tcKey = prefKeyBleOledTempc(macNo);
    const String humKey = prefKeyBleOledHum(macNo);
    const String voltKey = prefKeyBleOledVolt(macNo);
    const String battKey = prefKeyBleOledBatt(macNo);
    g_prefs.putBool(selKey.c_str(), g_webServer.hasArg(selKey));
    g_prefs.putBool(tcKey.c_str(), g_webServer.hasArg(tcKey));
    g_prefs.putBool(humKey.c_str(), g_webServer.hasArg(humKey));
    g_prefs.putBool(voltKey.c_str(), g_webServer.hasArg(voltKey));
    g_prefs.putBool(battKey.c_str(), g_webServer.hasArg(battKey));
  }
#if NETTEMP_ENABLE_I2C
  for (const auto& s : g_i2cSensors) {
    const String addrHex = (s.address < 16 ? String("0") : String("")) + String(s.address, HEX);
    const String selKey = prefKeyI2cOledSel(addrHex);
    const String tcKey = prefKeyI2cOledTempc(addrHex);
    const String humKey = prefKeyI2cOledHum(addrHex);
    const String pressKey = prefKeyI2cOledPress(addrHex);
    g_prefs.putBool(selKey.c_str(), g_webServer.hasArg(selKey));
    g_prefs.putBool(tcKey.c_str(), g_webServer.hasArg(tcKey));
    g_prefs.putBool(humKey.c_str(), g_webServer.hasArg(humKey));
    g_prefs.putBool(pressKey.c_str(), g_webServer.hasArg(pressKey));
  }
#endif
  g_prefs.putBool(PREF_OLED_DHT, g_webServer.hasArg(PREF_OLED_DHT));
  g_prefs.putBool(PREF_OLED_DHT_T, g_webServer.hasArg(PREF_OLED_DHT_T));
  g_prefs.putBool(PREF_OLED_DHT_F, g_webServer.hasArg(PREF_OLED_DHT_F));
  g_prefs.putBool(PREF_OLED_DHT_H, g_webServer.hasArg(PREF_OLED_DHT_H));
  for (size_t i = 0; i < g_dsRoms.size(); i++) {
    char romHex[17]{};
    for (int b = 0; b < 8; b++) sprintf(romHex + b * 2, "%02X", g_dsRoms[i][b]);
    const String selKey = prefKeyDsOledSel(romHex);
    const String tcKey = prefKeyDsOledTempc(romHex);
    const String tfKey = prefKeyDsOledTempf(romHex);
    g_prefs.putBool(selKey.c_str(), g_webServer.hasArg(selKey));
    g_prefs.putBool(tcKey.c_str(), g_webServer.hasArg(tcKey));
    g_prefs.putBool(tfKey.c_str(), g_webServer.hasArg(tfKey));
  }
  g_prefs.putBool(PREF_OLED_SOIL, g_webServer.hasArg(PREF_OLED_SOIL));
  g_prefs.putBool(PREF_OLED_SOIL_RAW, g_webServer.hasArg(PREF_OLED_SOIL_RAW));
  g_prefs.putBool(PREF_OLED_SOIL_PCT, g_webServer.hasArg(PREF_OLED_SOIL_PCT));
  g_prefs.putBool(PREF_OLED_HC, g_webServer.hasArg(PREF_OLED_HC));
  g_prefs.putBool(PREF_OLED_HC_DIST, g_webServer.hasArg(PREF_OLED_HC_DIST));
  g_prefs.putBool(PREF_OLED_VBAT, g_webServer.hasArg(PREF_OLED_VBAT));
  g_prefs.putBool(PREF_OLED_VBAT_V, g_webServer.hasArg(PREF_OLED_VBAT_V));
  g_prefs.putBool(PREF_OLED_VBAT_B, g_webServer.hasArg(PREF_OLED_VBAT_B));
  g_prefs.end();

  prefsSave();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

static void configHandleSavePower() {
  if (!webRequireAuth()) return;

  g_cfg.powerSleepEnabled = g_webServer.hasArg("pwrSlp");
  g_cfg.powerSleepSeconds = parseU32Or(g_webServer.arg("pwrSlpS"), g_cfg.powerSleepSeconds);
  g_cfg.powerBleScanSeconds = parseU32Or(g_webServer.arg("pwrBleS"), g_cfg.powerBleScanSeconds);
  g_cfg.powerWifiWaitSeconds = parseU32Or(g_webServer.arg("pwrWiS"), g_cfg.powerWifiWaitSeconds);
  g_cfg.powerBootGraceSeconds = parseU32Or(g_webServer.arg("pwrBootS"), g_cfg.powerBootGraceSeconds);
  g_cfg.powerAwakeHoldSeconds = parseU32Or(g_webServer.arg("pwrHoldS"), g_cfg.powerAwakeHoldSeconds);
  if (g_cfg.powerSleepSeconds < 10) g_cfg.powerSleepSeconds = 10;
  if (g_cfg.powerSleepSeconds > 86400) g_cfg.powerSleepSeconds = 86400;
  if (g_cfg.powerBleScanSeconds < 1) g_cfg.powerBleScanSeconds = 1;
  if (g_cfg.powerBleScanSeconds > 30) g_cfg.powerBleScanSeconds = 30;
  if (g_cfg.powerWifiWaitSeconds < 2) g_cfg.powerWifiWaitSeconds = 2;
  if (g_cfg.powerWifiWaitSeconds > 30) g_cfg.powerWifiWaitSeconds = 30;
  if (g_cfg.powerBootGraceSeconds > 600) g_cfg.powerBootGraceSeconds = 600;
  if (g_cfg.powerAwakeHoldSeconds > 120) g_cfg.powerAwakeHoldSeconds = 120;

  g_cfg.vbatMode = (uint8_t)parseU32Or(g_webServer.arg("vbMode"), g_cfg.vbatMode);
  if (g_cfg.vbatMode > 2) g_cfg.vbatMode = 0;
  g_cfg.vbatAdcPin = (int)parseU32Or(g_webServer.arg("vbPin"), (uint32_t)g_cfg.vbatAdcPin);
  g_cfg.vbatDividerX1000 = (uint16_t)parseU32Or(g_webServer.arg("vbDiv"), (uint32_t)g_cfg.vbatDividerX1000);
  g_cfg.vbatFullMv = (uint16_t)parseU32Or(g_webServer.arg("vbFull"), (uint32_t)g_cfg.vbatFullMv);
  g_cfg.vbatEmptyMv = (uint16_t)parseU32Or(g_webServer.arg("vbEmp"), (uint32_t)g_cfg.vbatEmptyMv);
  g_cfg.vbatSendVolt = g_webServer.hasArg("vbSV");
  if (g_cfg.vbatDividerX1000 < 500) g_cfg.vbatDividerX1000 = 500;
  if (g_cfg.vbatDividerX1000 > 5000) g_cfg.vbatDividerX1000 = 5000;
  if (g_cfg.vbatFullMv < 3600) g_cfg.vbatFullMv = 4200;
  if (g_cfg.vbatFullMv > 5000) g_cfg.vbatFullMv = 4200;
  if (g_cfg.vbatEmptyMv < 2500) g_cfg.vbatEmptyMv = 3200;
  if (g_cfg.vbatEmptyMv > 4200) g_cfg.vbatEmptyMv = 3200;
  g_vbatVolts = NAN;
  g_vbatPct = -1;
  g_vbatLastSeenMs = 0;
  g_powerBootCycleDone = false;
  g_powerCycleDoneThisBoot = false;

  g_prefs.begin("nettemp", false);
  g_prefs.putString("name_vbat", g_webServer.arg("name_vbat"));
  g_prefs.end();

  prefsSave();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

static void configHandleSaveAdmin() {
  if (!webRequireAuth()) return;

  const String webUser = g_webServer.arg("webUser");
  if (webUser.length()) g_cfg.webUser = webUser;
  const String webPass = g_webServer.arg("webPass");
  if (webPass.length()) g_cfg.webPass = webPass;

  prefsSave();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

static void configHandleSave() {
  if (!webRequireAuth()) return;

#if NETTEMP_HEADLESS
  g_headlessBleAutoScan = g_webServer.hasArg("bleAuto");
#endif

  // This handler is orphaned - no form uses /save_config anymore
  // All settings are now handled by dedicated handlers:
  // - /save_gpio, /save_mqtt, /save_server, /save_power, /save_admin, etc.
  // Just redirect to home without modifying anything to avoid breaking other settings
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

static void doFactoryReset() {
  g_prefs.begin("nettemp", false);
  g_prefs.clear();
  g_prefs.end();
}

static void configHandleReset() {
  if (!webRequireAuth()) return;
  doFactoryReset();
  g_webServer.send(200, "text/plain", "Reset OK. Rebooting...");
  delay(300);
  ESP.restart();
}

static void configHandleReboot() {
  if (!webRequireAuth()) return;
  g_webServer.send(200, "text/plain", "Rebooting...");
  delay(300);
  ESP.restart();
}

static void configServerStart() {
  if (g_portalRunning && g_webMode == WebMode::StaConfig) return;
  if (!wifiConnected()) return;
  if (g_cfg.webUser.length() == 0 || g_cfg.webPass.length() == 0) return;
  if (g_portalRunning) {
    portalStop();
  }
  g_webServer.close();
  g_webServer.on("/", HTTP_GET, configHandleRoot);
  g_webServer.on("/save_config", HTTP_POST, configHandleSave);
  g_webServer.on("/save_ble", HTTP_POST, configHandleSaveBle);
  g_webServer.on("/save_mqtt", HTTP_POST, configHandleSaveMqtt);
  g_webServer.on("/save_server", HTTP_POST, configHandleSaveServer);
  g_webServer.on("/save_webhook", HTTP_POST, configHandleSaveWebhook);
  g_webServer.on("/send_mqtt", HTTP_POST, configHandleSendMqtt);
  g_webServer.on("/send_server", HTTP_POST, configHandleSendServer);
#if NETTEMP_ENABLE_SERVER
  g_webServer.on("/send_webhook", HTTP_POST, configHandleSendWebhook);
#endif
  g_webServer.on("/ota", HTTP_POST, configHandleOtaFinish, configHandleOtaUpload);
  g_webServer.on("/save_device", HTTP_POST, configHandleSaveDevice);
  g_webServer.on("/save_oled", HTTP_POST, configHandleSaveOled);
  g_webServer.on("/save_gpio", HTTP_POST, configHandleSaveGpio);
  g_webServer.on("/save_power", HTTP_POST, configHandleSavePower);
  g_webServer.on("/save_admin", HTTP_POST, configHandleSaveAdmin);
  g_webServer.on("/save_sensors", HTTP_POST, configHandleSaveSensors);
#if NETTEMP_ENABLE_I2C
  g_webServer.on("/save_i2c", HTTP_POST, configHandleSaveI2c);
  g_webServer.on("/i2c_rescan", HTTP_POST, configHandleI2cRescan);
  g_webServer.on("/i2c_pins", HTTP_POST, configHandleI2cPins);
#endif
  g_webServer.on("/reset", HTTP_POST, configHandleReset);
  g_webServer.on("/reboot", HTTP_POST, configHandleReboot);
  g_webServer.onNotFound([]() {
    g_webServer.sendHeader("Location", "/", true);
    g_webServer.send(302, "text/plain", "");
  });
  g_webServer.begin();
  g_portalRunning = true;
  g_webMode = WebMode::StaConfig;
}

static void webTickMaybeStartConfig() {
  if (g_webMode == WebMode::StaConfig) return;
  if (!wifiConnected()) return;
  configServerStart();
}
#endif // NETTEMP_ENABLE_PORTAL
