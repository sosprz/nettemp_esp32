
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
                "ul{margin:8px 0 0 18px;color:var(--muted)}"
                "select{max-width:100%;padding:8px 10px;border-radius:10px;border:1px solid var(--border);background:#0d1628;color:var(--text)}"
                ".grid2{display:grid;grid-template-columns:1fr 1fr;gap:10px}"
                "@media(max-width:720px){.grid2{grid-template-columns:1fr}}"
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
  body += "<h2><a href='https://nettemp.pl' target='_blank' rel='noopener' style='color:inherit;text-decoration:none'>nettemp.pl</a></h2>";
  body += "<p>Device IP: <code>" + WiFi.localIP().toString() + "</code></p>";
  body += "</div></div>";

  body += "<div class='tabs'>"
          "<button type='button' class='tabbtn' id='btn-tab-status' onclick=\"showTab('tab-status')\">Status</button>"
          "<button type='button' class='tabbtn' id='btn-tab-ble' onclick=\"showTab('tab-ble')\">BLE</button>"
          "<button type='button' class='tabbtn' id='btn-tab-i2c' onclick=\"showTab('tab-i2c')\">I2C</button>"
          "<button type='button' class='tabbtn' id='btn-tab-gpio' onclick=\"showTab('tab-gpio')\">GPIO</button>"
          "<button type='button' class='tabbtn' id='btn-tab-power' onclick=\"showTab('tab-power')\">Power</button>"
          "<button type='button' class='tabbtn' id='btn-tab-mqtt' onclick=\"showTab('tab-mqtt')\">MQTT</button>"
          "<button type='button' class='tabbtn' id='btn-tab-server' onclick=\"showTab('tab-server')\">Server (docker/cloud)</button>"
          "<button type='button' class='tabbtn' id='btn-tab-webhook' onclick=\"showTab('tab-webhook')\">Webhook</button>"
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
        body += "<div class='tablewrap'><table><tr><th>BLE</th>";
        if (showTempc) body += "<th>tempc</th>";
        if (showTempf) body += "<th>tempf</th>";
        if (showHum) body += "<th>hum</th>";
        if (showVolt) body += "<th>volt</th>";
        if (showBatt) body += "<th>batt</th>";
        if (showRssi) body += "<th>rssi</th>";
        body += "</tr>";

        for (size_t k = 0; k < idx.size(); k++) {
          const auto& s = g_sensors[idx[k]];
          if (!s.selected) continue;
          const bool hasValues = !isnan(s.temperatureC) || !isnan(s.humidityPct) || s.batteryPct >= 0 || s.voltageMv >= 0 || s.rssi != 0;
          if (!hasValues) continue;
          body += "<tr>";
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
        body += "<div class='tablewrap' style='margin-top:8px'><table><tr><th>I2C</th>";
        if (showTempc) body += "<th>tempc</th>";
        if (showTempf) body += "<th>tempf</th>";
        if (showHum) body += "<th>hum</th>";
        if (showPress) body += "<th>press_hpa</th>";
        body += "</tr>";

        for (const auto& s : g_i2cSensors) {
          if (!s.selected) continue;
          if (!s.reading.ok) continue;
          const bool hasValues = !isnan(s.reading.temperature_c) || !isnan(s.reading.humidity_pct) || !isnan(s.reading.pressure_hpa);
          if (!hasValues) continue;
          String addrHex = String(s.address, HEX);
          addrHex.toLowerCase();
          if (addrHex.length() == 1) addrHex = "0" + addrHex;
          body += "<tr>";
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
      bool anyGpio = false;
      if (g_cfg.dsEnabled && !g_dsRoms.empty()) {
        for (size_t i = 0; i < g_dsRoms.size(); i++) {
          if (i >= g_dsTempsC.size() || isnan(g_dsTempsC[i])) continue;
          anyGpio = true;
          break;
        }
      }
      if (!anyGpio && g_cfg.dhtEnabled && (!isnan(g_dhtTempC) || !isnan(g_dhtHumPct))) anyGpio = true;
      if (!anyGpio && g_cfg.vbatMode != 0 && (g_vbatPct >= 0 || !isnan(g_vbatVolts))) anyGpio = true;
      if (!anyGpio && g_cfg.soilEnabled && (g_soilRaw >= 0 || !isnan(g_soilPct))) anyGpio = true;

      if (anyGpio) {
        body += "<div class='tablewrap' style='margin-top:8px'><table><tr><th>GPIO</th>";
        if (showTempc) body += "<th>tempc</th>";
        if (showTempf) body += "<th>tempf</th>";
        if (showHum) body += "<th>hum</th>";
        if (showVolt) body += "<th>volt</th>";
        if (showBatt) body += "<th>batt</th>";
        if (showSoilRaw) body += "<th>soil_raw</th>";
        if (showSoilPct) body += "<th>soil_pct</th>";
        body += "</tr>";

        if (g_cfg.dsEnabled && !g_dsRoms.empty()) {
          for (size_t i = 0; i < g_dsRoms.size(); i++) {
            if (i >= g_dsTempsC.size() || isnan(g_dsTempsC[i])) continue;
            char romHex[17]{};
            for (int b = 0; b < 8; b++) sprintf(romHex + b * 2, "%02X", g_dsRoms[i][b]);
            body += "<tr>";
            body += "<td><code>DS18B20 " + String(romHex) + "</code></td>";
            if (showTempc) body += "<td>" + String(g_dsTempsC[i], 2) + "</td>";
            if (showTempf) body += "<td>" + String((g_dsTempsC[i] * 9.0f / 5.0f) + 32.0f, 2) + "</td>";
            if (showHum) body += "<td>-</td>";
            if (showVolt) body += "<td>-</td>";
            if (showBatt) body += "<td>-</td>";
            if (showSoilRaw) body += "<td>-</td>";
            if (showSoilPct) body += "<td>-</td>";
            body += "</tr>";
          }
        }

        if (g_cfg.dhtEnabled && (!isnan(g_dhtTempC) || !isnan(g_dhtHumPct))) {
          body += "<tr>";
          body += "<td><code>DHT" + String(g_cfg.dhtType) + " GPIO" + String(g_cfg.dhtPin) + "</code></td>";
          if (showTempc) body += "<td>" + (isnan(g_dhtTempC) ? String("-") : String(g_dhtTempC, 2)) + "</td>";
          if (showTempf) body += "<td>" + (isnan(g_dhtTempC) ? String("-") : String((g_dhtTempC * 9.0f / 5.0f) + 32.0f, 2)) + "</td>";
          if (showHum) body += "<td>" + (isnan(g_dhtHumPct) ? String("-") : String(g_dhtHumPct, 1)) + "</td>";
          if (showVolt) body += "<td>-</td>";
          if (showBatt) body += "<td>-</td>";
          if (showSoilRaw) body += "<td>-</td>";
          if (showSoilPct) body += "<td>-</td>";
          body += "</tr>";
        }

        if (g_cfg.vbatMode != 0 && (g_vbatPct >= 0 || !isnan(g_vbatVolts))) {
          body += "<tr>";
          body += "<td><code>VBAT</code></td>";
          if (showTempc) body += "<td>-</td>";
          if (showTempf) body += "<td>-</td>";
          if (showHum) body += "<td>-</td>";
          if (showVolt) body += "<td>" + (isnan(g_vbatVolts) ? String("-") : String(g_vbatVolts, 2)) + "</td>";
          if (showBatt) body += "<td>" + (g_vbatPct < 0 ? String("-") : String(g_vbatPct)) + "</td>";
          if (showSoilRaw) body += "<td>-</td>";
          if (showSoilPct) body += "<td>-</td>";
          body += "</tr>";
        }

        if (g_cfg.soilEnabled && (g_soilRaw >= 0 || !isnan(g_soilPct))) {
          body += "<tr>";
          body += "<td><code>SOIL ADC" + String(g_cfg.soilAdcPin) + "</code></td>";
          if (showTempc) body += "<td>-</td>";
          if (showTempf) body += "<td>-</td>";
          if (showHum) body += "<td>-</td>";
          if (showVolt) body += "<td>-</td>";
          if (showBatt) body += "<td>-</td>";
          if (showSoilRaw) body += "<td>" + (g_soilRaw < 0 ? String("-") : String(g_soilRaw)) + "</td>";
          if (showSoilPct) body += "<td>" + (isnan(g_soilPct) ? String("-") : String(g_soilPct, 1)) + "</td>";
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
  body += "<div class='tablewrap'><table><tr><th>Send</th><th>Manual</th><th>MAC</th><th>Age(s)</th><th>tempc</th><th>hum</th><th>batt</th></tr>";
  const auto idx = buildBleSortedIndex();
  const uint32_t nowMs = millis();
  for (size_t k = 0; k < idx.size(); k++) {
    const auto& s = g_sensors[idx[k]];
    const bool hasValues = !isnan(s.temperatureC) || !isnan(s.humidityPct) || s.batteryPct >= 0;
    if (!hasValues) continue;
    const String mac = macWithColonsUpper(s.mac);
    const String macNo = macNoColonsUpper(mac);
    bool manualSelected = false;
    const bool manual = bleManualGetSelected(macNo, manualSelected);
    const bool selected = manual ? manualSelected : s.selected;
    const uint32_t ageS = (s.lastSeenMs > 0 && nowMs >= s.lastSeenMs) ? ((nowMs - s.lastSeenMs) / 1000UL) : 0;

    body += "<tr>";
    body += "<td><input type='checkbox' name='sel_" + macNo + "' value='1' " + String(selected ? "checked" : "") + "></td>";
    body += "<td><input type='checkbox' name='man_" + macNo + "' value='1' " + String(manual ? "checked" : "") + "></td>";
    body += "<td><code>" + mac + "</code></td>";
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
  body += "<p>Supported drivers:</p>";
  body += "<ul>"
          "<li><code>BMP280</code> (0x76/0x77)</li>"
          "<li><code>BME280</code> (0x76/0x77)</li>"
          "<li><code>TMP102</code> (0x48-0x4B)</li>"
          "<li><code>SHT3X</code> (0x44/0x45)</li>"
          "</ul>";

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

  body += "<p style='margin-top:12px'>Detected sensors: <code>" + String((unsigned long)g_i2cSensors.size()) + "</code></p>";
  if (g_i2cSensors.empty()) {
    body += "<p>No supported I2C sensors detected.</p>";
  } else {
    const uint32_t nowMs = millis();
    body += "<form method='POST' action='/save_i2c'>";
    body += "<div class='tablewrap'><table><tr><th>Send</th><th>Type</th><th>Addr</th><th>Age(s)</th><th>tempc</th><th>hum</th><th>press_hpa</th></tr>";
    for (const auto& s : g_i2cSensors) {
      const String addrHex = (s.address < 16 ? String("0") : String("")) + String(s.address, HEX);
      const String key = "i2c_" + addrHex;
      const uint32_t ageS = (s.last_seen_ms > 0 && nowMs >= s.last_seen_ms) ? ((nowMs - s.last_seen_ms) / 1000UL) : 0;
      body += "<tr>";
      body += "<td><input type='checkbox' name='" + key + "' value='1' " + String(s.selected ? "checked" : "") + "></td>";
      body += "<td><code>" + String(i2cSensorTypeName(s.type)) + "</code></td>";
      body += "<td><code>0x" + addrHex + "</code></td>";
      body += "<td>" + String((unsigned long)ageS) + "</td>";
      body += "<td>" + (s.reading.ok && !isnan(s.reading.temperature_c) ? String(s.reading.temperature_c, 2) : String("-")) + "</td>";
      body += "<td>" + (s.reading.ok && !isnan(s.reading.humidity_pct) ? String(s.reading.humidity_pct, 1) : String("-")) + "</td>";
      body += "<td>" + (s.reading.ok && !isnan(s.reading.pressure_hpa) ? String(s.reading.pressure_hpa, 1) : String("-")) + "</td>";
      body += "</tr>";
    }
    body += "</table></div>";
    body += "<p style='margin:12px 0 0'><button type='submit'>Save</button></p>";
    body += "</form>";
    body += "<form method='POST' action='/i2c_rescan' style='margin-top:10px' onsubmit=\"return confirm('Rescan I2C bus?');\">"
            "<button type='submit'>Rescan</button>"
            "</form>";
  }
#else
  body += "<p>I2C support is disabled in this build (<code>NETTEMP_ENABLE_I2C=0</code>).</p>";
#endif
  body += "</div>";

  body += "<div class='card tab hidden' id='tab-gpio'>";
  body += "<h3>GPIO sensors</h3>";
  body += "<form method='POST' action='/save_gpio'>";
  body += "<label><input type='checkbox' name='dsEn' value='1' " + String(g_cfg.dsEnabled ? "checked" : "") + "> Enable DS18B20 (1-Wire)</label>";
  body += "<label>DS18B20 GPIO</label><input name='dsPin' value='" + String(g_cfg.dsPin) + "'/>";
  body += "<p>Detected DS18B20: <code>" + String((unsigned long)g_dsRoms.size()) + "</code></p>";
  if (!g_dsRoms.empty()) {
    body += "<div class='tablewrap'><table><tr><th>#</th><th>ROM</th><th>tempc</th></tr>";
    for (size_t i = 0; i < g_dsRoms.size(); i++) {
      char romHex[17]{};
      for (int b = 0; b < 8; b++) sprintf(romHex + b * 2, "%02X", g_dsRoms[i][b]);
      body += "<tr><td>" + String((unsigned long)i) + "</td><td><code>" + String(romHex) + "</code></td><td>";
      body += (i < g_dsTempsC.size() && !isnan(g_dsTempsC[i]) ? String(g_dsTempsC[i], 2) : String("-"));
      body += "</td></tr>";
    }
    body += "</table></div>";
  }
  body += "<label style='margin-top:14px'><input type='checkbox' name='dhtEn' value='1' " + String(g_cfg.dhtEnabled ? "checked" : "") + "> Enable DHT</label>";
  body += "<div class='grid2'><div><label>DHT GPIO</label><input name='dhtPin' value='" + String(g_cfg.dhtPin) + "'/></div>";
  body += "<div><label>DHT type</label><input name='dhtType' value='" + String(g_cfg.dhtType) + "' placeholder='11 or 22'/></div></div>";
  body += "<p>Last DHT: tempc=<code>" + (isnan(g_dhtTempC) ? String("-") : String(g_dhtTempC, 2)) + "</code> hum=<code>" +
          (isnan(g_dhtHumPct) ? String("-") : String(g_dhtHumPct, 1)) + "</code></p>";
  body += "<label style='margin-top:14px'><input type='checkbox' name='soilEn' value='1' " + String(g_cfg.soilEnabled ? "checked" : "") + "> Enable capacitive soil moisture sensor v1.2 (ADC)</label>";
  body += "<label>Soil ADC pin</label><input name='soilPin' value='" + String(g_cfg.soilAdcPin) + "'/>";
  body += "<div class='grid2'><div><label>Dry raw (0%)</label><input name='soilDry' value='" + String(g_cfg.soilDryRaw) + "'/></div>";
  body += "<div><label>Wet raw (100%)</label><input name='soilWet' value='" + String(g_cfg.soilWetRaw) + "'/></div></div>";
  body += "<p>Last soil: raw=<code>" + (g_soilRaw < 0 ? String("-") : String(g_soilRaw)) + "</code> pct=<code>" +
          (isnan(g_soilPct) ? String("-") : String(g_soilPct, 1)) + "</code></p>";
  body += "<p style='margin:12px 0 0'><button type='submit'>Save</button></p>";
  body += "</form>";
  body += "</div>";

  body += "<div class='card tab hidden' id='tab-power'>";
  body += "<h3>Power</h3>";
  body += "<form method='POST' action='/save_power'>";
  body += "<label><input type='checkbox' name='pwrSlp' value='1' " + String(g_cfg.powerSleepEnabled ? "checked" : "") + "> Enable deep sleep duty-cycle</label>";
  body += "<label>Boot grace (seconds; power-on/reset)</label><input name='pwrBootS' value='" + String((unsigned long)g_cfg.powerBootGraceSeconds) + "' placeholder='30 (0=never sleep on boot)'/>";
  body += "<label>Awake hold (seconds; after cycle)</label><input name='pwrHoldS' value='" + String((unsigned long)g_cfg.powerAwakeHoldSeconds) + "' placeholder='5 (0=immediate sleep)'/>";
  body += "<label>Sleep interval (seconds)</label><input name='pwrSlpS' value='" + String((unsigned long)g_cfg.powerSleepSeconds) + "'/>";
  body += "<label>BLE scan per cycle (seconds)</label><input name='pwrBleS' value='" + String((unsigned long)g_cfg.powerBleScanSeconds) + "'/>";
  body += "<label>WiFi wait per cycle (seconds)</label><input name='pwrWiS' value='" + String((unsigned long)g_cfg.powerWifiWaitSeconds) + "'/>";
  body += "<h3 style='margin-top:16px'>Battery (VBAT)</h3>";
  body += "<label>Mode</label><select name='vbMode'>"
          "<option value='0' " + String(g_cfg.vbatMode == 0 ? "selected" : "") + ">off</option>"
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
  body += "<p style='margin-top:10px'>Note: on power-on/reset, the device stays awake for <code>boot_grace_s</code>, then performs one cycle and sleeps (unless blocked by Serial). Timer wakeups duty-cycle normally.</p>";
  body += "<p style='margin:12px 0 0'><button type='submit'>Save</button></p>";
  body += "</form>";
  body += "</div>";

  body += "<div class='card tab hidden' id='tab-mqtt'>";
  body += "<h3>MQTT</h3>";
  body += "<form method='POST' action='/save_mqtt'>";
  body += "<label><input type='checkbox' name='mqttEn' value='1' " + String(g_cfg.mqttEnabled ? "checked" : "") + "> Enable MQTT</label>";
  body += "<div class='grid2'><div><label>Host</label><input name='mqttHost' value='" + htmlEscape(g_cfg.mqttHost) + "'/></div>";
  body += "<div><label>Port</label><input name='mqttPort' value='" + String((unsigned)g_cfg.mqttPort) + "'/></div></div>";
  body += "<div class='grid2'><div><label>Username</label><input name='mqttUser' value='" + htmlEscape(g_cfg.mqttUser) + "'/></div>";
  body += "<div><label>Password</label><input name='mqttPass' type='password' value='' placeholder='(leave empty to keep existing)'/></div></div>";
  body += "<label>Interval (seconds)</label><input name='mqttInt' value='" + String((unsigned long)(g_cfg.mqttIntervalMs / 1000UL)) + "'/>";
  body += "<label style='margin-top:12px'>Sensors</label>";
  body += "<label><input type='checkbox' name='mqttBle' value='1' " + String(g_cfg.bleSendMqtt ? "checked" : "") + "> BLE</label>";
  body += "<label><input type='checkbox' name='mqttGpio' value='1' " + String(g_cfg.gpioSendMqtt ? "checked" : "") + "> GPIO</label>";
  body += "<label><input type='checkbox' name='mqttI2c' value='1' " + String(g_cfg.i2cSendMqtt ? "checked" : "") + "> I2C</label>";
  body += "<p style='margin:12px 0 0'><button type='submit'>Save</button></p>";
  body += "</form>";
  body += "</div>";

  body += "<div class='card tab hidden' id='tab-server'>";
  body += "<h3>Server</h3>";
  body += "<form method='POST' action='/save_server'>";
  body += "<label><input type='checkbox' name='srvEn' value='1' " + String(g_cfg.serverEnabled ? "checked" : "") + "> Enable Server</label>";
  body += "<label>Base URL</label><input name='srvUrl' value='" + htmlEscape(g_cfg.serverBaseUrl) + "'/>";
  body += "<label>API key (ntk_...)</label><input name='srvKey' type='password' value='' placeholder='(leave empty to keep existing)'/>";
  body += "<label>Interval (seconds)</label><input name='srvInt' value='" + String((unsigned long)(g_cfg.serverIntervalMs / 1000UL)) + "'/>";
  body += "<label style='margin-top:12px'>Sensors</label>";
  body += "<label><input type='checkbox' name='srvBle' value='1' " + String(g_cfg.bleSendServer ? "checked" : "") + "> BLE</label>";
  body += "<label><input type='checkbox' name='srvGpio' value='1' " + String(g_cfg.gpioSendServer ? "checked" : "") + "> GPIO</label>";
  body += "<label><input type='checkbox' name='srvI2c' value='1' " + String(g_cfg.i2cSendServer ? "checked" : "") + "> I2C</label>";
  body += "<label>Server JSON fields (BLE)</label>";
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

  body += "<div class='card tab hidden' id='tab-webhook'>";
  body += "<h3>Webhook (JSON)</h3>";
  body += "<form method='POST' action='/save_server'>";
  body += "<label><input type='checkbox' name='whEn' value='1' " + String(g_cfg.webhookEnabled ? "checked" : "") + "> Enable webhook</label>";
  body += "<label>Webhook URL</label><input name='whUrl' value='" + htmlEscape(g_cfg.webhookUrl) + "' placeholder='https://example.com/hook'/>";
  body += "<label>Interval (seconds)</label><input name='whInt' value='" + String((unsigned long)(g_cfg.webhookIntervalMs / 1000UL)) + "'/>";
  body += "<label style='margin-top:12px'>Sensors</label>";
  body += "<label><input type='checkbox' name='whBle' value='1' " + String(g_cfg.bleSendWebhook ? "checked" : "") + "> BLE</label>";
  body += "<label><input type='checkbox' name='whGpio' value='1' " + String(g_cfg.gpioSendWebhook ? "checked" : "") + "> GPIO</label>";
  body += "<label><input type='checkbox' name='whI2c' value='1' " + String(g_cfg.i2cSendWebhook ? "checked" : "") + "> I2C</label>";
  body += "<p style='margin:12px 0 0'><button type='submit'>Save</button></p>";
  body += "</form>";
  body += "</div>";

  body += "<div class='card tab hidden' id='tab-device'>";
  body += "<h3>Device</h3>";
  body += "<form method='POST' action='/save_device'>";
  body += "<label>Device ID (I2C/GPIO + sensor_id prefix)</label><input name='deviceId' value='" + htmlEscape(g_cfg.deviceId) + "'/>";
  body += "<p style='margin:12px 0 0'><button type='submit'>Save</button></p>";
  body += "</form>";
  body += "<h3 style='margin-top:16px'>Firmware update</h3>";
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
  body += "<p>WiFi: <code>" + String(wifiConnected() ? "connected" : "not connected") + "</code>";
#if NETTEMP_HEADLESS
  body += " | BLE autoscan: <code>" + String(g_headlessBleAutoScan ? "on" : "off") + "</code>";
#else
  body += " | BLE scan: <code>on</code> (" + String(g_activeScan ? "active" : "passive") + ")";
#endif
  body += "</p>";
  body += "<p>MQTT: <code>" + String(g_cfg.mqttEnabled ? "on" : "off") + "</code> | Server: <code>" + String(g_cfg.serverEnabled ? "on" : "off") + "</code></p>";
  body += "<p>Server JSON: <code>" + serverBleFieldsEnabledString() + "</code> (available <code>" + serverBleFieldsAvailableString() + "</code>)</p>";
#if NETTEMP_ENABLE_I2C
  body += "<p>Server JSON (I2C): <code>" + serverI2cFieldsEnabledString() + "</code> (available <code>" + serverI2cFieldsAvailableString() + "</code>)</p>";
#endif
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
  g_webServer.send(200, "text/html; charset=utf-8", body);
}

static void configHandleSaveSensors() {
  if (!webRequireAuth()) return;
  // Apply per-device manual overrides (or clear them).
  for (const auto& s : g_sensors) {
    const bool hasValues = !isnan(s.temperatureC) || !isnan(s.humidityPct) || s.batteryPct >= 0;
    if (!hasValues) continue;
    const String macNo = macNoColonsUpper(s.mac);
    const String selKey = "sel_" + macNo;
    const String manKey = "man_" + macNo;
    if (g_webServer.hasArg(manKey)) {
      const bool selected = g_webServer.hasArg(selKey);
      bleManualSet(macNo, selected);
    } else {
      bleManualClear(macNo);
    }
  }
  prefsSave();
  g_webServer.sendHeader("Location", "/", true);
  g_webServer.send(302, "text/plain", "");
}

#if NETTEMP_ENABLE_I2C
static void configHandleSaveI2c() {
  if (!webRequireAuth()) return;
  g_i2cSelDefined = true;
  g_i2cSelAddrs.clear();

  for (auto& s : g_i2cSensors) {
    const String addrHex = (s.address < 16 ? String("0") : String("")) + String(s.address, HEX);
    const String key = "i2c_" + addrHex;
    const bool selected = g_webServer.hasArg(key);
    s.selected = selected;
    if (selected) g_i2cSelAddrs.push_back(s.address);
  }

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
  g_i2cSensors = i2cDetectKnownSensors(Wire);
  if (!g_i2cSensors.empty()) i2cUpdateReadings(Wire, g_i2cSensors);
  i2cApplySelectionToDetected();

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
  g_i2cSensors = i2cDetectKnownSensors(Wire);
  if (!g_i2cSensors.empty()) i2cUpdateReadings(Wire, g_i2cSensors);
  if (g_i2cSelDefined) i2cApplySelectionToDetected();

  prefsSave();
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
  g_cfg.bleSendMqtt = g_webServer.hasArg("mqttBle");
  g_cfg.gpioSendMqtt = g_webServer.hasArg("mqttGpio");
  g_cfg.i2cSendMqtt = g_webServer.hasArg("mqttI2c");

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
  g_cfg.webhookEnabled = g_webServer.hasArg("whEn");
  const String whUrl = g_webServer.arg("whUrl");
  if (whUrl.length()) g_cfg.webhookUrl = whUrl;
  const uint32_t whIntS = parseU32Or(g_webServer.arg("whInt"), (g_cfg.webhookIntervalMs / 1000UL));
  if (whIntS >= 5 && whIntS <= 3600) g_cfg.webhookIntervalMs = whIntS * 1000UL;
  g_cfg.bleSendWebhook = g_webServer.hasArg("whBle");
  g_cfg.gpioSendWebhook = g_webServer.hasArg("whGpio");
  g_cfg.i2cSendWebhook = g_webServer.hasArg("whI2c");
  g_cfg.soilSendRaw = g_webServer.hasArg("f_soil_raw");
  g_cfg.soilSendPct = g_webServer.hasArg("f_soil_pct");
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

  g_cfg.dsEnabled = g_webServer.hasArg("dsEn");
  g_cfg.dhtEnabled = g_webServer.hasArg("dhtEn");
  g_cfg.dsPin = (int)parseU32Or(g_webServer.arg("dsPin"), (uint32_t)g_cfg.dsPin);
  g_cfg.dhtPin = (int)parseU32Or(g_webServer.arg("dhtPin"), (uint32_t)g_cfg.dhtPin);
  g_cfg.dhtType = (int)parseU32Or(g_webServer.arg("dhtType"), (uint32_t)g_cfg.dhtType);
  g_cfg.soilEnabled = g_webServer.hasArg("soilEn");
  g_cfg.soilAdcPin = (int)parseU32Or(g_webServer.arg("soilPin"), (uint32_t)g_cfg.soilAdcPin);
  g_cfg.soilDryRaw = (int)parseU32Or(g_webServer.arg("soilDry"), (uint32_t)g_cfg.soilDryRaw);
  g_cfg.soilWetRaw = (int)parseU32Or(g_webServer.arg("soilWet"), (uint32_t)g_cfg.soilWetRaw);
  if (g_cfg.dhtType != 11 && g_cfg.dhtType != 22) g_cfg.dhtType = 22;
  g_lastMqttLocalSentMs = 0;
  g_dhtTempC = NAN;
  g_dhtHumPct = NAN;
  g_dhtLastSeenMs = 0;
  g_dhtLastReadMs = 0;
  dsEnsureBus();
  if (g_cfg.dsEnabled) dsRescan();

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

  g_cfg.mqttEnabled = g_webServer.hasArg("mqttEn");
  const String mqttHost = g_webServer.arg("mqttHost");
  if (mqttHost.length()) g_cfg.mqttHost = mqttHost;
  const uint32_t mqttPort = parseU32Or(g_webServer.arg("mqttPort"), g_cfg.mqttPort);
  if (mqttPort > 0 && mqttPort <= 65535) g_cfg.mqttPort = (uint16_t)mqttPort;
  const String mqttUser = g_webServer.arg("mqttUser");
  g_cfg.mqttUser = mqttUser;
  const String mqttPass = g_webServer.arg("mqttPass");
  if (mqttPass.length()) g_cfg.mqttPass = mqttPass;
  const uint32_t mqttIntS = parseU32Or(g_webServer.arg("mqttInt"), (g_cfg.mqttIntervalMs / 1000UL));
  if (mqttIntS >= 5 && mqttIntS <= 3600) g_cfg.mqttIntervalMs = mqttIntS * 1000UL;

  g_cfg.serverEnabled = g_webServer.hasArg("srvEn");
  const String deviceId = g_webServer.arg("deviceId");
  if (deviceId.length()) g_cfg.deviceId = deviceId;
  const String srvUrl = g_webServer.arg("srvUrl");
  if (srvUrl.length()) g_cfg.serverBaseUrl = srvUrl;
  const String srvKey = g_webServer.arg("srvKey");
  if (srvKey.length()) g_cfg.serverApiKey = srvKey;
  const uint32_t srvIntS = parseU32Or(g_webServer.arg("srvInt"), (g_cfg.serverIntervalMs / 1000UL));
  if (srvIntS >= 5 && srvIntS <= 3600) g_cfg.serverIntervalMs = srvIntS * 1000UL;
  g_cfg.bleSendServer = g_webServer.hasArg("srvBle");
  g_cfg.gpioSendServer = g_webServer.hasArg("srvGpio");
  g_cfg.i2cSendServer = g_webServer.hasArg("srvI2c");
  g_cfg.webhookEnabled = g_webServer.hasArg("whEn");
  const String whUrl = g_webServer.arg("whUrl");
  if (whUrl.length()) g_cfg.webhookUrl = whUrl;
  const uint32_t whIntS = parseU32Or(g_webServer.arg("whInt"), (g_cfg.webhookIntervalMs / 1000UL));
  if (whIntS >= 5 && whIntS <= 3600) g_cfg.webhookIntervalMs = whIntS * 1000UL;
  g_cfg.bleSendWebhook = g_webServer.hasArg("whBle");
  g_cfg.gpioSendWebhook = g_webServer.hasArg("whGpio");
  g_cfg.i2cSendWebhook = g_webServer.hasArg("whI2c");
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

  g_cfg.dsEnabled = g_webServer.hasArg("dsEn");
  g_cfg.dhtEnabled = g_webServer.hasArg("dhtEn");
  g_cfg.dsPin = (int)parseU32Or(g_webServer.arg("dsPin"), (uint32_t)g_cfg.dsPin);
  g_cfg.dhtPin = (int)parseU32Or(g_webServer.arg("dhtPin"), (uint32_t)g_cfg.dhtPin);
  g_cfg.dhtType = (int)parseU32Or(g_webServer.arg("dhtType"), (uint32_t)g_cfg.dhtType);
  g_cfg.soilEnabled = g_webServer.hasArg("soilEn");
  g_cfg.soilAdcPin = (int)parseU32Or(g_webServer.arg("soilPin"), (uint32_t)g_cfg.soilAdcPin);
  g_cfg.soilDryRaw = (int)parseU32Or(g_webServer.arg("soilDry"), (uint32_t)g_cfg.soilDryRaw);
  g_cfg.soilWetRaw = (int)parseU32Or(g_webServer.arg("soilWet"), (uint32_t)g_cfg.soilWetRaw);
  if (g_cfg.dhtType != 11 && g_cfg.dhtType != 22) g_cfg.dhtType = 22;
  g_lastMqttLocalSentMs = 0;
  g_dhtTempC = NAN;
  g_dhtHumPct = NAN;
  g_dhtLastSeenMs = 0;
  g_dhtLastReadMs = 0;
  dsEnsureBus();
	  if (g_cfg.dsEnabled) dsRescan();

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

  const String webUser = g_webServer.arg("webUser");
  if (webUser.length()) g_cfg.webUser = webUser;
  const String webPass = g_webServer.arg("webPass");
  if (webPass.length()) g_cfg.webPass = webPass;

  prefsSave();
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
  g_webServer.on("/ota", HTTP_POST, configHandleOtaFinish, configHandleOtaUpload);
  g_webServer.on("/save_device", HTTP_POST, configHandleSaveDevice);
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
