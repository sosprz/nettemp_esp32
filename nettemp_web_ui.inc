struct WebPrefsView {
  Preferences* prefs;

  String get(const String& key) const {
    return prefs->getString(key.c_str(), "");
  }

  bool getBool(const String& key, bool defaultVal = false) const {
    return prefs->getBool(key.c_str(), defaultVal);
  }

  String getName(const String& key, const String& fallback) const {
    const String v = get(key);
    return v.length() ? v : fallback;
  }

  String getDsName(const String& romHex, const String& fallback) const {
    String v = get(prefKeyDsName(romHex));
    if (!v.length()) v = get(prefKeyDsNameLegacy(romHex));
    return v.length() ? v : fallback;
  }
};

static String buildConfigPage(WebPrefsView& prefs) {
  auto prefGetBool = [&](const String& key, bool defaultVal = false) -> bool {
    return prefs.getBool(key, defaultVal);
  };
  auto prefGetName = [&](const String& key, const String& fallback) -> String {
    return prefs.getName(key, fallback);
  };
  auto prefGetDsName = [&](const String& romHex, const String& fallback) -> String {
    return prefs.getDsName(romHex, fallback);
  };
  auto sortByNameThenId = [&](const String& nameA, const String& idA, const String& nameB, const String& idB) -> bool {
    if (nameA != nameB) return nameA < nameB;
    return idA < idB;
  };

  String body;
  body.reserve(60000);
  body += "<!doctype html><html><head><meta charset='utf-8'/>"
                "<meta name='viewport' content='width=device-width,initial-scale=1'/>"
                "<title>nettemp.pl</title>"
                "<style>"
                "*,*:before,*:after{box-sizing:border-box}"
                ":root{--bg:#0b1220;--card:#111b2e;--muted:#9fb0cc;--text:#e8eefc;--accent:#2f6bff;--border:#243251}"
                "body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial;margin:0;background:var(--bg);color:var(--text)}"
                ".wrap{max-width:980px;margin:0 auto;padding:16px}"
                ".top{display:flex;align-items:center;gap:10px;margin:6px 0 14px}"
                "h2{margin:0;font-size:20px} h3{margin:0 0 10px}"
                "p{color:var(--muted);line-height:1.35}"
                ".tabs{display:flex;gap:8px;flex-wrap:wrap;margin:0 0 12px;position:relative;z-index:2}"
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
                "let tab=document.getElementById(id);"
                "if(!tab){id='tab-status';tab=document.getElementById(id);}"
                "const btn=document.getElementById('btn-'+id);"
                "if(!tab) return;"
                "document.querySelectorAll('.tab').forEach(el=>el.classList.add('hidden'));"
                "document.querySelectorAll('.tabbtn').forEach(el=>el.classList.remove('active'));"
                "tab.classList.remove('hidden');"
                "if(btn) btn.classList.add('active');"
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
                "showTab(id);"
                "});"
                "</script>"
                "</head><body><div class='wrap'>";

  body += "<div class='top'><div>";
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

  auto renderTabStatus = [&]() {
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
    auto bleDisplayScore = [&](const SensorRow& s) {
      int score = 0;
      if (showTempc && !isnan(s.temperatureC)) score++;
      if (showTempf && !isnan(s.temperatureC)) score++;
      if (showHum && !isnan(s.humidityPct)) score++;
      if (showVolt && s.voltageMv >= 0) score++;
      if (showBatt && s.batteryPct >= 0) score++;
      if (showRssi && s.rssi != 0) score++;
      return score;
    };
    body += "<h4 style='margin:8px 0 6px'>Sensors</h4>";

    bool anyStatusTable = false;
    {
      const auto idx = buildBleSortedIndex();
      std::vector<size_t> bleIdx;
      std::vector<String> bleMacs;
      bleIdx.reserve(idx.size());
      bleMacs.reserve(idx.size());
      for (size_t k = 0; k < idx.size(); k++) {
        const size_t sensorIdx = idx[k];
        const auto& s = g_sensors[sensorIdx];
        if (!bleIsSelected(s)) continue;
        const int score = bleDisplayScore(s);
        if (score == 0) continue;
        const String macNo = macNoColonsUpper(s.mac);
        bool found = false;
        for (size_t i = 0; i < bleMacs.size(); i++) {
          if (bleMacs[i] == macNo) {
            const int existingScore = bleDisplayScore(g_sensors[bleIdx[i]]);
            if (score > existingScore) {
              bleIdx[i] = sensorIdx;
            }
            found = true;
            break;
          }
        }
        if (!found) {
          bleMacs.push_back(macNo);
          bleIdx.push_back(sensorIdx);
        }
      }

      if (!bleIdx.empty()) {
        body += "<div class='tablewrap'><table><tr><th>Name</th><th>BLE</th>";
        if (showTempc) body += "<th>tempc</th>";
        if (showTempf) body += "<th>tempf</th>";
        if (showHum) body += "<th>hum</th>";
        if (showVolt) body += "<th>volt</th>";
        if (showBatt) body += "<th>batt</th>";
        if (showRssi) body += "<th>rssi</th>";
        body += "</tr>";

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
  };
  renderTabStatus();

  auto renderTabBle = [&]() {
  body += "<div class='card tab hidden' id='tab-ble'>";
  body += "<h3>BLE</h3>";
  body += "<form method='POST' action='/save_ble'>";
  body += "<label><input type='checkbox' name='bleAuto' value='1' " + String(g_bleAutoScan ? "checked" : "") + "> Enable autoscan</label>";
  body += "<label><input type='checkbox' name='bleActive' value='1' " + String(g_activeScan ? "checked" : "") + "> Active scan (more power, more data)</label>";
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
  };
  renderTabBle();

  auto renderTabI2c = [&]() {
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
    body += "<details><summary style='cursor:pointer;color:var(--muted);font-size:0.9em'>🔍 I2C scan details</summary>";
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
  };
  renderTabI2c();

  auto renderTabGpio = [&]() {
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
  };
  renderTabGpio();

  auto renderTabOled = [&]() {
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
    bool manualSelected = false;
    const bool manual = bleManualGetSelected(macNo, manualSelected);
    const bool selected = manual ? manualSelected : s.selected;
    if (!selected) continue;
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
  };
  renderTabOled();

  auto renderTabChannels = [&]() {
  body += "<div class='card tab hidden' id='tab-channels'>";
  body += "<h3>Channels</h3>";
  body += "<div class='tile-grid'>";
  body += "<div class='tile'>";
  body += "<h4 style='margin:0 0 4px'>MQTT <span style='color:var(--muted);font-size:12px;font-weight:400'>(" + htmlEscape(formatSendStatus(g_lastMqttLocalSentMs, g_cfg.mqttIntervalMs)) + ")</span></h4>";
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
  body += "<details><summary style='cursor:pointer;color:var(--muted);font-size:0.9em'>Show JSON fields</summary>";
  body += "<label style='margin-top:12px'>MQTT JSON fields (BLE)</label>";
  appendBleFieldCheckboxes(body, "m_f_", g_mqttBleFields);
#if NETTEMP_ENABLE_I2C
  body += "<label style='margin-top:12px'>MQTT JSON fields (I2C)</label>";
  appendI2cFieldCheckboxes(body, "m_f_i2c_", g_mqttI2cFields);
#endif
  body += "<label style='margin-top:12px'>MQTT JSON fields (GPIO)</label>";
  appendGpioFieldCheckboxes(body, "m_f_gpio_", g_mqttGpioFields);
  body += "</details>";
  body += "<p style='margin:12px 0 0;display:flex;gap:8px;align-items:center'>";
  body += "<button type='submit'>Save</button>";
  body += "<button type='button' id='sendMqtt'>Send now</button>";
  body += "<span id='statusMqtt' style='color:var(--muted);font-size:0.9em'></span>";
  body += "</p>";
  body += "</form>";
  body += "</div>";

  body += "<div class='tile'>";
  body += "<h4 style='margin:0 0 4px'>Cloud server <span style='color:var(--muted);font-size:12px;font-weight:400'>(" + htmlEscape(formatSendStatus(g_lastServerSendMs, g_cfg.serverIntervalMs)) + ")</span></h4>";
  body += "<form method='POST' action='/save_server'>";
  body += "<label><input type='checkbox' name='srvEn' value='1' " + String(g_cfg.serverEnabled ? "checked" : "") + "> Enable cloud server</label>";
  body += "<label>Cloud base URL</label><input name='srvUrl' value='" + htmlEscape(g_cfg.serverBaseUrl) + "'/>";
  body += "<label>API key (ntk_...)</label><input name='srvKey' type='password' value='' placeholder='(leave empty to keep existing)'/>";
  body += "<label>Interval (seconds)</label><input name='srvInt' value='" + String((unsigned long)(g_cfg.serverIntervalMs / 1000UL)) + "'/>";
  body += "<label style='margin-top:12px'>Send data from</label>";
  body += "<div style='display:flex;gap:12px;flex-wrap:wrap'>";
  body += "<label><input type='checkbox' name='srvBle' value='1' " + String(g_cfg.bleSendServer ? "checked" : "") + "> BLE</label>";
  body += "<label><input type='checkbox' name='srvGpio' value='1' " + String(g_cfg.gpioSendServer ? "checked" : "") + "> GPIO</label>";
  body += "<label><input type='checkbox' name='srvI2c' value='1' " + String(g_cfg.i2cSendServer ? "checked" : "") + "> I2C</label>";
  body += "</div>";
  body += "<details><summary style='cursor:pointer;color:var(--muted);font-size:0.9em'>Show JSON fields</summary>";
  body += "<label style='margin-top:12px'>Cloud JSON fields (BLE)</label>";
  appendBleFieldCheckboxes(body, "f_", g_srvBleFields);
#if NETTEMP_ENABLE_I2C
  body += "<label style='margin-top:12px'>Cloud JSON fields (I2C)</label>";
  appendI2cFieldCheckboxes(body, "f_i2c_", g_srvI2cFields);
#endif
  body += "<label style='margin-top:12px'>Cloud JSON fields (GPIO)</label>";
  appendGpioFieldCheckboxes(body, "f_gpio_", g_srvGpioFields);
  body += "</details>";
  body += "<p style='margin:12px 0 0;display:flex;gap:8px;align-items:center'>";
  body += "<button type='submit'>Save</button>";
  body += "<button type='button' id='sendServer'>Send now</button>";
  body += "<span id='statusServer' style='color:var(--muted);font-size:0.9em'></span>";
  body += "</p>";
  body += "</form>";
  body += "</div>";

  body += "<div class='tile'>";
  body += "<h4 style='margin:0 0 4px'>Local server <span style='color:var(--muted);font-size:12px;font-weight:400'>(" + htmlEscape(formatSendStatus(g_lastLocalServerSendMs, g_cfg.localServerIntervalMs)) + ")</span></h4>";
  body += "<form method='POST' action='/save_local_server'>";
  body += "<label><input type='checkbox' name='lsEn' value='1' " + String(g_cfg.localServerEnabled ? "checked" : "") + "> Enable local server</label>";
  body += "<label>Local URL (full path)</label><input name='lsUrl' value='" + htmlEscape(g_cfg.localServerUrl) + "' placeholder='http://192.168.1.10/api/v1/data'/>";
  body += "<label>API key (optional)</label><input name='lsKey' type='password' value='' placeholder='(leave empty for no auth)'/>";
  body += "<label>Interval (seconds)</label><input name='lsInt' value='" + String((unsigned long)(g_cfg.localServerIntervalMs / 1000UL)) + "'/>";
  body += "<label style='margin-top:12px'>Send data from</label>";
  body += "<div style='display:flex;gap:12px;flex-wrap:wrap'>";
  body += "<label><input type='checkbox' name='lsBle' value='1' " + String(g_cfg.bleSendLocalServer ? "checked" : "") + "> BLE</label>";
  body += "<label><input type='checkbox' name='lsGpio' value='1' " + String(g_cfg.gpioSendLocalServer ? "checked" : "") + "> GPIO</label>";
  body += "<label><input type='checkbox' name='lsI2c' value='1' " + String(g_cfg.i2cSendLocalServer ? "checked" : "") + "> I2C</label>";
  body += "</div>";
  body += "<details><summary style='cursor:pointer;color:var(--muted);font-size:0.9em'>Show JSON fields</summary>";
  body += "<label style='margin-top:12px'>Local JSON fields (BLE)</label>";
  appendBleFieldCheckboxes(body, "ls_f_", g_localBleFields);
#if NETTEMP_ENABLE_I2C
  body += "<label style='margin-top:12px'>Local JSON fields (I2C)</label>";
  appendI2cFieldCheckboxes(body, "ls_f_i2c_", g_localI2cFields);
#endif
  body += "<label style='margin-top:12px'>Local JSON fields (GPIO)</label>";
  appendGpioFieldCheckboxes(body, "ls_f_gpio_", g_localGpioFields);
  body += "</details>";
  body += "<p style='margin:12px 0 0;display:flex;gap:8px;align-items:center'>";
  body += "<button type='submit'>Save</button>";
  body += "<button type='button' id='sendLocalServer'>Send now</button>";
  body += "<span id='statusLocalServer' style='color:var(--muted);font-size:0.9em'></span>";
  body += "</p>";
  body += "</form>";
  body += "</div>";

  body += "<div class='tile'>";
  body += "<h4 style='margin:0 0 4px'>Webhook <span style='color:var(--muted);font-size:12px;font-weight:400'>(" + htmlEscape(formatSendStatus(g_lastWebhookSendMs, g_cfg.webhookIntervalMs)) + ")</span></h4>";
  body += "<form method='POST' action='/save_webhook'>";
  body += "<label><input type='checkbox' name='whEn' value='1' " + String(g_cfg.webhookEnabled ? "checked" : "") + "> Enable Webhook</label>";
  body += "<label>Webhook URL</label><input name='whUrl' value='" + htmlEscape(g_cfg.webhookUrl) + "' placeholder='https://example.com/hook'/>";
  body += "<label>Interval (seconds)</label><input name='whInt' value='" + String((unsigned long)(g_cfg.webhookIntervalMs / 1000UL)) + "'/>";
  body += "<label style='margin-top:12px'>Send data from</label>";
  body += "<div style='display:flex;gap:12px;flex-wrap:wrap'>";
  body += "<label><input type='checkbox' name='whBle' value='1' " + String(g_cfg.bleSendWebhook ? "checked" : "") + "> BLE</label>";
  body += "<label><input type='checkbox' name='whGpio' value='1' " + String(g_cfg.gpioSendWebhook ? "checked" : "") + "> GPIO</label>";
  body += "<label><input type='checkbox' name='whI2c' value='1' " + String(g_cfg.i2cSendWebhook ? "checked" : "") + "> I2C</label>";
  body += "</div>";
  body += "<details><summary style='cursor:pointer;color:var(--muted);font-size:0.9em'>Show JSON fields</summary>";
  body += "<label style='margin-top:12px'>Webhook JSON fields (BLE)</label>";
  appendBleFieldCheckboxes(body, "w_f_", g_webhookBleFields);
#if NETTEMP_ENABLE_I2C
  body += "<label style='margin-top:12px'>Webhook JSON fields (I2C)</label>";
  appendI2cFieldCheckboxes(body, "w_f_i2c_", g_webhookI2cFields);
#endif
  body += "<label style='margin-top:12px'>Webhook JSON fields (GPIO)</label>";
  appendGpioFieldCheckboxes(body, "w_f_gpio_", g_webhookGpioFields);
  body += "</details>";
  body += "<p style='margin:12px 0 0;display:flex;gap:8px;align-items:center'>";
  body += "<button type='submit'>Save</button>";
  body += "<button type='button' id='sendWebhook'>Send now</button>";
  body += "<span id='statusWebhook' style='color:var(--muted);font-size:0.9em'></span>";
  body += "</p>";
  body += "</form>";
  body += "</div>";
  body += "</div>";
  body += "</div>";
  };
  renderTabChannels();

  auto renderTabDevice = [&]() {
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
  };
  renderTabDevice();

  auto renderTabAdmin = [&]() {
  body += "<div class='card tab hidden' id='tab-admin'>";
  body += "<h3>Admin</h3>";
  body += "<form method='POST' action='/save_admin'>";
  body += "<label>Username</label><input name='webUser' value='" + htmlEscape(g_cfg.webUser) + "'/>";
  body += "<label>Password</label><input name='webPass' type='password' value='' placeholder='(leave empty to keep existing)'/>";
  body += "<p style='margin:12px 0 0'><button type='submit'>Save</button></p>";
  body += "</form>";
  body += "</div>";
  };
  renderTabAdmin();

  auto renderTabSystem = [&]() {
  body += "<div class='card tab hidden' id='tab-system'>";
  body += "<h3>System</h3>";
  body += "<p><code>" + String(ESP.getChipModel()) + "</code> | rev <code>" + String(ESP.getChipRevision()) +
          "</code> | cores <code>" + String(ESP.getChipCores()) + "</code> | CPU <code>" +
          String((unsigned long)(ESP.getCpuFreqMHz())) + " MHz</code></p>";

  // Memory statistics
  {
    const uint32_t freeHeap = ESP.getFreeHeap();
    const uint32_t totalHeap = ESP.getHeapSize();
    const uint32_t usedHeap = totalHeap - freeHeap;
    const uint32_t minFreeHeap = ESP.getMinFreeHeap();
    const float usedPct = (totalHeap > 0) ? (usedHeap * 100.0f / totalHeap) : 0.0f;

    body += "<p>Heap: <code>" + String(usedHeap / 1024) + " KB</code> used (<code>" + String(usedPct, 1) + "%</code>)";
    body += " | <code>" + String(freeHeap / 1024) + " KB</code> free";
    body += " | total <code>" + String(totalHeap / 1024) + " KB</code>";
    body += " | min free <code>" + String(minFreeHeap / 1024) + " KB</code></p>";

    // PSRAM info
    const uint32_t psramSize = ESP.getPsramSize();
    if (psramSize > 0) {
      const uint32_t freePsram = ESP.getFreePsram();
      const uint32_t usedPsram = psramSize - freePsram;
      const float psramUsedPct = (psramSize > 0) ? (usedPsram * 100.0f / psramSize) : 0.0f;

      body += "<p>PSRAM: <code>" + String(usedPsram / 1024) + " KB</code> used (<code>" + String(psramUsedPct, 1) + "%</code>)";
      body += " | <code>" + String(freePsram / 1024) + " KB</code> free";
      body += " | total <code>" + String(psramSize / 1024) + " KB</code></p>";
    }

    // Flash info
    const uint32_t flashSize = ESP.getFlashChipSize();
    body += "<p>Flash: <code>" + String(flashSize / 1024 / 1024) + " MB</code>";
    body += " | speed <code>" + String(ESP.getFlashChipSpeed() / 1000000) + " MHz</code></p>";
  }

  // Uptime
  {
    const uint32_t uptimeS = millis() / 1000UL;
    const uint32_t days = uptimeS / 86400UL;
    const uint32_t hours = (uptimeS % 86400UL) / 3600UL;
    const uint32_t mins = (uptimeS % 3600UL) / 60UL;
    const uint32_t secs = uptimeS % 60UL;

    String uptimeStr;
    if (days > 0) uptimeStr = String(days) + "d " + String(hours) + "h " + String(mins) + "m";
    else if (hours > 0) uptimeStr = String(hours) + "h " + String(mins) + "m " + String(secs) + "s";
    else if (mins > 0) uptimeStr = String(mins) + "m " + String(secs) + "s";
    else uptimeStr = String(secs) + "s";

    body += "<p>Uptime: <code>" + uptimeStr + "</code></p>";
  }

  body += "<p>WiFi: <code>" + String(wifiConnected() ? "connected" : "not connected") + "</code>";
  body += " | BLE scan: <code>on</code> (" + String(g_activeScan ? "active" : "passive") + ")";
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
  };
  renderTabSystem();

  // JavaScript for real-time send status updates
  body += "<script>";
  body += "function setupSendButton(btnId, endpoint, statusEndpoint, statusId) {";
  body += "  const btn = document.getElementById(btnId);";
  body += "  const status = document.getElementById(statusId);";
  body += "  if (!btn) return;";
  body += "  btn.addEventListener('click', async () => {";
  body += "    btn.disabled = true;";
  body += "    status.textContent = 'Sending...';";
  body += "    status.style.color = 'var(--muted)';";
  body += "    try {";
  body += "      const resp = await fetch(endpoint, {method: 'POST'});";
  body += "      if (!resp.ok) throw new Error('Send failed');";
  body += "      let attempts = 0;";
  body += "      const checkStatus = setInterval(async () => {";
  body += "        attempts++;";
  body += "        if (attempts > 30) {"; // 30 seconds max
  body += "          clearInterval(checkStatus);";
  body += "          status.textContent = 'Timeout';";
  body += "          status.style.color = '#e74c3c';";
  body += "          btn.disabled = false;";
  body += "          setTimeout(() => { status.textContent = ''; }, 5000);";
  body += "          return;";
  body += "        }";
  body += "        const st = await fetch(statusEndpoint);";
  body += "        const code = parseInt(await st.text());";
  body += "        if (code === 2) {"; // Success
  body += "          clearInterval(checkStatus);";
  body += "          status.textContent = 'Sent successfully!';";
  body += "          status.style.color = '#27ae60';";
  body += "          btn.disabled = false;";
  body += "          setTimeout(() => { status.textContent = ''; }, 5000);";
  body += "        } else if (code === 3) {"; // Failed
  body += "          clearInterval(checkStatus);";
  body += "          status.textContent = 'Send failed';";
  body += "          status.style.color = '#e74c3c';";
  body += "          btn.disabled = false;";
  body += "          setTimeout(() => { status.textContent = ''; }, 5000);";
  body += "        }";
  body += "      }, 1000);"; // Poll every 1 second
  body += "    } catch (e) {";
  body += "      status.textContent = 'Error: ' + e.message;";
  body += "      status.style.color = '#e74c3c';";
  body += "      btn.disabled = false;";
  body += "      setTimeout(() => { status.textContent = ''; }, 5000);";
  body += "    }";
  body += "  });";
  body += "}";
  body += "setupSendButton('sendMqtt', '/send_mqtt', '/send_status_mqtt', 'statusMqtt');";
  body += "setupSendButton('sendServer', '/send_server', '/send_status_server', 'statusServer');";
  body += "setupSendButton('sendLocalServer', '/send_local_server', '/send_status_local_server', 'statusLocalServer');";
  body += "setupSendButton('sendWebhook', '/send_webhook', '/send_status_webhook', 'statusWebhook');";
  body += "</script>";

  body += "</div></body></html>";
  return body;
}
