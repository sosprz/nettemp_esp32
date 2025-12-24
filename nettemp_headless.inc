#if NETTEMP_HEADLESS
static void headlessPrintConfig() {
  Serial.println();
  Serial.println("=== Nettemp (HEADLESS) ===");
  Serial.printf("deviceId %s\n", g_cfg.deviceId.c_str());
  Serial.printf("wifi ssid %s status %s ip %s\n",
                g_cfg.wifiSsid.length() ? g_cfg.wifiSsid.c_str() : "(none)",
                wifiConnected() ? "connected" : "not connected",
                wifiConnected() ? WiFi.localIP().toString().c_str() : "-");
  Serial.printf("wifi pass: %s\n", g_cfg.wifiPass.length() ? "(set)" : "(none)");

  Serial.printf("ble scan %s\n", g_activeScan ? "ACTIVE" : "PASSIVE");
  Serial.printf("ble max_results %u\n", (unsigned)g_bleMaxResults);
  Serial.printf("ble autoscan %s\n", g_headlessBleAutoScan ? "on" : "off");
  Serial.printf("ble parse_mfg %s\n", g_bleParseMfg ? "on" : "off");
  Serial.printf("log %s interval_ms %lu\n",
                g_headlessStatusLogEnabled ? "on" : "off",
                (unsigned long)g_headlessStatusLogIntervalMs);
  Serial.printf("mqtt %s host %s port %u interval_s %lu\n",
                g_cfg.mqttEnabled ? "on" : "off",
                g_cfg.mqttHost.c_str(),
                (unsigned)g_cfg.mqttPort,
                (unsigned long)(g_cfg.mqttIntervalMs / 1000UL));
  Serial.printf("mqtt user %s\n", g_cfg.mqttUser.length() ? g_cfg.mqttUser.c_str() : "(none)");
  Serial.printf("mqtt pass: %s\n", g_cfg.mqttPass.length() ? "(set)" : "(none)");
  Serial.printf("server %s url %s interval_s %lu\n",
                g_cfg.serverEnabled ? "on" : "off",
                g_cfg.serverBaseUrl.c_str(),
                (unsigned long)(g_cfg.serverIntervalMs / 1000UL));
  Serial.printf("server key: %s...\n", g_cfg.serverApiKey.length() ? g_cfg.serverApiKey.substring(0, 10).c_str() : "(none)");
  Serial.printf("server json %s (available %s)\n", serverBleFieldsEnabledString().c_str(), serverBleFieldsAvailableString().c_str());
#if NETTEMP_ENABLE_I2C
  Serial.printf("server json i2c %s (available %s)\n", serverI2cFieldsEnabledString().c_str(), serverI2cFieldsAvailableString().c_str());
  Serial.printf("i2c pins sda %d scl %d\n", g_cfg.i2cSdaPin, g_cfg.i2cSclPin);
#endif
  Serial.printf("ds18b20 %s mode %s pin %d count %u\n",
                g_cfg.dsEnabled ? "on" : "off",
                "gpio",
                g_cfg.dsPin,
                (unsigned)g_dsRoms.size());
  Serial.printf("dht %s type %d pin %d\n", g_cfg.dhtEnabled ? "on" : "off", g_cfg.dhtType, g_cfg.dhtPin);
  Serial.printf("power sleep %s sleep_s %lu ble_scan_s %lu wifi_wait_s %lu\n",
                g_cfg.powerSleepEnabled ? "on" : "off",
                (unsigned long)g_cfg.powerSleepSeconds,
                (unsigned long)g_cfg.powerBleScanSeconds,
                (unsigned long)g_cfg.powerWifiWaitSeconds);
  Serial.printf("power boot_grace_s %lu awake_hold_s %lu\n",
                (unsigned long)g_cfg.powerBootGraceSeconds,
                (unsigned long)g_cfg.powerAwakeHoldSeconds);
  {
    const char* mode =
      (g_cfg.vbatMode == 0) ? "off" :
      (g_cfg.vbatMode == 1) ? "adc" :
      (g_cfg.vbatMode == 2) ? "pmic" : "?";
    Serial.printf("vbat mode %s adc_pin %d divider_x1000 %u full_mv %u empty_mv %u send_volt %s\n",
                  mode,
                  g_cfg.vbatAdcPin,
                  (unsigned)g_cfg.vbatDividerX1000,
                  (unsigned)g_cfg.vbatFullMv,
                  (unsigned)g_cfg.vbatEmptyMv,
                  g_cfg.vbatSendVolt ? "on" : "off");
    if (g_vbatPct >= 0) {
      Serial.printf("vbat last batt %d%%", g_vbatPct);
      if (!isnan(g_vbatVolts)) Serial.printf(" vbat %.3fV", g_vbatVolts);
      Serial.println();
    }
  }

  Serial.printf("autoSelect %s\n", g_headlessAutoSelect ? "on" : "off");
  Serial.printf("bleSensors %u\n", (unsigned)g_sensors.size());
#if NETTEMP_ENABLE_I2C
  Serial.printf("i2cSensors %u\n", (unsigned)g_i2cSensors.size());
#endif
  Serial.println("Type: help");
}

static void headlessPrintHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  help");
  Serial.println("  status");
  Serial.println("  log on|off");
  Serial.println("  log interval <ms>         (1000..300000)");
  Serial.println("  device <deviceId>");
  Serial.println("  wifi scan");
  Serial.println("  wifi set <ssid> <pass>");
  Serial.println("  wifi connect | wifi disconnect");
  Serial.println("  wifi ap on|off");
#if NETTEMP_ENABLE_PORTAL
  Serial.println("  web on <user> <pass>      (enable web config + set Basic Auth)");
  Serial.println("  web pass <pass>           (set web password; keeps current user)");
  Serial.println("  web status");
#endif
  Serial.println("  ble active | ble passive");
  Serial.println("  ble list                  (show decoded devices; * = selected, M = manual)");
  Serial.println("  ble select <mac> on|off   (manual override; persists)");
  Serial.println("  ble auto <mac>            (clear manual override)");
  Serial.println("  ble scan                  (scan 10s and print decoded JSON)");
  Serial.println("  ble scan raw              (scan 10s and print raw counts)");
  Serial.println("  ble max <n>               (5..200)");
  Serial.println("  ble mfg on|off             (decode manufacturer data; default: off)");
  Serial.println("  ble autoscan on|off       (start/stop background scan)");
  Serial.println("  autoselect on|off");
  Serial.println("  i2c rescan");
  Serial.println("  i2c select all|none");
  Serial.println("  i2c pins <sda> <scl>       (-1 = default)");
  Serial.println("  i2c default                (use board defaults)");
  Serial.println("  ds on|off");
  Serial.println("  ds pin <gpio>");
  Serial.println("  ds rescan");
  Serial.println("  dht on|off");
  Serial.println("  dht pin <gpio>");
  Serial.println("  dht type 11|22");
  Serial.println("  vbat off | vbat adc | vbat pmic");
  Serial.println("  vbat pin <gpio>           (ADC mode)");
  Serial.println("  vbat div <x1000>          (ratio; e.g. 2000 = 2.0)");
  Serial.println("  vbat full <mv> | vbat empty <mv>");
  Serial.println("  vbat sendvolt on|off");
  Serial.println("  mqtt on|off");
  Serial.println("  mqtt host <host>");
  Serial.println("  mqtt port <port>");
  Serial.println("  mqtt user <user>");
  Serial.println("  mqtt pass <pass>");
  Serial.println("  mqtt interval <seconds>");
  Serial.println("  mqtt set <host> <port> <interval_s>  (shortcut)");
  Serial.println("  mqtt json [fields...]     (BLE->MQTT fields; e.g. tempc hum batt rssi)");
#if NETTEMP_ENABLE_I2C
  Serial.println("  mqtt json i2c [fields...] (I2C->MQTT fields; e.g. tempc press_hpa)");
#endif
  Serial.println("  server on|off");
  Serial.println("  server url <baseUrl>");
  Serial.println("  server key <ntk_token>");
  Serial.println("  server interval <seconds>");
  Serial.println("  server json [fields...]   (BLE->Cloud fields; e.g. tempc hum batt rssi)");
#if NETTEMP_ENABLE_I2C
  Serial.println("  server json i2c [fields...] (I2C->Cloud fields; e.g. tempc press_hpa)");
#endif
  Serial.println("  local json [fields...]    (BLE->Local fields; e.g. tempc hum batt rssi)");
#if NETTEMP_ENABLE_I2C
  Serial.println("  local json i2c [fields...] (I2C->Local fields; e.g. tempc press_hpa)");
#endif
  Serial.println("  webhook json [fields...]  (BLE->Webhook fields; e.g. tempc hum batt rssi)");
#if NETTEMP_ENABLE_I2C
  Serial.println("  webhook json i2c [fields...] (I2C->Webhook fields; e.g. tempc press_hpa)");
#endif
  Serial.println("  server set <baseUrl> <ntk_token> <interval_s>  (shortcut)");
  Serial.println("  send now");
  Serial.println("  reset                    (factory reset + reboot)");
}

static String bleFieldsEnabledStringFor(uint32_t mask) {
  String out;
  auto add = [&](const char* s) {
    if (out.length()) out += " ";
    out += s;
  };
  if (mask & SRV_BLE_TEMPC) add("tempc");
  if (mask & SRV_BLE_TEMPF) add("tempf");
  if (mask & SRV_BLE_HUM) add("hum");
  if (mask & SRV_BLE_BATT) add("batt");
  if (mask & SRV_BLE_VOLT) add("volt");
  if (mask & SRV_BLE_RSSI) add("rssi");
  if (!out.length()) out = "(none)";
  return out;
}

static String bleFieldsAvailableStringFor() {
  return String("tempc tempf hum batt volt rssi");
}

#if NETTEMP_ENABLE_I2C
static String i2cFieldsEnabledStringFor(uint32_t mask) {
  String out;
  auto add = [&](const char* s) {
    if (out.length()) out += " ";
    out += s;
  };
  if (mask & SRV_I2C_TEMPC) add("tempc");
  if (mask & SRV_I2C_HUM) add("hum");
  if (mask & SRV_I2C_PRESS) add("press_hpa");
  if (!out.length()) out = "(none)";
  return out;
}

static String i2cFieldsAvailableStringFor() {
  return String("tempc hum press_hpa");
}
#endif

static String nextToken(const String& line, int& pos) {
  const int n = (int)line.length();
  while (pos < n && isspace((unsigned char)line[pos])) pos++;
  const int start = pos;
  while (pos < n && !isspace((unsigned char)line[pos])) pos++;
  if (start >= n) return "";
  return line.substring(start, pos);
}

static void headlessWifiScan() {
  Serial.println("Scanning WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  const int n = WiFi.scanNetworks(/*async=*/false, /*hidden=*/true);
  if (n <= 0) {
    Serial.println("(no networks found)");
    WiFi.scanDelete();
    return;
  }
  for (int i = 0; i < n; i++) {
    Serial.printf("  %2d: %s (%ddBm)\n", i, WiFi.SSID(i).c_str(), WiFi.RSSI(i));
  }
  WiFi.scanDelete();
}

static void headlessPrintWifiStatus() {
  if (wifiConnected()) {
    Serial.printf("WiFi connected: ssid=%s ip=%s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
  } else {
    Serial.printf("WiFi not connected (status=%d)\n", (int)WiFi.status());
  }
}

static void headlessWaitForWifi(uint32_t timeoutMs) {
  const uint32_t start = millis();
  while (!wifiConnected() && (millis() - start) < timeoutMs) {
    delay(250);
  }
  headlessPrintWifiStatus();
}

static void headlessHandleCommand(String line) {
  line.trim();
  if (!line.length()) return;

  int pos = 0;
  const String cmd = nextToken(line, pos);

  if (cmd == "help" || cmd == "?") {
    headlessPrintHelp();
    return;
  }
  if (cmd == "status") {
    headlessPrintConfig();
    return;
  }
  if (cmd == "log") {
    const String sub = nextToken(line, pos);
    if (sub == "on") {
      g_headlessStatusLogEnabled = true;
      prefsSave();
      Serial.println("OK");
      return;
    }
    if (sub == "off") {
      g_headlessStatusLogEnabled = false;
      prefsSave();
      Serial.println("OK");
      return;
    }
    if (sub == "interval") {
      const String msStr = nextToken(line, pos);
      const uint32_t ms = (uint32_t)msStr.toInt();
      if (ms < 1000 || ms > 300000) {
        Serial.println("Usage: log interval <ms>  (1000..300000)");
        return;
      }
      g_headlessStatusLogIntervalMs = ms;
      prefsSave();
      Serial.println("OK");
      return;
    }
    Serial.println("Usage: log on|off | log interval <ms>");
    return;
  }
  if (cmd == "device") {
    const String id = nextToken(line, pos);
    if (!id.length()) {
      Serial.println("Usage: device <deviceId>");
      return;
    }
    g_cfg.deviceId = id;
    prefsSave();
    Serial.println("OK");
    return;
  }
  if (cmd == "wifi") {
    const String sub = nextToken(line, pos);
    if (sub == "scan") {
      headlessWifiScan();
      return;
    }
    if (sub == "set") {
      const String ssid = nextToken(line, pos);
      const String pass = nextToken(line, pos);
      if (!ssid.length()) {
        Serial.println("Usage: wifi set <ssid> <pass>");
        return;
      }
      g_cfg.wifiSsid = ssid;
      g_cfg.wifiPass = pass;
      prefsSave();
      WiFi.disconnect(true);
      delay(200);
      g_lastWifiBeginMs = 0;
      wifiConnectIfConfigured();
      headlessWaitForWifi(12'000);
      return;
    }
    if (sub == "connect") {
      g_lastWifiBeginMs = 0;
      wifiConnectIfConfigured();
      headlessWaitForWifi(12'000);
      return;
    }
    if (sub == "disconnect") {
      WiFi.disconnect(true);
      headlessPrintWifiStatus();
      return;
    }
#if NETTEMP_ENABLE_PORTAL
    if (sub == "ap") {
      const String v = nextToken(line, pos);
      if (v == "on") {
        portalStart();
        Serial.printf("OK (portal ssid %s ip %s)\n", portalSsid().c_str(), WiFi.softAPIP().toString().c_str());
        return;
      }
      if (v == "off") {
        portalStop();
        Serial.println("OK");
        return;
      }
      Serial.println("Usage: wifi ap on|off");
      return;
    }
#endif
    Serial.println("Usage: wifi scan | wifi set <ssid> <pass> | wifi connect | wifi disconnect | wifi ap on|off");
    return;
  }
#if NETTEMP_ENABLE_PORTAL
  if (cmd == "web") {
    const String sub = nextToken(line, pos);
    if (sub == "status") {
      const char* mode = (g_webMode == WebMode::StaConfig) ? "config" : (g_webMode == WebMode::ApSetup) ? "portal" : "off";
      Serial.printf("web mode %s\n", mode);
      Serial.printf("web user %s\n", g_cfg.webUser.length() ? g_cfg.webUser.c_str() : "(none)");
      Serial.printf("web pass: %s\n", g_cfg.webPass.length() ? "(set)" : "(none)");
      if (wifiConnected()) {
        Serial.printf("web url http://%s/\n", WiFi.localIP().toString().c_str());
      } else if (g_webMode == WebMode::ApSetup) {
        Serial.printf("web url http://%s/\n", WiFi.softAPIP().toString().c_str());
      }
      return;
    }

    if (sub == "on") {
      const String user = nextToken(line, pos);
      const String pass = nextToken(line, pos);
      if (!user.length() || !pass.length()) {
        Serial.println("Usage: web on <user> <pass>");
        return;
      }
      g_cfg.webUser = user;
      g_cfg.webPass = pass;
      prefsSave();

      // If already connected to WiFi, start the config server immediately.
      if (wifiConnected()) {
        configServerStart();
        Serial.printf("OK (web config http://%s/)\n", WiFi.localIP().toString().c_str());
      } else {
        Serial.println("OK (saved). Connect WiFi first; web config server starts when connected.");
      }
      return;
    }

    if (sub == "pass") {
      const String pass = nextToken(line, pos);
      if (!pass.length()) {
        Serial.println("Usage: web pass <pass>");
        return;
      }
      if (!g_cfg.webUser.length()) g_cfg.webUser = "admin";
      g_cfg.webPass = pass;
      prefsSave();
      if (wifiConnected()) {
        configServerStart();
        Serial.printf("OK (web config http://%s/)\n", WiFi.localIP().toString().c_str());
      } else {
        Serial.println("OK");
      }
      return;
    }

    Serial.println("Usage: web on <user> <pass> | web pass <pass> | web status");
    return;
  }
#endif
  if (cmd == "ble") {
    const String mode = nextToken(line, pos);
    if (mode == "list") {
      auto idx = buildBleSortedIndex();
      if (idx.empty()) {
        Serial.println("(no BLE devices)");
        return;
      }
      Serial.println("BLE devices:");
      for (size_t j = 0; j < idx.size(); j++) {
        const auto& s = g_sensors[idx[j]];
        const bool hasValues = !isnan(s.temperatureC) || !isnan(s.humidityPct) || s.batteryPct >= 0;
        if (!hasValues) continue;
        const String mac = macWithColonsUpper(s.mac);
        const String macNo = macNoColonsUpper(mac);
        bool manualSelected = false;
        const bool manual = bleManualGetSelected(macNo, manualSelected);
        const bool selected = manual ? manualSelected : s.selected;
        const char selMark = selected ? '*' : ' ';
        const char manMark = manual ? 'M' : ' ';
        Serial.printf("%c%c %s tempc=%s hum=%s batt=%s\n",
                      selMark,
                      manMark,
                      mac.c_str(),
                      isnan(s.temperatureC) ? "-" : String(s.temperatureC, 2).c_str(),
                      isnan(s.humidityPct) ? "-" : String(s.humidityPct, 1).c_str(),
                      s.batteryPct < 0 ? "-" : String(s.batteryPct).c_str());
      }
      return;
    }
    if (mode == "select") {
      const String macIn = nextToken(line, pos);
      const String onOff = nextToken(line, pos);
      if (!macIn.length() || !onOff.length()) {
        Serial.println("Usage: ble select <mac> on|off");
        return;
      }
      const String mac = normalizeMacToColonsUpper(macIn);
      const String macNo = macNoColonsUpper(mac);
      if (onOff == "on") {
        bleManualSet(macNo, true);
        prefsSave();
        Serial.println("OK");
        return;
      }
      if (onOff == "off") {
        bleManualSet(macNo, false);
        prefsSave();
        Serial.println("OK");
        return;
      }
      Serial.println("Usage: ble select <mac> on|off");
      return;
    }
    if (mode == "auto") {
      const String macIn = nextToken(line, pos);
      if (!macIn.length()) {
        Serial.println("Usage: ble auto <mac>  (clear manual override)");
        return;
      }
      const String mac = normalizeMacToColonsUpper(macIn);
      const String macNo = macNoColonsUpper(mac);
      bleManualClear(macNo);
      prefsSave();
      Serial.println("OK");
      return;
    }
    if (mode == "scan") {
      const String sub = nextToken(line, pos);
      Serial.println("Scanning BLE for 10s...");
#if NETTEMP_HEADLESS
      g_headlessBleScanCollectUnknown = true;
      g_headlessDiagnosticsScanActive = true;
#endif
      bleStartScanFor(/*durationMs=*/10'000, /*keepExistingResults=*/false);
      // Always wait a bit to allow scan callbacks to run; don't depend on isScanning()
      // because start/stop is async and may briefly report false.
      const uint32_t start = millis();
      while ((millis() - start) < 10'800) {
        delay(50);
      }
      if (g_scan) g_scan->stop();

      auto idx = buildBleSortedIndex();
      if (idx.empty()) {
        Serial.println("(no devices)");
#if NETTEMP_HEADLESS
        g_headlessBleScanCollectUnknown = false;
        g_headlessDiagnosticsScanActive = false;
#endif
        return;
      }
      if (sub == "raw") {
        Serial.printf("Found %u devices (showing up to %u)\n", (unsigned)idx.size(), (unsigned)g_bleMaxResults);
        for (size_t j = 0; j < idx.size(); j++) {
          const auto& s = g_sensors[idx[j]];
          String advPreview = s.lastAdvHex;
          if (advPreview.length() > 48) advPreview = advPreview.substring(0, 48) + "...";
          Serial.printf(
            "%s rssi=%d src=%s uuid=%s adv_len=%u decoded=%s adv=%s\n",
            s.mac.c_str(),
            s.rssi,
            s.lastAdvSrc.length() ? s.lastAdvSrc.c_str() : "-",
            s.lastAdvUuid.length() ? s.lastAdvUuid.c_str() : "-",
            (unsigned)(s.lastAdvHex.length() / 2),
            buildBtToMqttJsonMinimal(s) == "{}" ? "no" : "yes",
            advPreview.c_str()
          );
        }
        Serial.println("Tip: paste the `adv=` hex for a sensor (e.g. A4:C1:...) if decoded=no; we can adjust the parser.");
      } else {
        Serial.println("Decoded devices:");
        bool any = false;
        for (size_t j = 0; j < idx.size(); j++) {
          const auto& s = g_sensors[idx[j]];
          const String json = buildBtToMqttJsonMinimal(s);
          if (json == "{}") continue;
          any = true;
          Serial.printf("%s %s\n", s.mac.c_str(), json.c_str());
        }
        if (!any) {
          Serial.printf("(no decoded devices; saw %u BLE addresses)\n", (unsigned)idx.size());
          Serial.println("Try: ble active, then ble mfg on (if your firmware uses manufacturer payloads).");
        }
        // Avoid keeping a huge list of random BLE devices in RAM after a diagnostic scan.
        // Keep only decoded sensors.
        blePurgeUndecoded();
      }

      if (g_headlessBleAutoScan) {
        bleConfigureScan();
      }
#if NETTEMP_HEADLESS
      g_headlessBleScanCollectUnknown = false;
      g_headlessDiagnosticsScanActive = false;
#endif
      return;
    }
    if (mode == "max") {
      const String nStr = nextToken(line, pos);
      const int n = nStr.toInt();
      if (n < 5 || n > 200) {
        Serial.println("Usage: ble max <n>  (5..200)");
        return;
      }
      g_bleMaxResults = (uint8_t)n;
      prefsSave();
      bleConfigureScan();
      Serial.println("OK");
      return;
    }
    if (mode == "mfg") {
      const String v = nextToken(line, pos);
      if (v == "on") {
        g_bleParseMfg = true;
        prefsSave();
        Serial.println("OK");
        return;
      }
      if (v == "off") {
        g_bleParseMfg = false;
        prefsSave();
        Serial.println("OK");
        return;
      }
      Serial.println("Usage: ble mfg on|off");
      return;
    }
    if (mode == "autoscan") {
      const String v = nextToken(line, pos);
      if (v == "on") {
        g_headlessBleAutoScan = true;
        prefsSave();
        bleConfigureScan();
        Serial.println("OK");
        return;
      }
      if (v == "off") {
        g_headlessBleAutoScan = false;
        prefsSave();
        if (g_scan) g_scan->stop();
        Serial.println("OK");
        return;
      }
      Serial.println("Usage: ble autoscan on|off");
      return;
    }
    if (mode == "active") {
      g_activeScan = true;
      prefsSave();
      bleConfigureScan();
      Serial.println("OK");
      return;
    }
    if (mode == "passive") {
      g_activeScan = false;
      prefsSave();
      bleConfigureScan();
      Serial.println("OK");
      return;
    }
    Serial.println("Usage: ble scan | ble active | ble passive | ble max <n> | ble mfg on|off | ble autoscan on|off");
    return;
  }
  if (cmd == "autoselect") {
    const String v = nextToken(line, pos);
    if (v == "on") {
      g_headlessAutoSelect = true;
      Serial.println("OK");
      return;
    }
    if (v == "off") {
      g_headlessAutoSelect = false;
      Serial.println("OK");
      return;
    }
    Serial.println("Usage: autoselect on|off");
    return;
  }
  if (cmd == "i2c") {
#if !NETTEMP_ENABLE_I2C
    Serial.println("I2C is disabled at compile time (NETTEMP_ENABLE_I2C=0).");
    return;
#else
    const String sub = nextToken(line, pos);
    if (sub == "pins") {
      const String sdaStr = nextToken(line, pos);
      const String sclStr = nextToken(line, pos);
      if (!sdaStr.length() || !sclStr.length()) {
        Serial.println("Usage: i2c pins <sda> <scl>  (-1 = default)");
        return;
      }
      g_cfg.i2cSdaPin = sdaStr.toInt();
      g_cfg.i2cSclPin = sclStr.toInt();
      prefsSave();
      i2cInitBus();
      g_i2cDetectedAddrs = i2cScanAllAddresses(Wire);
      g_i2cSensors = i2cDetectKnownSensors(Wire);
      if (!g_i2cSensors.empty()) i2cUpdateReadings(Wire, g_i2cSensors);
      if (g_headlessAutoSelect) for (auto& s : g_i2cSensors) s.selected = true;
      if (g_i2cSelDefined) i2cApplySelectionToDetected();
      prefsSave();
      Serial.printf("OK (pins sda=%d scl=%d; found %u)\n", g_cfg.i2cSdaPin, g_cfg.i2cSclPin, (unsigned)g_i2cSensors.size());
      return;
    }
    if (sub == "default") {
      g_cfg.i2cSdaPin = -1;
      g_cfg.i2cSclPin = -1;
      prefsSave();
      i2cInitBus();
      g_i2cDetectedAddrs = i2cScanAllAddresses(Wire);
      g_i2cSensors = i2cDetectKnownSensors(Wire);
      if (!g_i2cSensors.empty()) i2cUpdateReadings(Wire, g_i2cSensors);
      if (g_headlessAutoSelect) for (auto& s : g_i2cSensors) s.selected = true;
      if (g_i2cSelDefined) i2cApplySelectionToDetected();
      prefsSave();
      Serial.printf("OK (pins default; found %u)\n", (unsigned)g_i2cSensors.size());
      return;
    }
    if (sub == "rescan") {
      i2cInitBus();
      g_i2cDetectedAddrs = i2cScanAllAddresses(Wire);
      g_i2cSensors = i2cDetectKnownSensors(Wire);
      if (!g_i2cSensors.empty()) i2cUpdateReadings(Wire, g_i2cSensors);
      if (g_headlessAutoSelect) {
        for (auto& s : g_i2cSensors) s.selected = true;
      }
      Serial.printf("OK (found %u)\n", (unsigned)g_i2cSensors.size());
      return;
    }
    if (sub == "select") {
      const String which = nextToken(line, pos);
      if (which == "all") {
        for (auto& s : g_i2cSensors) s.selected = true;
        Serial.println("OK");
        return;
      }
      if (which == "none") {
        for (auto& s : g_i2cSensors) s.selected = false;
        Serial.println("OK");
        return;
      }
      Serial.println("Usage: i2c select all|none");
      return;
    }
    Serial.println("Usage: i2c rescan | i2c select all|none | i2c pins <sda> <scl> | i2c default");
    return;
#endif
  }
  if (cmd == "ds") {
    const String sub = nextToken(line, pos);
    if (sub == "on") {
      g_cfg.dsEnabled = true;
      prefsSave();
      dsEnsureBus();
      dsRescan();
      Serial.printf("OK (found %u)\n", (unsigned)g_dsRoms.size());
      return;
    }
    if (sub == "off") {
      g_cfg.dsEnabled = false;
      prefsSave();
      dsEnsureBus();
      Serial.println("OK");
      return;
    }
    if (sub == "pin") {
      const String pinStr = nextToken(line, pos);
      if (!pinStr.length()) {
        Serial.println("Usage: ds pin <gpio>");
        return;
      }
      g_cfg.dsPin = pinStr.toInt();
      g_cfg.dsEnabled = true;
      prefsSave();
      dsEnsureBus();
      dsRescan();
      Serial.printf("OK (found %u)\n", (unsigned)g_dsRoms.size());
      return;
    }
    if (sub == "rescan") {
      dsRescan();
      Serial.printf("OK (found %u)\n", (unsigned)g_dsRoms.size());
      return;
    }
    Serial.println("Usage: ds on|off | ds pin <gpio> | ds rescan");
    return;
  }
  if (cmd == "dht") {
    const String sub = nextToken(line, pos);
    if (sub == "on") {
      g_cfg.dhtEnabled = true;
      prefsSave();
      g_dhtLastReadMs = 0;
      Serial.println("OK");
      return;
    }
    if (sub == "off") {
      g_cfg.dhtEnabled = false;
      prefsSave();
      Serial.println("OK");
      return;
    }
    if (sub == "pin") {
      const String pinStr = nextToken(line, pos);
      if (!pinStr.length()) {
        Serial.println("Usage: dht pin <gpio>");
        return;
      }
      g_cfg.dhtPin = pinStr.toInt();
      g_cfg.dhtEnabled = true;
      prefsSave();
      g_dhtLastReadMs = 0;
      Serial.println("OK");
      return;
    }
    if (sub == "type") {
      const String tStr = nextToken(line, pos);
      const int t = tStr.toInt();
      if (t != 11 && t != 22) {
        Serial.println("Usage: dht type 11|22");
        return;
      }
      g_cfg.dhtType = t;
      g_cfg.dhtEnabled = true;
      prefsSave();
      g_dhtLastReadMs = 0;
      Serial.println("OK");
      return;
    }
    Serial.println("Usage: dht on|off | dht pin <gpio> | dht type 11|22");
    return;
  }
  if (cmd == "mqtt") {
#if !NETTEMP_ENABLE_MQTT
    Serial.println("MQTT is disabled at compile time (NETTEMP_ENABLE_MQTT=0).");
    return;
#else
    const String sub = nextToken(line, pos);
    if (sub == "on") {
      g_cfg.mqttEnabled = true;
      prefsSave();
      mqttEnsureConnected();
      Serial.println("OK");
      return;
    }
    if (sub == "off") {
      g_cfg.mqttEnabled = false;
      prefsSave();
      Serial.println("OK");
      return;
    }
    if (sub == "host") {
      const String host = nextToken(line, pos);
      if (!host.length()) {
        Serial.println("Usage: mqtt host <host>");
        return;
      }
      g_cfg.mqttHost = host;
      prefsSave();
      mqttEnsureConnected();
      Serial.println("OK");
      return;
    }
    if (sub == "port") {
      const String portStr = nextToken(line, pos);
      const int port = portStr.toInt();
      if (port <= 0 || port > 65535) {
        Serial.println("Usage: mqtt port <port>");
        return;
      }
      g_cfg.mqttPort = (uint16_t)port;
      prefsSave();
      mqttEnsureConnected();
      Serial.println("OK");
      return;
    }
    if (sub == "user") {
      const String user = nextToken(line, pos);
      // Allow empty username to clear.
      g_cfg.mqttUser = user;
      prefsSave();
      mqttEnsureConnected();
      Serial.println("OK");
      return;
    }
    if (sub == "pass") {
      const String pass = nextToken(line, pos);
      // Allow empty password to clear.
      g_cfg.mqttPass = pass;
      prefsSave();
      mqttEnsureConnected();
      Serial.println("OK");
      return;
    }
    if (sub == "interval") {
      const String secondsStr = nextToken(line, pos);
      const int seconds = secondsStr.toInt();
      if (seconds < 5 || seconds > 3600) {
        Serial.println("Usage: mqtt interval <seconds>  (5..3600)");
        return;
      }
      g_cfg.mqttIntervalMs = (uint32_t)seconds * 1000U;
      prefsSave();
      Serial.println("OK");
      return;
    }
    if (sub == "set") {
      const String host = nextToken(line, pos);
      const String portStr = nextToken(line, pos);
      const String intervalStr = nextToken(line, pos);
      if (!host.length() || !portStr.length() || !intervalStr.length()) {
        Serial.println("Usage: mqtt set <host> <port> <interval_s>");
        return;
      }
      g_cfg.mqttHost = host;
      g_cfg.mqttPort = (uint16_t)portStr.toInt();
      const uint32_t seconds = (uint32_t)intervalStr.toInt();
      g_cfg.mqttIntervalMs = seconds * 1000UL;
      prefsSave();
      mqttEnsureConnected();
      Serial.println("OK");
      return;
    }
    if (sub == "json") {
      const int posBefore = pos;
      const String maybeI2c = nextToken(line, pos);
      const bool isI2c = (maybeI2c == "i2c");
      if (!isI2c) pos = posBefore;

      uint32_t mask = 0;
      bool sawAny = false;
      while (true) {
        const String t = nextToken(line, pos);
        if (!t.length()) break;
        sawAny = true;
        if (!isI2c) {
          if (t == "tempc") mask |= SRV_BLE_TEMPC;
          else if (t == "tempf") mask |= SRV_BLE_TEMPF;
          else if (t == "hum" || t == "humid" || t == "humidity") mask |= SRV_BLE_HUM;
          else if (t == "batt" || t == "battery") mask |= SRV_BLE_BATT;
          else if (t == "volt" || t == "voltage") mask |= SRV_BLE_VOLT;
          else if (t == "rssi" || t == "signal") mask |= SRV_BLE_RSSI;
          else {
            Serial.printf("Unknown field: %s\n", t.c_str());
            Serial.printf("Available: %s\n", bleFieldsAvailableStringFor().c_str());
            return;
          }
        } else {
#if NETTEMP_ENABLE_I2C
          if (t == "tempc") mask |= SRV_I2C_TEMPC;
          else if (t == "hum" || t == "humid" || t == "humidity") mask |= SRV_I2C_HUM;
          else if (t == "press" || t == "pressure" || t == "press_hpa") mask |= SRV_I2C_PRESS;
          else {
            Serial.printf("Unknown field: %s\n", t.c_str());
            Serial.printf("Available: %s\n", i2cFieldsAvailableStringFor().c_str());
            return;
          }
#else
          (void)mask;
          Serial.println("I2C disabled at compile time.");
          return;
#endif
        }
      }

      if (!sawAny) {
        if (!isI2c) {
          Serial.printf("mqtt json %s (available %s)\n",
                        bleFieldsEnabledStringFor(g_mqttBleFields).c_str(),
                        bleFieldsAvailableStringFor().c_str());
        } else {
#if NETTEMP_ENABLE_I2C
          Serial.printf("mqtt json i2c %s (available %s)\n",
                        i2cFieldsEnabledStringFor(g_mqttI2cFields).c_str(),
                        i2cFieldsAvailableStringFor().c_str());
#else
          Serial.println("I2C disabled at compile time.");
#endif
        }
        return;
      }

      if (mask == 0) {
        if (!isI2c) {
          Serial.println("Usage: mqtt json <fields...>  (at least one field)");
          Serial.printf("Available: %s\n", bleFieldsAvailableStringFor().c_str());
        } else {
          Serial.println("Usage: mqtt json i2c <fields...>  (at least one field)");
#if NETTEMP_ENABLE_I2C
          Serial.printf("Available: %s\n", i2cFieldsAvailableStringFor().c_str());
#endif
        }
        return;
      }

#if NETTEMP_ENABLE_I2C
      if (isI2c) g_mqttI2cFields = mask;
      else g_mqttBleFields = mask;
#else
      g_mqttBleFields = mask;
#endif
      prefsSave();
      Serial.println("OK");
      return;
    }
    Serial.println("Usage: mqtt on|off | mqtt host <host> | mqtt port <port> | mqtt user <user> | mqtt pass <pass> | mqtt interval <seconds> | mqtt set <host> <port> <interval_s> | mqtt json [fields...] | mqtt json i2c [fields...]");
    return;
#endif
  }
  if (cmd == "vbat") {
    const String sub = nextToken(line, pos);
    if (sub == "off") {
      g_cfg.vbatMode = 0;
      prefsSave();
      Serial.println("OK");
      return;
    }
    if (sub == "adc") {
      g_cfg.vbatMode = 1;
      prefsSave();
      Serial.println("OK");
      return;
    }
    if (sub == "pmic") {
      g_cfg.vbatMode = 2;
      prefsSave();
      Serial.println("OK");
      return;
    }
    if (sub == "pin") {
      const String pinStr = nextToken(line, pos);
      const int pin = pinStr.toInt();
      if (!pinStr.length()) {
        Serial.println("Usage: vbat pin <gpio>");
        return;
      }
      g_cfg.vbatAdcPin = pin;
      prefsSave();
      Serial.println("OK");
      return;
    }
    if (sub == "div") {
      const String divStr = nextToken(line, pos);
      const uint32_t div = (uint32_t)divStr.toInt();
      if (!divStr.length() || div < 500 || div > 5000) {
        Serial.println("Usage: vbat div <x1000>  (500..5000)");
        return;
      }
      g_cfg.vbatDividerX1000 = (uint16_t)div;
      prefsSave();
      Serial.println("OK");
      return;
    }
    if (sub == "full") {
      const String mvStr = nextToken(line, pos);
      const uint32_t mv = (uint32_t)mvStr.toInt();
      if (!mvStr.length() || mv < 3000 || mv > 5000) {
        Serial.println("Usage: vbat full <mv>  (3000..5000)");
        return;
      }
      g_cfg.vbatFullMv = (uint16_t)mv;
      prefsSave();
      Serial.println("OK");
      return;
    }
    if (sub == "empty") {
      const String mvStr = nextToken(line, pos);
      const uint32_t mv = (uint32_t)mvStr.toInt();
      if (!mvStr.length() || mv < 2500 || mv > 4500) {
        Serial.println("Usage: vbat empty <mv>  (2500..4500)");
        return;
      }
      g_cfg.vbatEmptyMv = (uint16_t)mv;
      prefsSave();
      Serial.println("OK");
      return;
    }
    if (sub == "sendvolt") {
      const String v = nextToken(line, pos);
      if (v != "on" && v != "off") {
        Serial.println("Usage: vbat sendvolt on|off");
        return;
      }
      g_cfg.vbatSendVolt = (v == "on");
      prefsSave();
      Serial.println("OK");
      return;
    }

    // default: show current vbat config + last reading
    headlessPrintConfig();
    return;
  }
  if (cmd == "server") {
#if !NETTEMP_ENABLE_SERVER
    Serial.println("Server send is disabled at compile time (NETTEMP_ENABLE_SERVER=0).");
    return;
#else
    const String sub = nextToken(line, pos);
    if (sub == "on") {
      g_cfg.serverEnabled = true;
      prefsSave();
      Serial.println("OK");
      return;
    }
    if (sub == "off") {
      g_cfg.serverEnabled = false;
      prefsSave();
      Serial.println("OK");
      return;
    }
    if (sub == "url") {
      const String url = nextToken(line, pos);
      if (!url.length()) {
        Serial.println("Usage: server url <baseUrl>");
        return;
      }
      g_cfg.serverBaseUrl = url;
      prefsSave();
      Serial.println("OK");
      return;
    }
    if (sub == "key") {
      const String key = nextToken(line, pos);
      if (!key.length()) {
        Serial.println("Usage: server key <ntk_token>");
        return;
      }
      g_cfg.serverApiKey = key;
      prefsSave();
      Serial.println("OK");
      return;
    }
    if (sub == "interval") {
      const String secondsStr = nextToken(line, pos);
      const int seconds = secondsStr.toInt();
      if (seconds < 5 || seconds > 3600) {
        Serial.println("Usage: server interval <seconds>  (5..3600)");
        return;
      }
      g_cfg.serverIntervalMs = (uint32_t)seconds * 1000U;
      prefsSave();
      Serial.println("OK");
      return;
    }
    if (sub == "json") {
      const int posBefore = pos;
      const String maybeI2c = nextToken(line, pos);
      const bool isI2c = (maybeI2c == "i2c");
      if (!isI2c) {
        pos = posBefore; // parse BLE fields normally
      }

      uint32_t mask = 0;
      bool sawAny = false;
      while (true) {
        const String t = nextToken(line, pos);
        if (!t.length()) break;
        sawAny = true;
        if (!isI2c) {
          if (t == "tempc") mask |= SRV_BLE_TEMPC;
          else if (t == "tempf") mask |= SRV_BLE_TEMPF;
          else if (t == "hum" || t == "humid" || t == "humidity") mask |= SRV_BLE_HUM;
          else if (t == "batt" || t == "battery") mask |= SRV_BLE_BATT;
          else if (t == "volt" || t == "voltage") mask |= SRV_BLE_VOLT;
          else if (t == "rssi" || t == "signal") mask |= SRV_BLE_RSSI;
          else {
            Serial.printf("Unknown field: %s\n", t.c_str());
            Serial.printf("Available: %s\n", serverBleFieldsAvailableString().c_str());
            return;
          }
        } else {
#if NETTEMP_ENABLE_I2C
          if (t == "tempc") mask |= SRV_I2C_TEMPC;
          else if (t == "hum" || t == "humid" || t == "humidity") mask |= SRV_I2C_HUM;
          else if (t == "press" || t == "pressure" || t == "press_hpa") mask |= SRV_I2C_PRESS;
          else {
            Serial.printf("Unknown field: %s\n", t.c_str());
            Serial.printf("Available: %s\n", serverI2cFieldsAvailableString().c_str());
            return;
          }
#else
          (void)mask;
          Serial.println("I2C disabled at compile time.");
          return;
#endif
        }
      }

      if (!sawAny) {
        if (!isI2c) {
          Serial.printf("server json %s (available %s)\n",
                        serverBleFieldsEnabledString().c_str(),
                        serverBleFieldsAvailableString().c_str());
          // Print a sample payload using the first selected BLE sensor (if any).
          for (const auto& s : g_sensors) {
            if (!s.selected) continue;
            const String mac = macNoColonsUpper(s.mac);
            Serial.println("server json payload:");
            Serial.printf("{\"device_id\":\"%s\",\"readings\":[", mac.c_str());
            bool first = true;
            auto emit = [&](const String& sensorId, const char* sensorType, float value) {
              if (!first) Serial.print(",");
              first = false;
              Serial.printf("{\"sensor_id\":\"%s\",\"sensor_type\":\"%s\",\"value\":%.3f}", sensorId.c_str(), sensorType, value);
            };
            if ((g_srvBleFields & SRV_BLE_TEMPC) && !isnan(s.temperatureC)) emit(mac + "_tempc", "temperature", s.temperatureC);
            if ((g_srvBleFields & SRV_BLE_TEMPF) && !isnan(s.temperatureC)) emit(mac + "_tempf", "temperature_f", (s.temperatureC * 9.0f / 5.0f) + 32.0f);
            if ((g_srvBleFields & SRV_BLE_HUM) && !isnan(s.humidityPct)) emit(mac + "_hum", "humidity", s.humidityPct);
            if ((g_srvBleFields & SRV_BLE_BATT) && s.batteryPct >= 0) emit(mac + "_batt", "battery", (float)s.batteryPct);
            if ((g_srvBleFields & SRV_BLE_VOLT) && s.voltageMv >= 0) emit(mac + "_volt", "voltage", (float)s.voltageMv / 1000.0f);
            if ((g_srvBleFields & SRV_BLE_RSSI) && s.rssi != 0) emit(mac + "_rssi", "rssi", (float)s.rssi);
            Serial.println("]}");
            break;
          }
        } else {
#if NETTEMP_ENABLE_I2C
          Serial.printf("server json i2c %s (available %s)\n",
                        serverI2cFieldsEnabledString().c_str(),
                        serverI2cFieldsAvailableString().c_str());
          for (const auto& s : g_i2cSensors) {
            if (!s.selected) continue;
            if (!s.reading.ok) continue;
            const String dev = g_cfg.deviceId.length() ? g_cfg.deviceId : String("nettemp_esp32");
            String addrHex = String(s.address, HEX);
            addrHex.toLowerCase();
            if (addrHex.length() == 1) addrHex = "0" + addrHex;
            const String typeName = String(i2cSensorTypeName(s.type));
            const String base = dev + "-i2c_" + typeName + "_0x" + addrHex;
            Serial.println("server json payload:");
            Serial.printf("{\"device_id\":\"%s\",\"readings\":[", dev.c_str());
            bool first = true;
            auto emit = [&](const String& sensorId, const char* sensorType, float value) {
              if (!first) Serial.print(",");
              first = false;
              Serial.printf("{\"sensor_id\":\"%s\",\"sensor_type\":\"%s\",\"value\":%.3f}", sensorId.c_str(), sensorType, value);
            };
            if ((g_srvI2cFields & SRV_I2C_TEMPC) && !isnan(s.reading.temperature_c)) emit(base + "_temp", "temperature", s.reading.temperature_c);
            if ((g_srvI2cFields & SRV_I2C_HUM) && !isnan(s.reading.humidity_pct)) emit(base + "_hum", "humidity", s.reading.humidity_pct);
            if ((g_srvI2cFields & SRV_I2C_PRESS) && !isnan(s.reading.pressure_hpa)) emit(base + "_press", "pressure", s.reading.pressure_hpa);
            Serial.println("]}");
            break;
          }
#endif
        }
        return;
      }

      if (mask == 0) {
        if (!isI2c) {
          Serial.println("Usage: server json <fields...>  (at least one field)");
          Serial.printf("Available: %s\n", serverBleFieldsAvailableString().c_str());
        } else {
          Serial.println("Usage: server json i2c <fields...>  (at least one field)");
          Serial.printf("Available: %s\n", serverI2cFieldsAvailableString().c_str());
        }
        return;
      }

#if NETTEMP_ENABLE_I2C
      if (isI2c) g_srvI2cFields = mask;
      else g_srvBleFields = mask;
#else
      g_srvBleFields = mask;
#endif
      prefsSave();
      Serial.println("OK");
      return;
    }
    if (sub == "set") {
      const String url = nextToken(line, pos);
      const String key = nextToken(line, pos);
      const String intervalStr = nextToken(line, pos);
      if (!url.length() || !key.length() || !intervalStr.length()) {
        Serial.println("Usage: server set <baseUrl> <ntk_token> <interval_s>");
        return;
      }
      g_cfg.serverBaseUrl = url;
      g_cfg.serverApiKey = key;
      const uint32_t seconds = (uint32_t)intervalStr.toInt();
      g_cfg.serverIntervalMs = seconds * 1000UL;
      prefsSave();
      Serial.println("OK");
      return;
    }
    Serial.println(
      "Usage: server on|off | server url <baseUrl> | server key <ntk_token> | server interval <seconds> | "
      "server json [fields...] | server json i2c [fields...] | server set <baseUrl> <ntk_token> <interval_s>");
    return;
#endif
  }
  if (cmd == "local") {
    const String sub = nextToken(line, pos);
    if (sub == "json") {
      const int posBefore = pos;
      const String maybeI2c = nextToken(line, pos);
      const bool isI2c = (maybeI2c == "i2c");
      if (!isI2c) pos = posBefore;

      uint32_t mask = 0;
      bool sawAny = false;
      while (true) {
        const String t = nextToken(line, pos);
        if (!t.length()) break;
        sawAny = true;
        if (!isI2c) {
          if (t == "tempc") mask |= SRV_BLE_TEMPC;
          else if (t == "tempf") mask |= SRV_BLE_TEMPF;
          else if (t == "hum" || t == "humid" || t == "humidity") mask |= SRV_BLE_HUM;
          else if (t == "batt" || t == "battery") mask |= SRV_BLE_BATT;
          else if (t == "volt" || t == "voltage") mask |= SRV_BLE_VOLT;
          else if (t == "rssi" || t == "signal") mask |= SRV_BLE_RSSI;
          else {
            Serial.printf("Unknown field: %s\n", t.c_str());
            Serial.printf("Available: %s\n", bleFieldsAvailableStringFor().c_str());
            return;
          }
        } else {
#if NETTEMP_ENABLE_I2C
          if (t == "tempc") mask |= SRV_I2C_TEMPC;
          else if (t == "hum" || t == "humid" || t == "humidity") mask |= SRV_I2C_HUM;
          else if (t == "press" || t == "pressure" || t == "press_hpa") mask |= SRV_I2C_PRESS;
          else {
            Serial.printf("Unknown field: %s\n", t.c_str());
            Serial.printf("Available: %s\n", i2cFieldsAvailableStringFor().c_str());
            return;
          }
#else
          (void)mask;
          Serial.println("I2C disabled at compile time.");
          return;
#endif
        }
      }

      if (!sawAny) {
        if (!isI2c) {
          Serial.printf("local json %s (available %s)\n",
                        bleFieldsEnabledStringFor(g_localBleFields).c_str(),
                        bleFieldsAvailableStringFor().c_str());
        } else {
#if NETTEMP_ENABLE_I2C
          Serial.printf("local json i2c %s (available %s)\n",
                        i2cFieldsEnabledStringFor(g_localI2cFields).c_str(),
                        i2cFieldsAvailableStringFor().c_str());
#else
          Serial.println("I2C disabled at compile time.");
#endif
        }
        return;
      }

      if (mask == 0) {
        if (!isI2c) {
          Serial.println("Usage: local json <fields...>  (at least one field)");
          Serial.printf("Available: %s\n", bleFieldsAvailableStringFor().c_str());
        } else {
          Serial.println("Usage: local json i2c <fields...>  (at least one field)");
#if NETTEMP_ENABLE_I2C
          Serial.printf("Available: %s\n", i2cFieldsAvailableStringFor().c_str());
#endif
        }
        return;
      }

#if NETTEMP_ENABLE_I2C
      if (isI2c) g_localI2cFields = mask;
      else g_localBleFields = mask;
#else
      g_localBleFields = mask;
#endif
      prefsSave();
      Serial.println("OK");
      return;
    }
    Serial.println("Usage: local json [fields...] | local json i2c [fields...]");
    return;
  }
  if (cmd == "webhook") {
    const String sub = nextToken(line, pos);
    if (sub == "json") {
      const int posBefore = pos;
      const String maybeI2c = nextToken(line, pos);
      const bool isI2c = (maybeI2c == "i2c");
      if (!isI2c) pos = posBefore;

      uint32_t mask = 0;
      bool sawAny = false;
      while (true) {
        const String t = nextToken(line, pos);
        if (!t.length()) break;
        sawAny = true;
        if (!isI2c) {
          if (t == "tempc") mask |= SRV_BLE_TEMPC;
          else if (t == "tempf") mask |= SRV_BLE_TEMPF;
          else if (t == "hum" || t == "humid" || t == "humidity") mask |= SRV_BLE_HUM;
          else if (t == "batt" || t == "battery") mask |= SRV_BLE_BATT;
          else if (t == "volt" || t == "voltage") mask |= SRV_BLE_VOLT;
          else if (t == "rssi" || t == "signal") mask |= SRV_BLE_RSSI;
          else {
            Serial.printf("Unknown field: %s\n", t.c_str());
            Serial.printf("Available: %s\n", bleFieldsAvailableStringFor().c_str());
            return;
          }
        } else {
#if NETTEMP_ENABLE_I2C
          if (t == "tempc") mask |= SRV_I2C_TEMPC;
          else if (t == "hum" || t == "humid" || t == "humidity") mask |= SRV_I2C_HUM;
          else if (t == "press" || t == "pressure" || t == "press_hpa") mask |= SRV_I2C_PRESS;
          else {
            Serial.printf("Unknown field: %s\n", t.c_str());
            Serial.printf("Available: %s\n", i2cFieldsAvailableStringFor().c_str());
            return;
          }
#else
          (void)mask;
          Serial.println("I2C disabled at compile time.");
          return;
#endif
        }
      }

      if (!sawAny) {
        if (!isI2c) {
          Serial.printf("webhook json %s (available %s)\n",
                        bleFieldsEnabledStringFor(g_webhookBleFields).c_str(),
                        bleFieldsAvailableStringFor().c_str());
        } else {
#if NETTEMP_ENABLE_I2C
          Serial.printf("webhook json i2c %s (available %s)\n",
                        i2cFieldsEnabledStringFor(g_webhookI2cFields).c_str(),
                        i2cFieldsAvailableStringFor().c_str());
#else
          Serial.println("I2C disabled at compile time.");
#endif
        }
        return;
      }

      if (mask == 0) {
        if (!isI2c) {
          Serial.println("Usage: webhook json <fields...>  (at least one field)");
          Serial.printf("Available: %s\n", bleFieldsAvailableStringFor().c_str());
        } else {
          Serial.println("Usage: webhook json i2c <fields...>  (at least one field)");
#if NETTEMP_ENABLE_I2C
          Serial.printf("Available: %s\n", i2cFieldsAvailableStringFor().c_str());
#endif
        }
        return;
      }

#if NETTEMP_ENABLE_I2C
      if (isI2c) g_webhookI2cFields = mask;
      else g_webhookBleFields = mask;
#else
      g_webhookBleFields = mask;
#endif
      prefsSave();
      Serial.println("OK");
      return;
    }
    Serial.println("Usage: webhook json [fields...] | webhook json i2c [fields...]");
    return;
  }
  if (cmd == "send") {
    const String sub = nextToken(line, pos);
    if (sub == "now") {
      g_lastServerSendMs = 0;
#if NETTEMP_ENABLE_MQTT
      for (auto& s : g_sensors) s.lastMqttSentMs = 0;
#endif
      Serial.println("OK");
      return;
    }
    Serial.println("Usage: send now");
    return;
  }
  if (cmd == "reset") {
    Serial.println("Resetting to defaults...");
#if NETTEMP_ENABLE_PORTAL
    doFactoryReset();
#else
    g_prefs.begin("nettemp", false);
    g_prefs.clear();
    g_prefs.end();
#endif
    delay(200);
    ESP.restart();
  }

  Serial.printf("Unknown command: %s\n", cmd.c_str());
  Serial.println("Type: help");
}

static void headlessSerialPoll() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    g_lastSerialRxMs = millis();
    if (c == '\r') continue;
    if (c == '\n') {
      String line = g_serialLineBuf;
      g_serialLineBuf = "";
      headlessHandleCommand(line);
      continue;
    }
    if (g_serialLineBuf.length() < 256) g_serialLineBuf += c;
  }
}

static void headlessTickLog() {
  if (!g_headlessStatusLogEnabled) return;
  const uint32_t now = millis();
  if (now - g_lastHeadlessLogMs < g_headlessStatusLogIntervalMs) return;
  g_lastHeadlessLogMs = now;

  int selectedBle = 0;
  for (const auto& s : g_sensors) if (s.selected) selectedBle++;

  Serial.printf("[status] wifi=%s ip=%s ble=%u(sel %d) mqtt=%s server=%s",
                wifiConnected() ? "on" : "off",
                wifiConnected() ? WiFi.localIP().toString().c_str() : "-",
                (unsigned)g_sensors.size(),
                selectedBle,
                g_cfg.mqttEnabled ? "on" : "off",
                g_cfg.serverEnabled ? "on" : "off");
#if NETTEMP_ENABLE_I2C
  int selectedI2c = 0;
  for (const auto& s : g_i2cSensors) if (s.selected) selectedI2c++;
  Serial.printf(" i2c=%u(sel %d)", (unsigned)g_i2cSensors.size(), selectedI2c);
#endif
  Serial.println();
}
#endif
