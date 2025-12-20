static int vbatToPercent(float vbatV) {
  const float full = (float)g_cfg.vbatFullMv / 1000.0f;
  const float empty = (float)g_cfg.vbatEmptyMv / 1000.0f;
  if (!(full > empty)) return -1;
  if (vbatV <= empty) return 0;
  if (vbatV >= full) return 100;
  const float pct = (vbatV - empty) / (full - empty) * 100.0f;
  const int ip = (int)(pct + 0.5f);
  return std::min(100, std::max(0, ip));
}

static bool vbatReadVolts(float& outV) {
  outV = NAN;
  if (g_cfg.vbatMode == 0) return false;

  if (g_cfg.vbatMode == 2) {
#if !NETTEMP_HEADLESS
    // Cardputer PMIC (best-effort; only if the M5Cardputer Power API provides it).
    struct M5Pmic {
      template <typename T>
      static auto batteryVoltage(const T& o, int) -> decltype(o.Power.getBatteryVoltage()) {
        return o.Power.getBatteryVoltage();
      }
      static float batteryVoltage(...) { return NAN; }

      template <typename T>
      static auto batteryLevel(const T& o, int) -> decltype(o.Power.getBatteryLevel()) {
        return o.Power.getBatteryLevel();
      }
      static int batteryLevel(...) { return -1; }
    };

    float v = (float)M5Pmic::batteryVoltage(M5Cardputer, 0);
    if (!isnan(v)) {
      if (v > 20.0f) v = v / 1000.0f; // mV -> V
      if (v > 1.0f && v < 6.0f) {
        outV = v;
        return true;
      }
    }

    const int pct = (int)M5Pmic::batteryLevel(M5Cardputer, 0);
    if (pct >= 0 && pct <= 100) {
      g_vbatPct = pct;
      g_vbatLastSeenMs = millis();
      // No voltage available; treat as "read succeeded" for percent-only flows.
      return false;
    }
#endif
    // If PMIC not available, continue to ADC path.
  }

  // ADC mode (or PMIC fallback)
  if (g_cfg.vbatMode == 1 || g_cfg.vbatMode == 2) {
    const int pin = g_cfg.vbatAdcPin;
    if (pin < 0) return false;
    const float ratio = (float)g_cfg.vbatDividerX1000 / 1000.0f;
    if (ratio <= 0.1f) return false;

    // Configure ADC for more stable readings (best-effort).
#if defined(ARDUINO_ARCH_ESP32)
    static bool adcConfigured = false;
    static int adcConfiguredPin = -999;
    if (!adcConfigured || adcConfiguredPin != pin) {
      adcConfigured = true;
      adcConfiguredPin = pin;
      analogReadResolution(12);
      analogSetPinAttenuation((uint8_t)pin, ADC_11db);
    }
#endif

    uint32_t mv = 0;
#if defined(ARDUINO_ARCH_ESP32)
    mv = (uint32_t)analogReadMilliVolts((uint8_t)pin);
#endif
    if (mv == 0) {
      const int raw = analogRead((uint8_t)pin);
      if (raw <= 0) return false;
      // Fallback: assume 12-bit ADC and 3.3V reference on the ADC pin.
      const float vAdc = ((float)raw / 4095.0f) * 3.3f;
      float v = vAdc * ratio;
      // Heuristic: some boards/cores already scale ADC to VBAT (divider built-in).
      // If applying the divider pushes us out of the plausible Li-ion range, use the ADC voltage as-is.
      if ((v > 6.0f || v < 1.0f) && (vAdc > 1.0f && vAdc < 6.0f)) v = vAdc;
      outV = v;
      return true;
    }
    const float vAdc = (float)mv / 1000.0f;
    float v = vAdc * ratio;
    if ((v > 6.0f || v < 1.0f) && (vAdc > 1.0f && vAdc < 6.0f)) v = vAdc;
    outV = v;
    return true;
  }

  return false;
}

static void vbatTick() {
  if (g_cfg.vbatMode == 0) return;
  const uint32_t nowMs = millis();
  // Don't spam ADC reads; 1s is plenty (and in sleep cycle we call it explicitly).
  if (g_vbatLastSeenMs != 0 && (nowMs - g_vbatLastSeenMs) < 1000UL) return;

  float v = NAN;
  if (!vbatReadVolts(v)) return;
  if (isnan(v) || v < 1.0f || v > 6.0f) return;
  g_vbatVolts = v;
  g_vbatPct = vbatToPercent(v);
  g_vbatLastSeenMs = nowMs;
}

static bool serialIsConnected() { return false; }

static void wifiWaitConnected(uint32_t timeoutMs) {
  const uint32_t start = millis();
  while (!wifiConnected() && (millis() - start) < timeoutMs) {
    wifiConnectIfConfigured();
    delay(120);
  }
}

static void powerEnterDeepSleep(uint32_t sleepSeconds) {
  if (sleepSeconds < 1) sleepSeconds = 1;
  const uint64_t us = (uint64_t)sleepSeconds * 1000000ULL;

#if NETTEMP_ENABLE_PORTAL
  // Stop web/portal services to avoid keeping anything alive.
  portalStop();
#endif

  // Best-effort shutdown of radios before sleep.
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  btStop();

  esp_sleep_enable_timer_wakeup(us);
  esp_deep_sleep_start();
}

static void powerSleepCycleMaybe() {
  if (!g_cfg.powerSleepEnabled) return;
  if (g_powerCycleDoneThisBoot) return;

  const auto cause = esp_sleep_get_wakeup_cause();
  const auto resetReason = esp_reset_reason();
  // Some boards/cores report wakeup cause as UNDEFINED even after deep sleep timer wake.
  // Treat deep-sleep resets as timer wakeups for duty-cycle purposes.
  const bool timerWake = (cause == ESP_SLEEP_WAKEUP_TIMER) || (resetReason == ESP_RST_DEEPSLEEP);

  // On manual reset/power-on we wait a grace period and only run the first cycle once,
  // so the user can still enter the web UI (or connect Serial to block sleep).
  if (!timerWake) {
    if (g_powerBootCycleDone) return;
    if (g_cfg.powerBootGraceSeconds == 0) return;
    if (g_powerBootMs == 0) return;
    if ((millis() - g_powerBootMs) < (g_cfg.powerBootGraceSeconds * 1000UL)) return;
    if (g_cfg.wifiSsid.length() == 0) return;
    g_powerBootCycleDone = true;
  }

  wifiWaitConnected(g_cfg.powerWifiWaitSeconds * 1000UL);
  if (!wifiConnected()) {
    // If WiFi can't connect within the wake window, do not enter deep sleep.
    // We intentionally do NOT auto-start the AP portal here to avoid confusing "setup portal" after wake.
    return;
  }

  g_powerCycleDoneThisBoot = true;

  // Short BLE scan burst (default 5s).
  if (g_scan) {
    bleStartScanFor(g_cfg.powerBleScanSeconds * 1000UL, /*keepExistingResults=*/false);
    const uint32_t start = millis();
    const uint32_t waitMs = (g_cfg.powerBleScanSeconds * 1000UL) + 300UL;
    while ((millis() - start) < waitMs) {
      delay(40);
    }
    g_scan->stop();
  }

#if NETTEMP_ENABLE_I2C
  if (!g_i2cSensors.empty()) i2cUpdateReadings(Wire, g_i2cSensors);
#endif
  dsTick();
  dhtTick();
  vbatTick();

  // Force send now (ignore intervals).
  g_lastServerSendMs = 0;
#if NETTEMP_ENABLE_MQTT
  for (auto& s : g_sensors) s.lastMqttSentMs = 0;
  g_lastMqttLocalSentMs = 0;
#endif
  tickSendMqtt();
  tickSendServer();
  delay(200);

  if (g_cfg.powerAwakeHoldSeconds > 0) {
    const uint32_t untilMs = millis() + (g_cfg.powerAwakeHoldSeconds * 1000UL);
    while ((int32_t)(untilMs - millis()) > 0) {
#if NETTEMP_ENABLE_PORTAL
      portalTick();
      webTickMaybeStartConfig();
#endif
      delay(50);
    }
  }

  powerEnterDeepSleep(g_cfg.powerSleepSeconds);
}
