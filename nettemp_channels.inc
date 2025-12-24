using BleSensor = SensorRow;
#if NETTEMP_ENABLE_I2C
using I2cSensor = I2cSensorInfo;
#endif

#if NETTEMP_ENABLE_SERVER
template <typename NameFn>
static void appendBleReadingsServerLike(NettempBatch& batch, const BleSensor& s, const String& base,
                                        uint32_t mask, uint32_t ts, NameFn nameFor) {
  if ((mask & SRV_BLE_TEMPC) && !isnan(s.temperatureC)) {
    batch.readings.push_back(NettempReading{
      .sensorId = base + "_temp",
      .value = s.temperatureC,
      .sensorType = "temperature",
      .unit = "°C",
      .timestamp = ts,
      .friendlyName = nameFor("temperature"),
    });
  }
  if ((mask & SRV_BLE_TEMPF) && !isnan(s.temperatureC)) {
    batch.readings.push_back(NettempReading{
      .sensorId = base + "_tempf",
      .value = (s.temperatureC * 9.0f / 5.0f) + 32.0f,
      .sensorType = "temperature_f",
      .unit = "",
      .timestamp = ts,
      .friendlyName = nameFor("temperature_f"),
    });
  }
  if ((mask & SRV_BLE_HUM) && !isnan(s.humidityPct)) {
    batch.readings.push_back(NettempReading{
      .sensorId = base + "_hum",
      .value = s.humidityPct,
      .sensorType = "humidity",
      .unit = "%",
      .timestamp = ts,
      .friendlyName = nameFor("humidity"),
    });
  }
  if ((mask & SRV_BLE_BATT) && s.batteryPct >= 0) {
    batch.readings.push_back(NettempReading{
      .sensorId = base + "_batt",
      .value = (float)s.batteryPct,
      .sensorType = "battery",
      .unit = "%",
      .timestamp = ts,
      .friendlyName = nameFor("battery"),
    });
  }
  if ((mask & SRV_BLE_VOLT) && s.voltageMv >= 0) {
    batch.readings.push_back(NettempReading{
      .sensorId = base + "_volt",
      .value = (float)s.voltageMv / 1000.0f,
      .sensorType = "voltage",
      .unit = "",
      .timestamp = ts,
      .friendlyName = nameFor("volt"),
    });
  }
  if ((mask & SRV_BLE_RSSI) && s.rssi != 0) {
    batch.readings.push_back(NettempReading{
      .sensorId = base + "_rssi",
      .value = (float)s.rssi,
      .sensorType = "rssi",
      .unit = "dBm",
      .timestamp = ts,
      .friendlyName = nameFor("rssi"),
    });
  }
}

template <typename NameFn>
static void appendBleReadingsWebhook(NettempBatch& batch, const BleSensor& s, const String& base,
                                     uint32_t mask, uint32_t ts, NameFn nameFor) {
  if ((mask & SRV_BLE_TEMPC) && !isnan(s.temperatureC)) {
    batch.readings.push_back(NettempReading{
      .sensorId = base + "_tempc",
      .value = s.temperatureC,
      .sensorType = "temperature",
      .unit = "°C",
      .timestamp = ts,
      .friendlyName = nameFor("temperature"),
    });
  }
  if ((mask & SRV_BLE_TEMPF) && !isnan(s.temperatureC)) {
    batch.readings.push_back(NettempReading{
      .sensorId = base + "_tempf",
      .value = (s.temperatureC * 9.0f / 5.0f) + 32.0f,
      .sensorType = "temperature_f",
      .unit = "",
      .timestamp = ts,
      .friendlyName = nameFor("temperature_f"),
    });
  }
  if ((mask & SRV_BLE_HUM) && !isnan(s.humidityPct)) {
    batch.readings.push_back(NettempReading{
      .sensorId = base + "_hum",
      .value = s.humidityPct,
      .sensorType = "humidity",
      .unit = "%",
      .timestamp = ts,
      .friendlyName = nameFor("humidity"),
    });
  }
  if ((mask & SRV_BLE_BATT) && s.batteryPct >= 0) {
    batch.readings.push_back(NettempReading{
      .sensorId = base + "_batt",
      .value = (float)s.batteryPct,
      .sensorType = "battery",
      .unit = "%",
      .timestamp = ts,
      .friendlyName = nameFor("battery"),
    });
  }
  if ((mask & SRV_BLE_VOLT) && s.voltageMv >= 0) {
    batch.readings.push_back(NettempReading{
      .sensorId = base + "_volt",
      .value = (float)s.voltageMv / 1000.0f,
      .sensorType = "voltage",
      .unit = "",
      .timestamp = ts,
      .friendlyName = nameFor("volt"),
    });
  }
  if ((mask & SRV_BLE_RSSI) && s.rssi != 0) {
    batch.readings.push_back(NettempReading{
      .sensorId = base + "_rssi",
      .value = (float)s.rssi,
      .sensorType = "rssi",
      .unit = "dBm",
      .timestamp = ts,
      .friendlyName = nameFor("rssi"),
    });
  }
}

#if NETTEMP_ENABLE_I2C
template <typename NameFn>
static void appendI2cReadingsServerLike(NettempBatch& batch, const I2cSensor& s, const String& base,
                                        uint32_t mask, uint32_t ts, NameFn nameFor) {
  if ((mask & SRV_I2C_TEMPC) && !isnan(s.reading.temperature_c)) {
    batch.readings.push_back(NettempReading{
      .sensorId = base + "_temp",
      .value = s.reading.temperature_c,
      .sensorType = "temperature",
      .unit = "°C",
      .timestamp = ts,
      .friendlyName = nameFor("temp"),
    });
  }
  if ((mask & SRV_I2C_HUM) && !isnan(s.reading.humidity_pct)) {
    batch.readings.push_back(NettempReading{
      .sensorId = base + "_hum",
      .value = s.reading.humidity_pct,
      .sensorType = "humidity",
      .unit = "%",
      .timestamp = ts,
      .friendlyName = nameFor("hum"),
    });
  }
  if ((mask & SRV_I2C_PRESS) && !isnan(s.reading.pressure_hpa)) {
    batch.readings.push_back(NettempReading{
      .sensorId = base + "_press",
      .value = s.reading.pressure_hpa,
      .sensorType = "pressure",
      .unit = "hPa",
      .timestamp = ts,
      .friendlyName = nameFor("press"),
    });
  }
}
#endif
#endif

#if NETTEMP_ENABLE_MQTT

template <typename PublishFn, typename NameFn>
static void publishBleReadingsMqtt(const BleSensor& s, const String& base, uint32_t mask,
                                   PublishFn publish, NameFn nameFor) {
  if ((mask & SRV_BLE_TEMPC) && !isnan(s.temperatureC)) {
    publish(base + "_tempc", "temperature", nameFor("temperature"), String(s.temperatureC, 2));
  }
  if ((mask & SRV_BLE_TEMPF) && !isnan(s.temperatureC)) {
    const float tempf = (s.temperatureC * 9.0f / 5.0f) + 32.0f;
    publish(base + "_tempf", "temperature_f", nameFor("temperature_f"), String(tempf, 2));
  }
  if ((mask & SRV_BLE_HUM) && !isnan(s.humidityPct)) {
    publish(base + "_hum", "humidity", nameFor("humidity"), String(s.humidityPct, 1));
  }
  if ((mask & SRV_BLE_BATT) && s.batteryPct >= 0) {
    publish(base + "_batt", "battery", nameFor("battery"), String(s.batteryPct));
  }
  if ((mask & SRV_BLE_VOLT) && s.voltageMv >= 0) {
    publish(base + "_volt", "voltage", nameFor("volt"), String((float)s.voltageMv / 1000.0f, 3));
  }
  if ((mask & SRV_BLE_RSSI) && s.rssi != 0) {
    publish(base + "_rssi", "rssi", nameFor("rssi"), String(s.rssi));
  }
}

#if NETTEMP_ENABLE_I2C
template <typename PublishFn, typename NameFn>
static void publishI2cReadingsMqtt(const I2cSensor& s, const String& base, uint32_t mask,
                                   PublishFn publish, NameFn nameFor) {
  if ((mask & SRV_I2C_TEMPC) && !isnan(s.reading.temperature_c)) {
    publish(base + "_tempc", "temperature", nameFor("temp"), String(s.reading.temperature_c, 2));
  }
  if ((mask & SRV_I2C_HUM) && !isnan(s.reading.humidity_pct)) {
    publish(base + "_hum", "humidity", nameFor("hum"), String(s.reading.humidity_pct, 1));
  }
  if ((mask & SRV_I2C_PRESS) && !isnan(s.reading.pressure_hpa)) {
    publish(base + "_press_hpa", "pressure", nameFor("press"), String(s.reading.pressure_hpa, 1));
  }
}
#endif
#endif
