#include "simulator.h"

#include <math.h>

#include "config.h"

namespace {

float clampf(const float value, const float lo, const float hi) {
  return value < lo ? lo : (value > hi ? hi : value);
}

uint16_t toAdcRaw(const float value) {
  if (value <= 0.0f) {
    return 0;
  }
  if (value >= static_cast<float>(ADC_MAX)) {
    return ADC_MAX;
  }
  return static_cast<uint16_t>(value + 0.5f);
}

}  // namespace

void readSimulator(SensorData &data, const uint32_t sampleIndex) {
  const float t = static_cast<float>(sampleIndex) * 0.35f;

  data.temperatureC = 29.0f + 5.0f * sinf(t);
  data.temperatureC = clampf(data.temperatureC, 24.0f, 34.0f);

  data.humidityPct = 65.0f + 20.0f * sinf(t * 0.73f + 0.8f);
  data.humidityPct = clampf(data.humidityPct, 45.0f, 85.0f);

  data.lux = 10000.0f + 10000.0f * sinf(t * 0.41f + 1.1f);
  data.lux = clampf(data.lux, 0.0f, 20000.0f);

  const float soilWave = 2200.0f + 1400.0f * sinf(t * 0.52f + 0.4f);
  const float rainWave = 1800.0f + 1700.0f * sinf(t * 0.67f + 2.1f);
  data.soilAO = toAdcRaw(clampf(soilWave, 0.0f, static_cast<float>(ADC_MAX)));
  data.rainAO = toAdcRaw(clampf(rainWave, 0.0f, static_cast<float>(ADC_MAX)));
  data.soilPct = ((static_cast<float>(SOIL_RAW_DRY) - static_cast<float>(data.soilAO)) * 100.0f) /
                 static_cast<float>(SOIL_RAW_DRY - SOIL_RAW_WET);
  data.soilPct = clampf(data.soilPct, 0.0f, 100.0f);

  data.soilDO = (data.soilAO >= SOIL_DO_THRESHOLD_AO) ? 1 : 0;
  data.rainDO = (data.rainAO >= RAIN_DO_THRESHOLD_AO) ? 1 : 0;

  data.sensor_invalid = false;
  data.dht_ok = true;
  data.bh1750_ok = true;
  data.soil_ok = true;
  data.rain_ok = true;
  data.dht_last_valid_ms = millis();
  data.bh1750_last_valid_ms = millis();
  data.soil_last_valid_ms = millis();
  data.rain_last_valid_ms = millis();
  data.dht_max_stale_ms = SENSOR_MAX_STALE_MS;
  data.bh1750_max_stale_ms = SENSOR_MAX_STALE_MS;
  data.soil_max_stale_ms = SENSOR_MAX_STALE_MS;
  data.rain_max_stale_ms = SENSOR_MAX_STALE_MS;
}
