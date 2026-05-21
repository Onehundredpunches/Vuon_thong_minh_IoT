#include "sensor_manager.h"

#include <BH1750.h>
#include <DHT.h>
#include <math.h>

#include "config.h"
#include "simulator.h"

namespace SensorManager {
namespace {

DHT g_dht(PIN_DHT22_DATA, DHT22);
BH1750 g_bh1750;

static constexpr float SOIL_EMA_ALPHA = 0.25f;

SensorData g_lastValid{};
float g_soilEmaPct = 0.0f;
bool g_soilEmaInitialized = false;
uint8_t g_debouncedRainDO = 0;
uint8_t g_pendingRainDO = 0;
uint32_t g_rainPendingSinceMs = 0;
bool g_rainDebounceInitialized = false;

float adcToVoltage(const uint16_t raw) {
  return (static_cast<float>(raw) * ADC_REF_VOLTAGE) / static_cast<float>(ADC_MAX);
}

float calibratedRawToPct(const uint16_t raw, const uint16_t wetRaw, const uint16_t dryRaw) {
  const float span = static_cast<float>(dryRaw - wetRaw);
  if (span <= 0.0f) {
    return 0.0f;
  }
  const float pct = ((static_cast<float>(dryRaw) - static_cast<float>(raw)) * 100.0f) / span;
  if (pct < 0.0f) {
    return 0.0f;
  }
  if (pct > 100.0f) {
    return 100.0f;
  }
  return pct;
}

float soilRawToPct(const uint16_t raw) {
  return calibratedRawToPct(raw, SOIL_RAW_WET, SOIL_RAW_DRY);
}

float rainRawToPct(const uint16_t raw) {
  return calibratedRawToPct(raw, RAIN_RAW_WET, RAIN_RAW_DRY);
}

float updateSoilEma(const float soilPct) {
  if (!g_soilEmaInitialized) {
    g_soilEmaPct = soilPct;
    g_soilEmaInitialized = true;
  } else {
    g_soilEmaPct = (SOIL_EMA_ALPHA * soilPct) + ((1.0f - SOIL_EMA_ALPHA) * g_soilEmaPct);
  }
  return g_soilEmaPct;
}

bool stale(const uint32_t nowMs, const uint32_t lastValidMs, const uint32_t maxStaleMs) {
  return lastValidMs == 0 || static_cast<uint32_t>(nowMs - lastValidMs) > maxStaleMs;
}

bool debounceRainDO(const uint8_t rawRainDO, const uint32_t nowMs, uint8_t &debouncedOut) {
  if (!g_rainDebounceInitialized && g_rainPendingSinceMs == 0) {
    g_pendingRainDO = rawRainDO;
    g_rainPendingSinceMs = nowMs;
    debouncedOut = rawRainDO;
    return false;
  }

  if (rawRainDO != g_pendingRainDO) {
    g_pendingRainDO = rawRainDO;
    g_rainPendingSinceMs = nowMs;
    debouncedOut = g_debouncedRainDO;
    return false;
  }

  const bool stable = static_cast<uint32_t>(nowMs - g_rainPendingSinceMs) >= RAIN_DEBOUNCE_MS;
  if (stable) {
    g_debouncedRainDO = g_pendingRainDO;
    g_rainDebounceInitialized = true;
  }

  debouncedOut = g_rainDebounceInitialized ? g_debouncedRainDO : g_pendingRainDO;
  return g_rainDebounceInitialized && stable;
}

void printRainValidity(const bool valid, const uint8_t rawRainDO, const uint8_t debouncedRainDO,
                       const __FlashStringHelper *reason) {
  Serial.print(F("RAIN_VALID valid="));
  Serial.print(valid ? 1 : 0);
  Serial.print(F(" raw_do="));
  Serial.print(rawRainDO);
  Serial.print(F(" debounced_do="));
  Serial.print(debouncedRainDO);
  Serial.print(F(" reason="));
  Serial.println(reason);
}

void resetRainDebounceForTest() {
  g_debouncedRainDO = 0;
  g_pendingRainDO = 0;
  g_rainPendingSinceMs = 0;
  g_rainDebounceInitialized = false;
}

void resetSensorPolicyForTest() {
  g_lastValid = {};
  g_soilEmaPct = 0.0f;
  g_soilEmaInitialized = false;
  resetRainDebounceForTest();
}

void setStaleDefaults(SensorData &data) {
  data.dht_max_stale_ms = SENSOR_MAX_STALE_MS;
  data.bh1750_max_stale_ms = SENSOR_MAX_STALE_MS;
  data.soil_max_stale_ms = SENSOR_MAX_STALE_MS;
  data.rain_max_stale_ms = SENSOR_MAX_STALE_MS;
}

void applyValidityPolicy(SensorData &data, const uint32_t nowMs) {
  setStaleDefaults(data);

  if (data.dht_ok) {
    data.dht_last_valid_ms = nowMs;
    g_lastValid.temperatureC = data.temperatureC;
    g_lastValid.humidityPct = data.humidityPct;
    g_lastValid.dht_last_valid_ms = nowMs;
  } else {
    data.temperatureC = g_lastValid.temperatureC;
    data.humidityPct = g_lastValid.humidityPct;
    data.dht_last_valid_ms = g_lastValid.dht_last_valid_ms;
  }

  if (data.bh1750_ok) {
    data.bh1750_last_valid_ms = nowMs;
    g_lastValid.lux = data.lux;
    g_lastValid.bh1750_last_valid_ms = nowMs;
  } else {
    data.lux = g_lastValid.lux;
    data.bh1750_last_valid_ms = g_lastValid.bh1750_last_valid_ms;
  }

  if (data.soil_ok) {
    data.soil_last_valid_ms = nowMs;
    data.soilPct = updateSoilEma(soilRawToPct(data.soilAO));
    g_lastValid.soilAO = data.soilAO;
    g_lastValid.soilDO = data.soilDO;
    g_lastValid.soilPct = data.soilPct;
    g_lastValid.soil_last_valid_ms = nowMs;
  } else {
    data.soilAO = g_lastValid.soilAO;
    data.soilDO = g_lastValid.soilDO;
    data.soilPct = g_lastValid.soilPct;
    data.soil_last_valid_ms = g_lastValid.soil_last_valid_ms;
  }

  const uint8_t rawRainDO = data.rainDO;
  if (data.rain_ok) {
    uint8_t debouncedRainDO = g_debouncedRainDO;
    const bool rainStable = debounceRainDO(rawRainDO, nowMs, debouncedRainDO);
    data.rainDO = debouncedRainDO;
    if (rainStable) {
      data.rain_last_valid_ms = nowMs;
      g_lastValid.rainAO = data.rainAO;
      g_lastValid.rainDO = data.rainDO;
      g_lastValid.rain_last_valid_ms = nowMs;
      printRainValidity(true, rawRainDO, data.rainDO, F("stable"));
    } else {
      data.rainAO = g_lastValid.rainAO;
      data.rainDO = g_lastValid.rainDO;
      data.rain_last_valid_ms = g_lastValid.rain_last_valid_ms;
      data.rain_ok = false;
      printRainValidity(false, rawRainDO, debouncedRainDO, F("settling"));
    }
  } else {
    data.rainAO = g_lastValid.rainAO;
    data.rainDO = g_lastValid.rainDO;
    data.rain_last_valid_ms = g_lastValid.rain_last_valid_ms;
    printRainValidity(false, rawRainDO, data.rainDO, F("stale"));
  }

  data.dht_ok = data.dht_ok && !stale(nowMs, data.dht_last_valid_ms, data.dht_max_stale_ms);
  data.bh1750_ok = data.bh1750_ok && !stale(nowMs, data.bh1750_last_valid_ms, data.bh1750_max_stale_ms);
  data.soil_ok = data.soil_ok && !stale(nowMs, data.soil_last_valid_ms, data.soil_max_stale_ms);
  data.rain_ok = data.rain_ok && !stale(nowMs, data.rain_last_valid_ms, data.rain_max_stale_ms);
  data.sensor_invalid = !(data.dht_ok && data.bh1750_ok && data.soil_ok && data.rain_ok);
}

void readHardware(SensorData &data) {
  const float humidity = g_dht.readHumidity();
  const float temperatureC = g_dht.readTemperature();
  data.dht_ok = !(isnan(humidity) || isnan(temperatureC));
  if (data.dht_ok) {
    data.humidityPct = humidity;
    data.temperatureC = temperatureC;
  }

  const float lux = g_bh1750.readLightLevel();
  data.bh1750_ok = (lux >= 0.0f && !isnan(lux));
  if (data.bh1750_ok) {
    data.lux = lux;
  }

  data.soilAO = analogRead(PIN_SOIL_AO);
  data.rainAO = analogRead(PIN_RAIN_AO);
  data.soilDO = static_cast<uint8_t>(digitalRead(PIN_SOIL_DO));
  data.rainDO = static_cast<uint8_t>(digitalRead(PIN_RAIN_DO));
  data.soil_ok = true;
  data.rain_ok = true;
}

}  // namespace

void begin() {
  pinMode(PIN_DHT22_DATA, INPUT_PULLUP);
  pinMode(PIN_SOIL_AO, INPUT);
  pinMode(PIN_RAIN_AO, INPUT);
  pinMode(PIN_SOIL_DO, INPUT);
  pinMode(PIN_RAIN_DO, INPUT);

#if APP_MODE_SIMULATOR
  // Simulator mode intentionally avoids real sensor init/read calls.
#else
  g_dht.begin();
  const bool bh1750Ok = g_bh1750.begin();
  if (!bh1750Ok) {
    Serial.println(F("BH1750_ERR"));
  }
#endif
}

SensorData read(const uint32_t sampleIndex) {
  SensorData data{};
#if APP_MODE_SIMULATOR
  readSimulator(data, sampleIndex);
#else
  readHardware(data);
#endif
  applyValidityPolicy(data, millis());
  return data;
}

void printCompactBlock(const SensorData &data) {
  const float soilV = adcToVoltage(data.soilAO);
  const float rainV = adcToVoltage(data.rainAO);

  Serial.print(F("DATA Temp:"));
  if (data.dht_ok) {
    Serial.print(data.temperatureC, 1);
    Serial.print(F("C"));
  } else {
    Serial.print(F("DHT_ERR"));
  }

  Serial.print(F(" Hum:"));
  if (data.dht_ok) {
    Serial.print(data.humidityPct, 1);
    Serial.print(F("%RH"));
  } else {
    Serial.print(F("DHT_ERR"));
  }

  Serial.print(F(" Lux:"));
  if (data.bh1750_ok) {
    Serial.print(data.lux, 1);
    Serial.print(F("lx"));
  } else {
    Serial.print(F("BH1750_ERR"));
  }

  Serial.print(F(" SoilAO:"));
  Serial.print(data.soilAO);
  Serial.print(F(" SoilPct:"));
  Serial.print(data.soilPct, 1);
  Serial.print(F("%"));
  Serial.print(F("("));
  Serial.print(soilV, 3);
  Serial.print(F("V)"));

  Serial.print(F(" RainAO:"));
  Serial.print(data.rainAO);
  Serial.print(F("("));
  Serial.print(rainV, 3);
  Serial.print(F("V)"));
  Serial.print(F(" RainPctDiag:"));
  Serial.print(rainRawToPct(data.rainAO), 1);
  Serial.print(F("%"));

  Serial.print(F(" SoilDO:"));
  Serial.print(data.soilDO);
  Serial.print(F(" RainDO:"));
  Serial.print(data.rainDO);
  Serial.print(F(" Flags dht_ok:"));
  Serial.print(data.dht_ok ? 1 : 0);
  Serial.print(F(" bh1750_ok:"));
  Serial.print(data.bh1750_ok ? 1 : 0);
  Serial.print(F(" soil_ok:"));
  Serial.print(data.soil_ok ? 1 : 0);
  Serial.print(F(" rain_ok:"));
  Serial.println(data.rain_ok ? 1 : 0);
}

bool runPolicySelfTest() {
  const SensorData savedLastValid = g_lastValid;
  const float savedSoilEmaPct = g_soilEmaPct;
  const bool savedSoilEmaInitialized = g_soilEmaInitialized;
  const uint8_t savedDebouncedRainDO = g_debouncedRainDO;
  const uint8_t savedPendingRainDO = g_pendingRainDO;
  const uint32_t savedRainPendingSinceMs = g_rainPendingSinceMs;
  const bool savedRainDebounceInitialized = g_rainDebounceInitialized;

  resetSensorPolicyForTest();

  bool pass = true;
  SensorData first{};
  first.temperatureC = 25.0f;
  first.humidityPct = 60.0f;
  first.lux = 1000.0f;
  first.soilAO = 4095;
  first.rainAO = 3000;
  first.soilDO = 1;
  first.rainDO = 1;
  first.dht_ok = true;
  first.bh1750_ok = true;
  first.soil_ok = true;
  first.rain_ok = true;
  applyValidityPolicy(first, 1000);
  pass &= first.soilPct < 0.1f;
  pass &= !first.rain_ok;
  Serial.print(F("SOIL_CAL raw="));
  Serial.print(SOIL_RAW_DRY);
  Serial.print(F(" pct="));
  Serial.println(soilRawToPct(SOIL_RAW_DRY), 1);

  SensorData second = first;
  second.soilAO = SOIL_RAW_WET;
  second.rainAO = 3000;
  second.rainDO = 1;
  second.rain_ok = true;
  applyValidityPolicy(second, 3000);
  pass &= second.soilPct > 24.9f && second.soilPct < 25.1f;
  pass &= second.rain_ok && second.rainDO == 1;
  Serial.print(F("SOIL_CAL raw="));
  Serial.print(SOIL_RAW_WET);
  Serial.print(F(" pct="));
  Serial.println(soilRawToPct(SOIL_RAW_WET), 1);
  Serial.print(F("RAIN_AO_CAL dry_raw="));
  Serial.print(RAIN_RAW_DRY);
  Serial.print(F(" dry_pct="));
  Serial.print(rainRawToPct(RAIN_RAW_DRY), 1);
  Serial.print(F(" wet_raw="));
  Serial.print(RAIN_RAW_WET);
  Serial.print(F(" wet_pct="));
  Serial.println(rainRawToPct(RAIN_RAW_WET), 1);

  SensorData rainSample = second;
  rainSample.rainDO = 0;
  rainSample.rain_ok = true;
  applyValidityPolicy(rainSample, 3500);
  pass &= !rainSample.rain_ok && rainSample.rainDO == 1;
  rainSample.rainDO = 0;
  rainSample.rain_ok = true;
  applyValidityPolicy(rainSample, 4600);
  pass &= rainSample.rain_ok && rainSample.rainDO == 0;

  SensorData staleSample = rainSample;
  staleSample.dht_ok = false;
  applyValidityPolicy(staleSample, 12000);
  pass &= !staleSample.dht_ok;
  pass &= staleSample.sensor_invalid;
  pass &= staleSample.temperatureC == first.temperatureC;

  g_lastValid = savedLastValid;
  g_soilEmaPct = savedSoilEmaPct;
  g_soilEmaInitialized = savedSoilEmaInitialized;
  g_debouncedRainDO = savedDebouncedRainDO;
  g_pendingRainDO = savedPendingRainDO;
  g_rainPendingSinceMs = savedRainPendingSinceMs;
  g_rainDebounceInitialized = savedRainDebounceInitialized;

  Serial.print(F("SENSOR_POLICY_SELF_TEST: "));
  Serial.println(pass ? F("PASS") : F("FAIL"));
  return pass;
}

}  // namespace SensorManager
