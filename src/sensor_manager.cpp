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

float adcToVoltage(const uint16_t raw) {
  return (static_cast<float>(raw) * ADC_REF_VOLTAGE) / static_cast<float>(ADC_MAX);
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
  Serial.print(F("("));
  Serial.print(soilV, 3);
  Serial.print(F("V)"));

  Serial.print(F(" RainAO:"));
  Serial.print(data.rainAO);
  Serial.print(F("("));
  Serial.print(rainV, 3);
  Serial.print(F("V)"));

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

}  // namespace SensorManager
