#ifndef SENSOR_TYPES_H
#define SENSOR_TYPES_H

#include <Arduino.h>

struct SensorData {
  float temperatureC;
  float humidityPct;
  float lux;
  float soilPct;
  uint16_t soilAO;
  uint16_t rainAO;
  uint8_t soilDO;
  uint8_t rainDO;
  bool sensor_invalid;
  bool dht_ok;
  bool bh1750_ok;
  bool soil_ok;
  bool rain_ok;
  uint32_t dht_last_valid_ms;
  uint32_t bh1750_last_valid_ms;
  uint32_t soil_last_valid_ms;
  uint32_t rain_last_valid_ms;
  uint32_t dht_max_stale_ms;
  uint32_t bh1750_max_stale_ms;
  uint32_t soil_max_stale_ms;
  uint32_t rain_max_stale_ms;
};

#endif
