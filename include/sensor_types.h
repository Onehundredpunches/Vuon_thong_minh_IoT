#ifndef SENSOR_TYPES_H
#define SENSOR_TYPES_H

#include <Arduino.h>

struct SensorData {
  float temperatureC;
  float humidityPct;
  float lux;
  uint16_t soilAO;
  uint16_t rainAO;
  uint8_t soilDO;
  uint8_t rainDO;
  bool dht_ok;
  bool bh1750_ok;
  bool soil_ok;
  bool rain_ok;
};

#endif
