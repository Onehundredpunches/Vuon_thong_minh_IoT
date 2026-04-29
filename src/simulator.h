#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <Arduino.h>

#include "sensor_types.h"

void readSimulator(SensorData &data, uint32_t sampleIndex);

#endif
