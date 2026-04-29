#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>

#include "sensor_types.h"

namespace SensorManager {

void begin();
SensorData read(uint32_t sampleIndex);
void printCompactBlock(const SensorData &data);
bool runPolicySelfTest();

}  // namespace SensorManager

#endif
