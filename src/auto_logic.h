#ifndef AUTO_LOGIC_H
#define AUTO_LOGIC_H

#include <Arduino.h>

#include "sensor_types.h"

namespace AutoLogic {

void begin(uint32_t nowMs);
void tick(uint32_t nowMs, const SensorData &data);
const char *controlOwner();
bool runSelfTest();

}  // namespace AutoLogic

#endif
