#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include "actuator_types.h"
#include "sensor_types.h"
#include "system_mode.h"

struct LcdDisplayState {
  SensorData sensors;
  ActuatorSnapshot actuators;
  SystemMode mode;
  bool wifiOk;
  bool mqttOk;
  const char *controlOwner;
};

void initLcdIfPresent();
void updateLcd(uint32_t nowMs, const LcdDisplayState &state);
bool lcdDeviceAvailable();
bool runLcdFormatterSelfTest();

#endif
