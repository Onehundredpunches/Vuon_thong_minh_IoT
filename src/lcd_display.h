#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include "sensor_types.h"

void initLcdIfPresent();
void updateLcd(const SensorData &data);

#endif
