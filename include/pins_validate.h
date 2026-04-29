#ifndef PINS_VALIDATE_H
#define PINS_VALIDATE_H

#include "config.h"

static_assert(PIN_I2C_SDA == 21, "Schematic A3 pin mismatch: SDA_I2C");
static_assert(PIN_I2C_SCL == 22, "Schematic A3 pin mismatch: SCL_I2C");
static_assert(PIN_DHT22_DATA == 27, "Schematic A3 pin mismatch: DHT22_PIN");
static_assert(PIN_SOIL_AO == 34, "Schematic A3 pin mismatch: SOIL_AO_PIN");
static_assert(PIN_SOIL_DO == 26, "Schematic A3 pin mismatch: SOIL_DO_PIN");
static_assert(PIN_RAIN_AO == 35, "Schematic A3 pin mismatch: RAIN_AO_PIN");
static_assert(PIN_RAIN_DO == 25, "Schematic A3 pin mismatch: RAIN_DO_PIN");
static_assert(PIN_SERVO_ROOF == 19, "Schematic A3 pin mismatch: SERVO_PIN");
static_assert(PIN_RELAY_LIGHT == 18, "Schematic A3 pin mismatch: LIGHT_PIN");
static_assert(PIN_RELAY_FAN == 17, "Schematic A3 pin mismatch: FAN_PIN");
static_assert(PIN_RELAY_PUMP == 16, "Schematic A3 pin mismatch: PUMP_PIN");

#endif
