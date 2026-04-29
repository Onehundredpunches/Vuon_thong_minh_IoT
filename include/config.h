#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <Arduino.h>

#ifndef APP_MODE_SIMULATOR
#define APP_MODE_SIMULATOR 0
#endif

static constexpr uint32_t SERIAL_BAUD = 115200;
static constexpr uint32_t LOOP_INTERVAL_MS = 2000;

static constexpr uint8_t PIN_I2C_SDA = 21;
static constexpr uint8_t PIN_I2C_SCL = 22;
static constexpr uint8_t PIN_DHT22_DATA = 27;
static constexpr uint8_t PIN_SOIL_AO = 34;
static constexpr uint8_t PIN_SOIL_DO = 26;
static constexpr uint8_t PIN_RAIN_AO = 35;
static constexpr uint8_t PIN_RAIN_DO = 25;
static constexpr uint8_t PIN_SERVO_ROOF = 19;
static constexpr uint8_t PIN_RELAY_LIGHT = 18;
static constexpr uint8_t PIN_RELAY_FAN = 17;
static constexpr uint8_t PIN_RELAY_PUMP = 16;

static constexpr uint8_t LCD_I2C_ADDR = 0x27;
static constexpr uint8_t LCD_COLS = 16;
static constexpr uint8_t LCD_ROWS = 2;

static constexpr uint16_t ADC_MAX = 4095;
static constexpr float ADC_REF_VOLTAGE = 3.3f;

static constexpr uint16_t SOIL_DO_THRESHOLD_AO = 2000;
static constexpr uint16_t RAIN_DO_THRESHOLD_AO = 2000;

#endif
