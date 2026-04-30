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

static constexpr uint32_t SENSOR_SAMPLE_INTERVAL_MS = 2000;
static constexpr uint32_t SENSOR_MAX_STALE_MS = 6000;
static constexpr uint32_t RAIN_DEBOUNCE_MS = 1000;
static constexpr uint32_t RAIN_CLEAR_REOPEN_DELAY_MS = 30000;

static constexpr uint16_t SOIL_RAW_WET = 1260;
static constexpr uint16_t SOIL_RAW_DRY = 4095;
static constexpr uint16_t RAIN_RAW_WET = 1410;
static constexpr uint16_t RAIN_RAW_DRY = 4095;

static constexpr float AUTO_TEMP_FAN_ON_C = 28.0f;
static constexpr float AUTO_TEMP_FAN_OFF_C = 24.0f;
static constexpr float AUTO_SOIL_PUMP_ON_PCT = 35.0f;
static constexpr float AUTO_SOIL_PUMP_OFF_PCT = 55.0f;
static constexpr float AUTO_LIGHT_ON_LUX = 50.0f;
static constexpr float AUTO_LIGHT_OFF_LUX = 200.0f;
static constexpr int AUTO_ROOF_OPEN_ANGLE = 0;
static constexpr int AUTO_ROOF_SAFE_ANGLE = 90;

static constexpr const char *WIFI_PRIMARY_SSID = "Trang 72";
static constexpr const char *WIFI_PRIMARY_PASSWORD = "79797979";
static constexpr const char *MQTT_DEMO_HOST = "broker.hivemq.com";
static constexpr uint16_t MQTT_DEMO_PORT = 1883;
static constexpr const char *MQTT_DEMO_CLIENT_ID = "vuon-iot1";
static constexpr uint16_t MQTT_KEEP_ALIVE_SECONDS = 30;
static constexpr size_t MAX_MQTT_PAYLOAD_BYTES = 512;

#endif
