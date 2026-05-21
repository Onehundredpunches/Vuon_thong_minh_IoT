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
static constexpr uint32_t RAIN_CLEAR_REOPEN_DELAY_MS = 10000;

static constexpr uint16_t SOIL_RAW_WET = 1260;
static constexpr uint16_t SOIL_RAW_DRY = 4095;
static constexpr uint16_t RAIN_RAW_WET = 1410;
static constexpr uint16_t RAIN_RAW_DRY = 4095;

static constexpr float AUTO_TEMP_FAN_ON_C = 28.0f;
static constexpr float AUTO_TEMP_FAN_OFF_C = 24.0f;
static constexpr float AUTO_SOIL_PUMP_ON_PCT = 35.0f;
static constexpr float AUTO_SOIL_PUMP_OFF_PCT = 55.0f;
static constexpr float AUTO_LIGHT_ON_LUX = 4.0f;
static constexpr float AUTO_LIGHT_OFF_LUX = 6.0f;
static constexpr float AIR_HUMIDITY_HIGH_PCT = 85.0f;
static constexpr float AIR_HUMIDITY_LOW_PCT = 45.0f;

// Continuous-rotation roof servo tuning.
// ESP32Servo::write() uses 0..180 command units for pulse width; for a 360 servo
// these are speed/direction commands, not positional angles.
static constexpr int SERVO_STOP_CMD = 90;
static constexpr int SERVO_OPEN_CMD = 70;
static constexpr int SERVO_CLOSE_CMD = 110;

// Approximate open-loop timing for the roof's short 80-degree-equivalent travel.
// Start with these values, then tune on real hardware using MQTT open/close commands.
static constexpr float ROOF_ESTIMATED_DEG_PER_SEC = 25.0f;
static constexpr float ROOF_TRAVEL_DEG_EQUIV = 80.0f;
static constexpr uint32_t ROOF_ESTIMATED_TRAVEL_MS =
    static_cast<uint32_t>((ROOF_TRAVEL_DEG_EQUIV * 1000.0f) / ROOF_ESTIMATED_DEG_PER_SEC);
// Close timing calibrated from real hardware: 260 ms moved about 30 deg-equiv,
// so 520 ms moves about 60 deg-equiv. Scale close 520 * 80 / 60 = 693 ms.
// Open at 500 ms overshot about 5 deg from an 80-deg close, so estimate
// 500 ms ~= 85 deg actual; 500 * 80 / 85 = 471 ms. Use 470 ms.
static constexpr uint32_t ROOF_MOVE_OPEN_MS = 470;
static constexpr uint32_t ROOF_MOVE_CLOSE_MS = 690;

// Logical roof state markers kept for compatibility with NVS, MQTT, CLI, and tests.
// They are not physical servo angles; a continuous-rotation servo has no position feedback.
static constexpr int AUTO_ROOF_OPEN_ANGLE = 0;
static constexpr int AUTO_ROOF_SAFE_ANGLE = 1;

static constexpr const char *WIFI_PRIMARY_SSID = "pekkunu";
static constexpr const char *WIFI_PRIMARY_PASSWORD = "44444444";
static constexpr const char *WIFI_FALLBACK_SSID = "San bat cuop Thu Duc";
static constexpr const char *WIFI_FALLBACK_PASSWORD = "Truong@11t";
static constexpr uint32_t WIFI_PRIMARY_CONNECT_TIMEOUT_MS = 15000;
static constexpr const char *MQTT_DEMO_HOST = "broker.hivemq.com";
static constexpr uint16_t MQTT_DEMO_PORT = 1883;
static constexpr const char *MQTT_DEMO_CLIENT_ID = "vuon-iot1";
static constexpr uint16_t MQTT_KEEP_ALIVE_SECONDS = 30;
static constexpr size_t MAX_MQTT_PAYLOAD_BYTES = 1024;

#endif
