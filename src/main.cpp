#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "sensor_types.h"
#include "actuator_types.h"
#include "system_mode.h"
#include "pins_validate.h"
#include "log_config.h"
#include "simulator.h"
#include "lcd_display.h"
#include "actuator_manager.h"
#include "sensor_manager.h"
#include "serial_cli.h"
#include "self_test.h"
#include "command_handler.h"
#include "mode_controller.h"
#include "auto_logic.h"
#include "mqtt_manager.h"

namespace {

uint32_t g_nextSampleMs = 0;
uint32_t g_sampleIndex = 0;

bool probeI2cAddress(const uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void printPinMap() {
  Serial.println(F("PINMAP I2C_SDA=21 I2C_SCL=22 DHT=27 SoilAO=34 SoilDO=26 RainAO=35 RainDO=25 SERVO=19 LIGHT=18 FAN=17 PUMP=16"));
}

void scanI2cBus() {
  Serial.print(F("I2C_SCAN"));
  bool foundAny = false;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    if (probeI2cAddress(addr)) {
      Serial.print(foundAny ? ',' : ' ');
      if (addr < 16) {
        Serial.print('0');
      }
      Serial.print(addr, HEX);
      foundAny = true;
    }
  }
  if (!foundAny) {
    Serial.print(F(" NONE"));
  }
  Serial.println();
}

void printStartup() {
#if APP_MODE_SIMULATOR
  Serial.println(F("MODE: SIMULATOR"));
#else
  Serial.println(F("MODE: HARDWARE"));
#endif
  printPinMap();
  ActuatorManager::printRelayActiveLevel();
}

}  // namespace

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  SensorManager::begin();
  ActuatorManager::begin();
  ModeController::begin(millis());
  AutoLogic::begin(millis());
  MqttManager::begin(millis());

  printStartup();
  ActuatorManager::printLightState();
  ActuatorManager::printFanState();
  ActuatorManager::printPumpState();
  scanI2cBus();

  initLcdIfPresent();
  ActuatorManager::initServoControl();
  SerialCLI::begin(CommandHandler::handle);
  const bool selfTestPass = SelfTest::run(CommandHandler::executeForTest);
  if (selfTestPass) {
    ModeController::setAuto("self_test_pass", millis());
  } else {
    ModeController::enterError("self_test_fail", millis());
  }
  SerialCLI::printHelp();

  g_nextSampleMs = millis();
}

void loop() {
  const uint32_t now = millis();
  SerialCLI::tick();
  ActuatorManager::updateServoSweep(now);
  ActuatorManager::updateLightBlink(now);
  ActuatorManager::tickSafety(now);
  ModeController::tick(millis());
  MqttManager::tick(millis());

  if (static_cast<int32_t>(now - g_nextSampleMs) < 0) {
    return;
  }
  g_nextSampleMs += LOOP_INTERVAL_MS;

  SensorData data = SensorManager::read(g_sampleIndex);
  AutoLogic::tick(now, data);

  SensorManager::printCompactBlock(data);
  LcdDisplayState lcdState{};
  lcdState.sensors = data;
  lcdState.actuators = ActuatorManager::snapshot();
  lcdState.mode = ModeController::mode();
  lcdState.wifiOk = MqttManager::wifiConnected();
  lcdState.mqttOk = MqttManager::mqttConnected();
  lcdState.controlOwner = AutoLogic::controlOwner();
  updateLcd(now, lcdState);
  MqttManager::updateState(now, data, lcdState.actuators);

  ++g_sampleIndex;
}
