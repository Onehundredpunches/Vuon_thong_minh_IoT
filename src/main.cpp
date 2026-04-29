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

void processSerialCommand(char *cmd) {
  if (strcmp(cmd, "servo off") == 0) {
    ActuatorManager::setServoOnOff(false);
    ActuatorManager::printServoAck(cmd);
    return;
  }
  if (strcmp(cmd, "servo on") == 0) {
    ActuatorManager::setServoOnOff(true);
    ActuatorManager::printServoAck(cmd);
    return;
  }
  if (strcmp(cmd, "servo sweep on") == 0) {
    const ActuatorCommandStatus status = ActuatorManager::requestServoSweep(true, millis());
    if (status != ActuatorCommandStatus::Ok) {
      ActuatorManager::printCommandReject(cmd, status);
      return;
    }
    ActuatorManager::printServoAck(cmd);
    return;
  }
  if (strcmp(cmd, "servo sweep off") == 0 || strcmp(cmd, "servo stop") == 0) {
    const ActuatorCommandStatus status = ActuatorManager::requestServoSweep(false, millis());
    if (status != ActuatorCommandStatus::Ok) {
      ActuatorManager::printCommandReject(cmd, status);
      return;
    }
    ActuatorManager::printServoAck(cmd);
    return;
  }

  int targetAngle = -1;
  if (sscanf(cmd, "servo %d", &targetAngle) == 1 &&
      (targetAngle == 0 || targetAngle == 90 || targetAngle == 180)) {
    const ActuatorCommandStatus status = ActuatorManager::requestServoAngleCommand(targetAngle, millis());
    if (status != ActuatorCommandStatus::Ok) {
      ActuatorManager::printCommandReject(cmd, status);
      return;
    }
    ActuatorManager::printServoAck(cmd);
    return;
  }

  if (strcmp(cmd, "light on") == 0) {
    const ActuatorCommandStatus status = ActuatorManager::requestLight(true, millis());
    if (status != ActuatorCommandStatus::Ok) {
      ActuatorManager::printCommandReject(cmd, status);
      return;
    }
    ActuatorManager::printRelayCommandLog(cmd, "LIGHT", PIN_RELAY_LIGHT, true);
    ActuatorManager::printLightState();
    return;
  }
  if (strcmp(cmd, "light off") == 0) {
    const ActuatorCommandStatus status = ActuatorManager::requestLight(false, millis());
    if (status != ActuatorCommandStatus::Ok) {
      ActuatorManager::printCommandReject(cmd, status);
      return;
    }
    ActuatorManager::printRelayCommandLog(cmd, "LIGHT", PIN_RELAY_LIGHT, false);
    ActuatorManager::printLightState();
    return;
  }
  if (strcmp(cmd, "light toggle") == 0) {
    const bool targetOn = !ActuatorManager::lightOn();
    const ActuatorCommandStatus status = ActuatorManager::requestLight(targetOn, millis());
    if (status != ActuatorCommandStatus::Ok) {
      ActuatorManager::printCommandReject(cmd, status);
      return;
    }
    ActuatorManager::printRelayCommandLog(cmd, "LIGHT", PIN_RELAY_LIGHT, targetOn);
    ActuatorManager::printLightState();
    return;
  }
  if (strcmp(cmd, "light blink") == 0) {
    const ActuatorCommandStatus status = ActuatorManager::requestLightBlink(millis());
    if (status != ActuatorCommandStatus::Ok) {
      ActuatorManager::printCommandReject(cmd, status);
      return;
    }
    ActuatorManager::printRelayCommandLog(cmd, "LIGHT", PIN_RELAY_LIGHT, true);
    ActuatorManager::printLightState();
    return;
  }
  if (strcmp(cmd, "fan on") == 0) {
    const ActuatorCommandStatus status = ActuatorManager::requestFan(true, millis());
    if (status != ActuatorCommandStatus::Ok) {
      ActuatorManager::printCommandReject(cmd, status);
      return;
    }
    ActuatorManager::printRelayCommandLog(cmd, "FAN", PIN_RELAY_FAN, true);
    ActuatorManager::printFanState();
    return;
  }
  if (strcmp(cmd, "fan off") == 0) {
    const ActuatorCommandStatus status = ActuatorManager::requestFan(false, millis());
    if (status != ActuatorCommandStatus::Ok) {
      ActuatorManager::printCommandReject(cmd, status);
      return;
    }
    ActuatorManager::printRelayCommandLog(cmd, "FAN", PIN_RELAY_FAN, false);
    ActuatorManager::printFanState();
    return;
  }
  if (strcmp(cmd, "pump on") == 0) {
    const ActuatorCommandStatus status = ActuatorManager::requestPump(true, millis());
    if (status != ActuatorCommandStatus::Ok) {
      ActuatorManager::printCommandReject(cmd, status);
      return;
    }
    ActuatorManager::printRelayCommandLog(cmd, "PUMP", PIN_RELAY_PUMP, true);
    ActuatorManager::printPumpState();
    return;
  }
  if (strcmp(cmd, "pump off") == 0) {
    const ActuatorCommandStatus status = ActuatorManager::requestPump(false, millis());
    if (status != ActuatorCommandStatus::Ok) {
      ActuatorManager::printCommandReject(cmd, status);
      return;
    }
    ActuatorManager::printRelayCommandLog(cmd, "PUMP", PIN_RELAY_PUMP, false);
    ActuatorManager::printPumpState();
    return;
  }

  Serial.println(F("CMD: UNKNOWN"));
}

void executeCommandForTest(const char *cmd) {
  char local[48] = {0};
  strncpy(local, cmd, sizeof(local) - 1);
  processSerialCommand(local);
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

  printStartup();
  ActuatorManager::printLightState();
  ActuatorManager::printFanState();
  ActuatorManager::printPumpState();
  scanI2cBus();

  initLcdIfPresent();
  ActuatorManager::initServoControl();
  SerialCLI::begin(processSerialCommand);
  SelfTest::run(executeCommandForTest);
  SerialCLI::printHelp();

  g_nextSampleMs = millis();
}

void loop() {
  const uint32_t now = millis();
  SerialCLI::tick();
  ActuatorManager::updateServoSweep(now);
  ActuatorManager::updateLightBlink(now);
  ActuatorManager::tickSafety(now);

  if (static_cast<int32_t>(now - g_nextSampleMs) < 0) {
    return;
  }
  g_nextSampleMs += LOOP_INTERVAL_MS;

  SensorData data = SensorManager::read(g_sampleIndex);

  SensorManager::printCompactBlock(data);
  updateLcd(data);

  ++g_sampleIndex;
}
