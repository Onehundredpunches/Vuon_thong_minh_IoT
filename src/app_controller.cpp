#include "app_controller.h"

#include <Arduino.h>
#include <Wire.h>

#include "actuator_manager.h"
#include "auto_logic.h"
#include "command_handler.h"
#include "config.h"
#include "lcd_display.h"
#include "mode_controller.h"
#include "mqtt_manager.h"
#include "nvs_store.h"
#include "self_test.h"
#include "sensor_manager.h"
#include "serial_cli.h"
#include "time_sync.h"

namespace AppController {
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
  Serial.print(F("I2C_SCAN: "));
  int count = 0;
  for (uint8_t i = 1; i < 127; i++) {
    if (probeI2cAddress(i)) {
      Serial.print(F("0x"));
      if (i < 16) {
        Serial.print('0');
      }
      Serial.print(i, HEX);
      Serial.print(' ');
      count++;
    }
  }
  if (count == 0) {
    Serial.println(F("none"));
  } else {
    Serial.println();
  }
}

void printStartup() {
  Serial.println(F("========================================"));
  Serial.println(F("       SMART GARDEN IOT - BOOT"));
  Serial.println(F("========================================"));
  printPinMap();
  Serial.println(F("SYSTEM_MODE: INIT"));
  Serial.print(F("APP_MODE_SIMULATOR="));
  Serial.println(APP_MODE_SIMULATOR);
}

void printBootPhase(const char *phase) {
  Serial.print(F("BOOT_PHASE phase="));
  Serial.print(phase);
  Serial.print(F(" ms="));
  Serial.println(millis());
}

void printModeRestore(const bool valid, const SystemMode mode, const int roofAngle, const char *source) {
  Serial.print(F("MODE_RESTORE valid="));
  Serial.print(valid ? 1 : 0);
  Serial.print(F(" mode="));
  switch (mode) {
    case SystemMode::Auto:
      Serial.print(F("AUTO"));
      break;
    case SystemMode::Manual:
      Serial.print(F("MANUAL"));
      break;
    case SystemMode::SafeStop:
      Serial.print(F("SAFE_STOP"));
      break;
    case SystemMode::Error:
      Serial.print(F("ERROR"));
      break;
    case SystemMode::Boot:
      Serial.print(F("BOOT"));
      break;
  }
  Serial.print(F(" roof="));
  Serial.print(roofAngle);
  Serial.print(F(" source="));
  Serial.println(source);
}

SensorData g_lastSensorData{};

// --- NVS auto-save state tracking ---
SystemMode g_lastSavedMode = SystemMode::Boot;
bool g_lastSavedPump = false;
bool g_lastSavedFan = false;
bool g_lastSavedLight = false;
int g_lastSavedRoofAngle = AUTO_ROOF_OPEN_ANGLE; // Default to OPEN per user request

void checkAndPersistState(const SensorData &data, const ActuatorSnapshot &act) {
  const SystemMode currentMode = ModeController::mode();
  const bool roofChanged = act.servoAngle != g_lastSavedRoofAngle;
  const bool stateChanged =
      currentMode != g_lastSavedMode ||
      act.pumpOn != g_lastSavedPump ||
      act.fanOn != g_lastSavedFan ||
      act.lightOn != g_lastSavedLight ||
      roofChanged;

  if (!stateChanged) {
    return;
  }

  const bool modeChanged = currentMode != g_lastSavedMode;

  g_lastSavedMode = currentMode;
  g_lastSavedPump = act.pumpOn;
  g_lastSavedFan = act.fanOn;
  g_lastSavedLight = act.lightOn;
  g_lastSavedRoofAngle = act.servoAngle;

  NvsStore::buildCurrentState(
      currentMode, act.pumpOn, act.fanOn, act.lightOn,
      act.servoAngle, data.soilPct, data.temperatureC);
  NvsStore::markDirty();

  if (modeChanged || currentMode == SystemMode::Manual || roofChanged) {
    NvsStore::savePending();
    Serial.println(F("NVS_FORCE_SAVE: immediate write"));
  }

  Serial.print(F("NVS_DIRTY: mode="));
  Serial.print(ModeController::modeName());
  Serial.print(F(" pump="));
  Serial.print(act.pumpOn);
  Serial.print(F(" fan="));
  Serial.print(act.fanOn);
  Serial.print(F(" light="));
  Serial.print(act.lightOn);
  Serial.print(F(" roof="));
  Serial.println(act.servoAngle);
}

void tickFastTasks(const uint32_t now) {
  SerialCLI::tick();
  ActuatorManager::setAutoSafetyEnabled(ModeController::mode() == SystemMode::Auto);
  ActuatorManager::updateServoSweep(now);
  ActuatorManager::updateLightBlink(now);
  ActuatorManager::tickSafety(now);
  ModeController::tick(now);
  MqttManager::tick(now);
  
  if (ModeController::mode() == SystemMode::Manual) {
    checkAndPersistState(g_lastSensorData, ActuatorManager::snapshot());
  }

  NvsStore::tickCoalescedWrite(now);
}

void tickSampleTasks(const uint32_t now) {
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

  g_lastSensorData = data;
  checkAndPersistState(data, lcdState.actuators);

  ++g_sampleIndex;
}

}  // namespace

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);
  printBootPhase("setup_start");

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  SensorManager::begin();
  ActuatorManager::begin();

  NvsStore::begin();
  printBootPhase("nvs_load_start");
  NvsStore::PersistedState nvsState{};
  const bool nvsValid = NvsStore::load(nvsState);
  printBootPhase("nvs_load_end");

  const int initialRoof = nvsValid ? nvsState.roofAngle : AUTO_ROOF_OPEN_ANGLE;
  ActuatorManager::initServoControl(initialRoof);
  g_lastSavedRoofAngle = initialRoof;
  printBootPhase("servo_seed");

  ModeController::begin(millis());
  MqttManager::begin(millis());

  printStartup();
  ActuatorManager::printLightState();
  ActuatorManager::printFanState();
  ActuatorManager::printPumpState();
  scanI2cBus();

  initLcdIfPresent();
  SerialCLI::begin(CommandHandler::handle);

  printBootPhase("self_test_start");
  const bool selfTestPass = SelfTest::run(CommandHandler::executeForTest);
  printBootPhase(selfTestPass ? "self_test_pass" : "self_test_fail");

  // Restore NVS state after self-test pollution
  ActuatorManager::initServoControl(initialRoof);
  g_lastSavedRoofAngle = initialRoof;
  printBootPhase("restore_start");

  if (!selfTestPass) {
    Serial.println(F("ERROR_REASON reason=self_test_fail"));
    ModeController::enterError("self_test_fail", millis());
    printModeRestore(nvsValid, SystemMode::Error, initialRoof, nvsValid ? "nvs" : "default");
  } else if (nvsValid) {
    const SystemMode savedMode = static_cast<SystemMode>(nvsState.mode);
    if (savedMode == SystemMode::Manual) {
      if (ModeController::setManual("nvs_restore", millis())) {
        Serial.println(F("BOOT_RESTORE: mode=MANUAL from NVS"));
        printModeRestore(true, SystemMode::Manual, nvsState.roofAngle, "nvs");
        const uint32_t nowMs = millis();
        ActuatorManager::requestPump(nvsState.pumpOn > 0, nowMs);
        ActuatorManager::requestFan(nvsState.fanOn > 0, nowMs);
        ActuatorManager::requestLight(nvsState.lightOn > 0, nowMs);
        ActuatorManager::requestServoAngleCommand(nvsState.roofAngle, nowMs);
      } else {
        ModeController::setAuto("nvs_restore_fallback", millis());
        Serial.println(F("BOOT_RESTORE: MANUAL rejected, fallback AUTO"));
        printModeRestore(true, SystemMode::Auto, initialRoof, "nvs");
      }
    } else {
      ModeController::setAuto("nvs_restore", millis());
      Serial.println(F("BOOT_RESTORE: mode=AUTO from NVS"));
      printModeRestore(true, SystemMode::Auto, nvsState.roofAngle, "nvs");
    }
  } else {
    ModeController::setAuto("first_boot", millis());
    Serial.println(F("BOOT_RESTORE: mode=AUTO (first boot/default)"));
    printModeRestore(false, SystemMode::Auto, initialRoof, "default");
  }
  AutoLogic::begin(millis());
  printBootPhase("restore_end");
  SerialCLI::printHelp();

  TimeSync::begin();

  g_nextSampleMs = millis();
  printBootPhase("ready");
}

void loop() {
  const uint32_t now = millis();
  tickFastTasks(now);
  if (static_cast<int32_t>(now - g_nextSampleMs) < 0) {
    return;
  }
  g_nextSampleMs += LOOP_INTERVAL_MS;
  tickSampleTasks(now);
}

}  // namespace AppController
