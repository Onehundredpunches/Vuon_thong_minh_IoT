#include "self_test.h"

#include <Arduino.h>
#include <string.h>

#include "actuator_manager.h"
#include "auto_logic.h"
#include "command_handler.h"
#include "config.h"
#include "lcd_display.h"
#include "mode_controller.h"
#include "mqtt_manager.h"
#include "sensor_manager.h"

namespace SelfTest {
namespace {

static constexpr bool ENABLE_BOOT_SELF_TEST = true;

bool g_relaySelfTestPass = false;
bool g_lightSelfTestPass = false;
bool g_fanSelfTestPass = false;
bool g_pumpSelfTestPass = false;
bool g_servoSelfTestPass = false;
bool g_pinMapSelfTestPass = false;
bool g_actuatorSafetySelfTestPass = false;
bool g_sensorPolicySelfTestPass = false;
bool g_commandHandlerSelfTestPass = false;
bool g_modeFsmSelfTestPass = false;
bool g_lcdFormatterSelfTestPass = false;
bool g_autoLogicSelfTestPass = false;
bool g_mqttReconnectSelfTestPass = false;
bool g_mqttCommandAckSelfTestPass = false;
bool g_mqttRetainedStateSelfTestPass = false;
bool g_fullIntegrationSelfTestPass = false;

bool reportTestStep(const char *group, const char *step, const bool pass) {
  Serial.print(group);
  Serial.print(F(" "));
  Serial.print(step);
  Serial.print(F(": "));
  Serial.println(pass ? F("PASS") : F("FAIL"));
  return pass;
}

void runRelaySelfTest(const CommandExecutor executeCommand) {
  bool allPass = true;
  ActuatorManager::stopLightBlink();
  ActuatorManager::setRelayLight(false);
  g_lightSelfTestPass = reportTestStep("LIGHT_TEST", "boot_off", !ActuatorManager::lightOn());
  allPass &= g_lightSelfTestPass;

  executeCommand("light on");
  g_lightSelfTestPass &= reportTestStep("LIGHT_TEST", "light_on", ActuatorManager::lightOn());
  allPass &= g_lightSelfTestPass;

  executeCommand("light off");
  g_lightSelfTestPass &= reportTestStep("LIGHT_TEST", "light_off", !ActuatorManager::lightOn());
  allPass &= g_lightSelfTestPass;

  executeCommand("light toggle");
  g_lightSelfTestPass &= reportTestStep("LIGHT_TEST", "toggle_on", ActuatorManager::lightOn());
  allPass &= g_lightSelfTestPass;

  executeCommand("light toggle");
  g_lightSelfTestPass &= reportTestStep("LIGHT_TEST", "toggle_off", !ActuatorManager::lightOn());
  allPass &= g_lightSelfTestPass;

  executeCommand("light blink");
  ActuatorManager::completeLightBlinkForTest();
  g_lightSelfTestPass &= reportTestStep("LIGHT_TEST", "blink_complete_off",
                                        !ActuatorManager::lightBlinkActive() && !ActuatorManager::lightOn());
  allPass &= g_lightSelfTestPass;

  ActuatorManager::setRelayFan(false);
  g_fanSelfTestPass = reportTestStep("FAN_TEST", "boot_off", !ActuatorManager::fanOn());
  executeCommand("fan on");
  g_fanSelfTestPass &= reportTestStep("FAN_TEST", "fan_on", ActuatorManager::fanOn());
  executeCommand("fan off");
  g_fanSelfTestPass &= reportTestStep("FAN_TEST", "fan_off", !ActuatorManager::fanOn());
  allPass &= g_fanSelfTestPass;

  ActuatorManager::setRelayPump(false);
  g_pumpSelfTestPass = reportTestStep("PUMP_TEST", "boot_off", !ActuatorManager::pumpOn());
  executeCommand("pump on");
  g_pumpSelfTestPass &= reportTestStep("PUMP_TEST", "pump_on", ActuatorManager::pumpOn());
  executeCommand("pump off");
  g_pumpSelfTestPass &= reportTestStep("PUMP_TEST", "pump_off", !ActuatorManager::pumpOn());
  allPass &= g_pumpSelfTestPass;

  Serial.print(F("LIGHT_SELF_TEST: "));
  Serial.println(g_lightSelfTestPass ? F("PASS") : F("FAIL"));
  Serial.print(F("FAN_SELF_TEST: "));
  Serial.println(g_fanSelfTestPass ? F("PASS") : F("FAIL"));
  Serial.print(F("PUMP_SELF_TEST: "));
  Serial.println(g_pumpSelfTestPass ? F("PASS") : F("FAIL"));

  g_relaySelfTestPass = allPass;
  Serial.print(F("RELAY_SELF_TEST: "));
  Serial.println(allPass ? F("PASS") : F("FAIL"));
}

void runServoSelfTest(const CommandExecutor executeCommand) {
  bool allPass = true;

  executeCommand("servo on");
  allPass &= reportTestStep("SERVO_TEST", "servo_on", ActuatorManager::servoEnabled());

  executeCommand("servo sweep on");
  allPass &= reportTestStep("SERVO_TEST", "sweep_on",
                            ActuatorManager::servoEnabled() && ActuatorManager::servoSweepEnabled());

  executeCommand("servo sweep off");
  allPass &= reportTestStep("SERVO_TEST", "sweep_off", !ActuatorManager::servoSweepEnabled());

  executeCommand("servo 180");
  allPass &= reportTestStep("SERVO_TEST", "angle_180",
                            ActuatorManager::servoAngle() == 180 && !ActuatorManager::servoSweepEnabled());

  executeCommand("servo off");
  allPass &= reportTestStep("SERVO_TEST", "servo_off",
                            !ActuatorManager::servoEnabled() && ActuatorManager::servoDetachedOrSelfTest());

  g_servoSelfTestPass = allPass;
  Serial.print(F("SERVO_SELF_TEST: "));
  Serial.println(g_servoSelfTestPass ? F("PASS") : F("FAIL"));
}

void runPinMapSelfTest() {
  const bool pass =
      PIN_I2C_SDA == 21 &&
      PIN_I2C_SCL == 22 &&
      PIN_DHT22_DATA == 27 &&
      PIN_SOIL_AO == 34 &&
      PIN_SOIL_DO == 26 &&
      PIN_RAIN_AO == 35 &&
      PIN_RAIN_DO == 25 &&
      PIN_SERVO_ROOF == 19 &&
      PIN_RELAY_LIGHT == 18 &&
      PIN_RELAY_FAN == 17 &&
      PIN_RELAY_PUMP == 16;

  g_pinMapSelfTestPass = reportTestStep("PINMAP_TEST", "schematic_a3", pass);
}

bool runFullIntegrationSelfTest() {
  bool pass = true;
  ActuatorManager::setSelfTestMode(true);
  ActuatorManager::restoreSafeDefaults();
  ModeController::begin(1000);
  pass &= ModeController::setAuto("integration_test", 1100);

  CommandHandler::ExecuteResult result = CommandHandler::executeStructured("light on", false, 1200);
  pass &= result.status == CommandHandler::ExecuteStatus::Rejected &&
          strcmp(result.reason, "invalid_mode") == 0;

  pass &= ModeController::setManual("integration_test", 1300);
  result = CommandHandler::executeStructured("light on", false, 1400);
  pass &= result.status == CommandHandler::ExecuteStatus::Ok && ActuatorManager::lightOn();

  pass &= ModeController::setAuto("integration_test", 1500);
  SensorData invalid{};
  invalid.temperatureC = 31.0f;
  invalid.humidityPct = 70.0f;
  invalid.lux = 1000.0f;
  invalid.soilPct = 20.0f;
  invalid.rainDO = 1;
  invalid.dht_ok = false;
  invalid.bh1750_ok = false;
  invalid.soil_ok = false;
  invalid.rain_ok = false;
  invalid.sensor_invalid = true;
  AutoLogic::tick(1600, invalid);
  pass &= !ActuatorManager::lightOn();
  pass &= !ActuatorManager::fanOn();
  pass &= !ActuatorManager::pumpOn();
  pass &= ActuatorManager::servoAngle() == AUTO_ROOF_SAFE_ANGLE;

  ModeController::enterSafeStop("integration_fault", 1700);
  result = CommandHandler::executeStructured("pump on", false, 1800);
  pass &= result.status == CommandHandler::ExecuteStatus::Rejected &&
          strcmp(result.reason, "sensor_fault_active") == 0;

  ActuatorManager::restoreSafeDefaults();
  ActuatorManager::setSelfTestMode(false);
  ModeController::begin(millis());
  Serial.print(F("FULL_INTEGRATION_SELF_TEST: "));
  Serial.println(pass ? F("PASS") : F("FAIL"));
  return pass;
}

}  // namespace

bool run(const CommandExecutor executeCommand) {
  if (!ENABLE_BOOT_SELF_TEST) {
    return true;
  }

  Serial.println(F("SELF_TEST: START"));
  ActuatorManager::setSelfTestMode(true);
  runPinMapSelfTest();
  runRelaySelfTest(executeCommand);
  runServoSelfTest(executeCommand);
  ActuatorManager::setSelfTestMode(false);
  g_actuatorSafetySelfTestPass = ActuatorManager::runSafetySelfTest();
  g_sensorPolicySelfTestPass = SensorManager::runPolicySelfTest();
  g_commandHandlerSelfTestPass = CommandHandler::runParserSelfTest();
  g_modeFsmSelfTestPass = ModeController::runSelfTest();
  g_autoLogicSelfTestPass = AutoLogic::runSelfTest();
  g_mqttReconnectSelfTestPass = MqttManager::runReconnectSelfTest();
  g_mqttCommandAckSelfTestPass = MqttManager::runCommandAckSelfTest();
  g_mqttRetainedStateSelfTestPass = MqttManager::runRetainedStateSelfTest();
  g_fullIntegrationSelfTestPass = runFullIntegrationSelfTest();
  g_lcdFormatterSelfTestPass = runLcdFormatterSelfTest();
  Serial.print(F("LCD_DEVICE_SELF_TEST: "));
  Serial.println(lcdDeviceAvailable() ? F("READY") : F("NOT_READY"));

  ActuatorManager::restoreSafeDefaults();

  const bool allPass = g_pinMapSelfTestPass && g_relaySelfTestPass && g_servoSelfTestPass &&
                       g_actuatorSafetySelfTestPass && g_sensorPolicySelfTestPass &&
                       g_commandHandlerSelfTestPass && g_modeFsmSelfTestPass &&
                       g_autoLogicSelfTestPass && g_mqttReconnectSelfTestPass &&
                       g_mqttCommandAckSelfTestPass && g_mqttRetainedStateSelfTestPass &&
                       g_fullIntegrationSelfTestPass && g_lcdFormatterSelfTestPass;
  Serial.print(F("SELF_TEST: "));
  Serial.println(allPass ? F("PASS") : F("FAIL"));
  return allPass;
}

}  // namespace SelfTest
