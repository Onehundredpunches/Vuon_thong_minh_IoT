#include "auto_logic.h"

#include "actuator_manager.h"
#include "config.h"
#include "mode_controller.h"

namespace AutoLogic {
namespace {

uint32_t g_rainClearSinceMs = 0;
bool g_roofClosedForRain = false;

bool elapsedAtLeast(const uint32_t nowMs, const uint32_t sinceMs, const uint32_t durationMs) {
  const int32_t elapsedMs = static_cast<int32_t>(nowMs - sinceMs);
  return elapsedMs >= 0 && static_cast<uint32_t>(elapsedMs) >= durationMs;
}

bool rainDetected(const SensorData &data) {
  return data.rain_ok && data.rainDO == 0;
}

void applyRainPolicy(const uint32_t nowMs, const SensorData &data) {
  if (!data.rain_ok) {
    g_rainClearSinceMs = 0;
    g_roofClosedForRain = true;
    ActuatorManager::setRoofOpenInterlock(true);
    ActuatorManager::requestServoAngleCommand(AUTO_ROOF_SAFE_ANGLE, nowMs);
    return;
  }

  if (rainDetected(data)) {
    g_rainClearSinceMs = 0;
    g_roofClosedForRain = true;
    ActuatorManager::setRoofOpenInterlock(true);
    ActuatorManager::requestServoAngleCommand(AUTO_ROOF_SAFE_ANGLE, nowMs);
    return;
  }

  if (g_rainClearSinceMs == 0) {
    g_rainClearSinceMs = nowMs;
  }

  if (g_roofClosedForRain && elapsedAtLeast(nowMs, g_rainClearSinceMs, RAIN_CLEAR_REOPEN_DELAY_MS)) {
    ActuatorManager::setRoofOpenInterlock(false);
    ActuatorManager::requestServoAngleCommand(AUTO_ROOF_OPEN_ANGLE, nowMs);
    g_roofClosedForRain = false;
  } else if (!g_roofClosedForRain) {
    ActuatorManager::setRoofOpenInterlock(false);
  }
}

void applyPumpPolicy(const uint32_t nowMs, const SensorData &data) {
  if (!data.soil_ok) {
    ActuatorManager::requestPump(false, nowMs);
    return;
  }
  if (!ActuatorManager::pumpOn() && data.soilPct < AUTO_SOIL_PUMP_ON_PCT) {
    ActuatorManager::requestPump(true, nowMs);
  } else if (ActuatorManager::pumpOn() && data.soilPct > AUTO_SOIL_PUMP_OFF_PCT) {
    ActuatorManager::requestPump(false, nowMs);
  }
}

void applyFanPolicy(const uint32_t nowMs, const SensorData &data) {
  if (!data.dht_ok) {
    ActuatorManager::requestFan(false, nowMs);
    return;
  }
  if (!ActuatorManager::fanOn() && data.temperatureC > AUTO_TEMP_FAN_ON_C) {
    ActuatorManager::requestFan(true, nowMs);
  } else if (ActuatorManager::fanOn() && data.temperatureC < AUTO_TEMP_FAN_OFF_C) {
    ActuatorManager::requestFan(false, nowMs);
  }
}

void applyLightPolicy(const uint32_t nowMs, const SensorData &data) {
  if (!data.bh1750_ok) {
    ActuatorManager::requestLight(false, nowMs);
    return;
  }
  if (!ActuatorManager::lightOn() && data.lux < AUTO_LIGHT_ON_LUX) {
    ActuatorManager::requestLight(true, nowMs);
  } else if (ActuatorManager::lightOn() && data.lux > AUTO_LIGHT_OFF_LUX) {
    ActuatorManager::requestLight(false, nowMs);
  }
}

SensorData makeSample(const float tempC, const float lux, const float soilPct, const uint8_t rainDO) {
  SensorData data{};
  data.temperatureC = tempC;
  data.humidityPct = 65.0f;
  data.lux = lux;
  data.soilPct = soilPct;
  data.rainDO = rainDO;
  data.dht_ok = true;
  data.bh1750_ok = true;
  data.soil_ok = true;
  data.rain_ok = true;
  return data;
}

}  // namespace

void begin(const uint32_t nowMs) {
  g_rainClearSinceMs = nowMs;
  g_roofClosedForRain = false;
}

void tick(const uint32_t nowMs, const SensorData &data) {
  if (ModeController::mode() != SystemMode::Auto) {
    return;
  }

  applyRainPolicy(nowMs, data);
  applyPumpPolicy(nowMs, data);
  applyFanPolicy(nowMs, data);
  applyLightPolicy(nowMs, data);
}

const char *controlOwner() {
  return (ModeController::mode() == SystemMode::Auto) ? "AUTO" : "BE";
}

bool runSelfTest() {
  const SystemMode savedMode = ModeController::mode();
  const char *savedReason = ModeController::lastReason();
  const uint32_t savedManualActivity = ModeController::lastManualActivityMs();
  (void)savedMode;
  (void)savedReason;
  (void)savedManualActivity;

  bool pass = true;
  ActuatorManager::setSelfTestMode(true);
  ActuatorManager::restoreSafeDefaults();
  ModeController::setAuto("auto_logic_test", 1000);

  tick(1000, makeSample(29.0f, 2000.0f, 30.0f, 1));
  pass &= ActuatorManager::fanOn();
  pass &= ActuatorManager::lightOn();
  pass &= ActuatorManager::pumpOn();

  tick(2000, makeSample(23.0f, 9000.0f, 80.0f, 1));
  pass &= !ActuatorManager::fanOn();
  pass &= !ActuatorManager::lightOn();
  pass &= !ActuatorManager::pumpOn();

  SensorData invalid = makeSample(30.0f, 1000.0f, 20.0f, 1);
  invalid.dht_ok = false;
  invalid.bh1750_ok = false;
  invalid.soil_ok = false;
  tick(3000, invalid);
  pass &= !ActuatorManager::fanOn();
  pass &= !ActuatorManager::lightOn();
  pass &= !ActuatorManager::pumpOn();

  tick(4000, makeSample(25.0f, 4000.0f, 60.0f, 0));
  pass &= ActuatorManager::servoAngle() == AUTO_ROOF_SAFE_ANGLE;
  tick(5000, makeSample(25.0f, 4000.0f, 60.0f, 1));
  tick(5000 + RAIN_CLEAR_REOPEN_DELAY_MS + 1, makeSample(25.0f, 4000.0f, 60.0f, 1));
  pass &= ActuatorManager::servoAngle() == AUTO_ROOF_OPEN_ANGLE;

  ActuatorManager::restoreSafeDefaults();
  ActuatorManager::setSelfTestMode(false);
  ModeController::begin(millis());

  Serial.print(F("AUTO_LOGIC_SELF_TEST: "));
  Serial.println(pass ? F("PASS") : F("FAIL"));
  return pass;
}

}  // namespace AutoLogic
