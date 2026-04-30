#include "auto_logic.h"

#include "actuator_manager.h"
#include "config.h"
#include "mode_controller.h"

namespace AutoLogic {
namespace {

uint32_t g_rainClearSinceMs = 0;
uint8_t g_pumpOnConfirm = 0;
uint8_t g_pumpOffConfirm = 0;
uint8_t g_lightOnConfirm = 0;
uint8_t g_lightOffConfirm = 0;
uint32_t g_pumpLastOnMs = 0;
uint32_t g_pumpLastOffMs = 0;
uint32_t g_pumpInvalidSinceMs = 0;
bool g_pumpInvalidHoldLogged = false;
bool g_pumpInvalidForceLogged = false;
enum class RoofState { Open, Closed };

RoofState g_roofState = RoofState::Closed;
uint32_t g_lastServoCmdMs = 0;
bool g_rainWasDetected = false;

static constexpr uint8_t AUTO_CONFIRM_SAMPLES = 2;
static constexpr uint32_t ROOF_MIN_COMMAND_INTERVAL_MS = 800;
static constexpr uint32_t AUTO_PUMP_MIN_ON_MS = 30000;
static constexpr uint32_t AUTO_PUMP_MIN_OFF_MS = 10000;
static constexpr uint32_t AUTO_PUMP_SOIL_INVALID_HOLD_MS = 8000;

bool elapsedAtLeast(const uint32_t nowMs, const uint32_t sinceMs, const uint32_t durationMs) {
  const int32_t elapsedMs = static_cast<int32_t>(nowMs - sinceMs);
  return elapsedMs >= 0 && static_cast<uint32_t>(elapsedMs) >= durationMs;
}

bool rainDetected(const SensorData &data) {
  return data.rain_ok && data.rainDO == 0;
}

const __FlashStringHelper *roofStateText() {
  return (g_roofState == RoofState::Open) ? F("OPEN") : F("CLOSED");
}

void printRainState(const SensorData &data) {
  Serial.print(F("RAIN_STATE do="));
  Serial.print(rainDetected(data) ? F("WET") : F("DRY"));
  Serial.print(F(" debounced="));
  Serial.println(data.rain_ok ? 1 : 0);
}

void printRoofState() {
  Serial.print(F("ROOF_STATE current="));
  Serial.print((g_roofState == RoofState::Open) ? F("O") : F("C"));
  Serial.print(F(" last_servo_cmd_ms="));
  Serial.print(g_lastServoCmdMs);
  Serial.print(F(" roof_motion_active="));
  Serial.println(ActuatorManager::roofMotionActive() ? 1 : 0);
}

void printRoofCmd(const int target, const bool sent, const __FlashStringHelper *reason) {
  const bool opening = target == AUTO_ROOF_OPEN_ANGLE;
  Serial.print(F("ROOF_CMD action="));
  Serial.print(opening ? F("OPEN") : F("CLOSE"));
  Serial.print(F(" cmd="));
  Serial.print(opening ? 70 : 110);
  Serial.print(F(" dur_ms="));
  Serial.print(1800);
  Serial.print(F(" sent="));
  Serial.print(sent ? 1 : 0);
  Serial.print(F(" reason="));
  Serial.print(reason);
  Serial.print(F(" last_servo_cmd_ms="));
  Serial.print(g_lastServoCmdMs);
  Serial.print(F(" roof_motion_active="));
  Serial.println(ActuatorManager::roofMotionActive() ? 1 : 0);
}

void printPumpDecision(const bool oldState, const bool newState, const SensorData &data,
                       const __FlashStringHelper *reason, const ActuatorCommandStatus status) {
  Serial.print(F("PUMP_DECISION old_state="));
  Serial.print(oldState ? F("ON") : F("OFF"));
  Serial.print(F(" new_state="));
  Serial.print(newState ? F("ON") : F("OFF"));
  Serial.print(F(" soilPct="));
  Serial.print(data.soilPct, 1);
  Serial.print(F(" rainDO="));
  Serial.print(data.rainDO);
  Serial.print(F(" rainAO="));
  Serial.print(data.rainAO);
  Serial.print(F(" mode="));
  Serial.print(ModeController::modeName());
  Serial.print(F(" reason="));
  Serial.print(reason);
  Serial.print(F(" status="));
  Serial.println(status == ActuatorCommandStatus::Ok ? F("ok") : ActuatorManager::statusReason(status));
}

bool sendRoofCommand(const int target, const uint32_t nowMs, const __FlashStringHelper *reason) {
  if (ActuatorManager::roofMotionActive()) {
    printRoofCmd(target, false, F("guard_block"));
    return false;
  }
  if (ActuatorManager::servoAngle() == target) {
    g_roofState = (target == AUTO_ROOF_OPEN_ANGLE) ? RoofState::Open : RoofState::Closed;
    printRoofCmd(target, false, F("no_change"));
    return false;
  }
  if (g_lastServoCmdMs != 0 && !elapsedAtLeast(nowMs, g_lastServoCmdMs, ROOF_MIN_COMMAND_INTERVAL_MS)) {
    printRoofCmd(target, false, F("guard_block"));
    return false;
  }
  const ActuatorCommandStatus status = ActuatorManager::requestServoAngleCommand(target, nowMs);
  const bool sent = status == ActuatorCommandStatus::Ok;
  if (sent) {
    g_lastServoCmdMs = nowMs;
    g_roofState = (target == AUTO_ROOF_OPEN_ANGLE) ? RoofState::Open : RoofState::Closed;
    printRoofCmd(target, true, reason);
  } else {
    printRoofCmd(target, false, F("guard_block"));
  }
  return sent;
}

void applyRainPolicy(const uint32_t nowMs, const SensorData &data) {
  printRainState(data);
  printRoofState();

  if (!data.rain_ok || rainDetected(data)) {
    const bool wet = rainDetected(data);
    g_rainClearSinceMs = 0;
    g_rainWasDetected = true;
    ActuatorManager::setRoofOpenInterlock(true);
    sendRoofCommand(AUTO_ROOF_SAFE_ANGLE, nowMs, wet ? F("rain_detected") : F("guard_block"));
    return;
  }

  if (g_rainClearSinceMs == 0) {
    g_rainClearSinceMs = nowMs;
  }

  if (g_rainWasDetected && elapsedAtLeast(nowMs, g_rainClearSinceMs, RAIN_CLEAR_REOPEN_DELAY_MS)) {
    ActuatorManager::setRoofOpenInterlock(false);
    if (sendRoofCommand(AUTO_ROOF_OPEN_ANGLE, nowMs, F("rain_clear"))) {
      g_rainWasDetected = false;
    }
  } else if (!g_rainWasDetected) {
    ActuatorManager::setRoofOpenInterlock(false);
    printRoofCmd(AUTO_ROOF_OPEN_ANGLE, false, F("no_change"));
  }
}

void applyPumpPolicy(const uint32_t nowMs, const SensorData &data) {
  if (!data.soil_ok) {
    g_pumpOnConfirm = 0;
    g_pumpOffConfirm = 0;
    if (g_pumpInvalidSinceMs == 0) {
      g_pumpInvalidSinceMs = nowMs;
      g_pumpInvalidHoldLogged = false;
      g_pumpInvalidForceLogged = false;
    }
    const bool oldState = ActuatorManager::pumpOn();
    if (!elapsedAtLeast(nowMs, g_pumpInvalidSinceMs, AUTO_PUMP_SOIL_INVALID_HOLD_MS)) {
      if (oldState && !g_pumpInvalidHoldLogged) {
        printPumpDecision(oldState, oldState, data, F("SOIL_INVALID_HOLD"), ActuatorCommandStatus::Ok);
        g_pumpInvalidHoldLogged = true;
      }
      return;
    }
    const ActuatorCommandStatus status = ActuatorManager::requestPump(false, nowMs);
    if (status == ActuatorCommandStatus::Ok) {
      g_pumpLastOffMs = nowMs;
    }
    if (oldState || status != ActuatorCommandStatus::Ok || !g_pumpInvalidForceLogged) {
      printPumpDecision(oldState, ActuatorManager::pumpOn(), data, F("SOIL_INVALID_FORCE_OFF"), status);
      g_pumpInvalidForceLogged = true;
    }
    return;
  }
  g_pumpInvalidSinceMs = 0;
  g_pumpInvalidHoldLogged = false;
  g_pumpInvalidForceLogged = false;
  if (!ActuatorManager::pumpOn()) {
    g_pumpOffConfirm = 0;
    if (data.soilPct < AUTO_SOIL_PUMP_ON_PCT) {
      ++g_pumpOnConfirm;
      Serial.print(F("AUTO_PUMP_CONFIRM reason=soil_dry pct="));
      Serial.print(data.soilPct, 1);
      Serial.print(F(" count="));
      Serial.println(g_pumpOnConfirm);
      if (g_pumpOnConfirm >= AUTO_CONFIRM_SAMPLES) {
        const bool offGuardClear = g_pumpLastOffMs == 0 || elapsedAtLeast(nowMs, g_pumpLastOffMs, AUTO_PUMP_MIN_OFF_MS);
        const bool oldState = ActuatorManager::pumpOn();
        const ActuatorCommandStatus status = offGuardClear ? ActuatorManager::requestPump(true, nowMs)
                                                           : ActuatorCommandStatus::CooldownActive;
        if (status == ActuatorCommandStatus::Ok) {
          g_pumpLastOnMs = nowMs;
        }
        printPumpDecision(oldState, ActuatorManager::pumpOn(), data,
                          offGuardClear ? F("SOIL_DRY_ON") : F("MIN_OFF_HOLD"), status);
        g_pumpOnConfirm = 0;
      }
    } else {
      g_pumpOnConfirm = 0;
    }
  } else {
    g_pumpOnConfirm = 0;
    if (data.soilPct > AUTO_SOIL_PUMP_OFF_PCT) {
      ++g_pumpOffConfirm;
      Serial.print(F("AUTO_PUMP_CONFIRM reason=soil_wet pct="));
      Serial.print(data.soilPct, 1);
      Serial.print(F(" count="));
      Serial.println(g_pumpOffConfirm);
      if (g_pumpOffConfirm >= AUTO_CONFIRM_SAMPLES) {
        const bool onGuardClear = g_pumpLastOnMs == 0 || elapsedAtLeast(nowMs, g_pumpLastOnMs, AUTO_PUMP_MIN_ON_MS);
        const bool oldState = ActuatorManager::pumpOn();
        const ActuatorCommandStatus status = onGuardClear ? ActuatorManager::requestPump(false, nowMs)
                                                          : ActuatorCommandStatus::CooldownActive;
        if (status == ActuatorCommandStatus::Ok) {
          g_pumpLastOffMs = nowMs;
        }
        printPumpDecision(oldState, ActuatorManager::pumpOn(), data,
                          onGuardClear ? F("SOIL_WET_OFF") : F("MIN_ON_HOLD"), status);
        g_pumpOffConfirm = 0;
      }
    } else {
      g_pumpOffConfirm = 0;
    }
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
    g_lightOnConfirm = 0;
    g_lightOffConfirm = 0;
    if (ActuatorManager::lightOn()) {
      Serial.print(F("LIGHT_DECISION lux="));
      Serial.print(data.lux, 1);
      Serial.println(F(" action=OFF reason=blocked"));
    }
    ActuatorManager::requestLight(false, nowMs);
    return;
  }
  if (!ActuatorManager::lightOn()) {
    g_lightOffConfirm = 0;
    if (data.lux < AUTO_LIGHT_ON_LUX) {
      ++g_lightOnConfirm;
      if (g_lightOnConfirm >= AUTO_CONFIRM_SAMPLES) {
        const ActuatorCommandStatus status = ActuatorManager::requestLight(true, nowMs);
        Serial.print(F("LIGHT_DECISION lux="));
        Serial.print(data.lux, 1);
        Serial.print(F(" action=ON reason="));
        Serial.print(status == ActuatorCommandStatus::Ok ? F("lux_on") : F("blocked"));
        Serial.print(F(" status="));
        Serial.println(status == ActuatorCommandStatus::Ok ? F("ok") : ActuatorManager::statusReason(status));
        g_lightOnConfirm = 0;
      }
    } else {
      g_lightOnConfirm = 0;
    }
  } else {
    g_lightOnConfirm = 0;
    if (data.lux > AUTO_LIGHT_OFF_LUX) {
      ++g_lightOffConfirm;
      if (g_lightOffConfirm >= AUTO_CONFIRM_SAMPLES) {
        const ActuatorCommandStatus status = ActuatorManager::requestLight(false, nowMs);
        Serial.print(F("LIGHT_DECISION lux="));
        Serial.print(data.lux, 1);
        Serial.print(F(" action=OFF reason="));
        Serial.print(status == ActuatorCommandStatus::Ok ? F("lux_off") : F("blocked"));
        Serial.print(F(" status="));
        Serial.println(status == ActuatorCommandStatus::Ok ? F("ok") : ActuatorManager::statusReason(status));
        g_lightOffConfirm = 0;
      }
    } else {
      g_lightOffConfirm = 0;
    }
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
  g_roofState = (ActuatorManager::servoAngle() == AUTO_ROOF_OPEN_ANGLE) ? RoofState::Open : RoofState::Closed;
  g_lastServoCmdMs = 0;
  g_rainWasDetected = false;
  g_pumpOnConfirm = 0;
  g_pumpOffConfirm = 0;
  g_pumpLastOnMs = 0;
  g_pumpLastOffMs = 0;
  g_pumpInvalidSinceMs = 0;
  g_pumpInvalidHoldLogged = false;
  g_pumpInvalidForceLogged = false;
  g_lightOnConfirm = 0;
  g_lightOffConfirm = 0;
}

void tick(const uint32_t nowMs, const SensorData &data) {
  if (ModeController::mode() != SystemMode::Auto) {
    if (data.bh1750_ok) {
      Serial.print(F("LIGHT_DECISION lux="));
      Serial.print(data.lux, 1);
      Serial.println(F(" action=OFF reason=manual"));
    }
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

  begin(1000);
  g_roofState = RoofState::Open;
  ActuatorManager::requestServoAngleCommand(AUTO_ROOF_OPEN_ANGLE, 1000);

  tick(1000, makeSample(29.0f, 3.0f, 30.0f, 1));
  ActuatorManager::updateServoSweep(3000);
  tick(3000, makeSample(29.0f, 3.0f, 30.0f, 1));
  pass &= ActuatorManager::fanOn();
  pass &= ActuatorManager::lightOn();
  pass &= ActuatorManager::pumpOn();

  tick(34000, makeSample(23.0f, 7.0f, 80.0f, 1));
  tick(36000, makeSample(23.0f, 7.0f, 80.0f, 1));
  pass &= !ActuatorManager::fanOn();
  pass &= !ActuatorManager::lightOn();
  pass &= !ActuatorManager::pumpOn();

  SensorData invalid = makeSample(30.0f, 1000.0f, 20.0f, 1);
  invalid.dht_ok = false;
  invalid.bh1750_ok = false;
  invalid.soil_ok = false;
  tick(38000, invalid);
  pass &= !ActuatorManager::fanOn();
  pass &= !ActuatorManager::lightOn();
  pass &= !ActuatorManager::pumpOn();

  tick(40000, makeSample(25.0f, 7.0f, 60.0f, 0));
  pass &= ActuatorManager::servoAngle() == AUTO_ROOF_SAFE_ANGLE;
  ActuatorManager::updateServoSweep(42000);
  tick(42000, makeSample(25.0f, 7.0f, 60.0f, 1));
  tick(42000 + RAIN_CLEAR_REOPEN_DELAY_MS + 1, makeSample(25.0f, 7.0f, 60.0f, 1));
  pass &= ActuatorManager::servoAngle() == AUTO_ROOF_OPEN_ANGLE;

  ActuatorManager::restoreSafeDefaults();
  begin(millis());
  ActuatorManager::setSelfTestMode(false);
  ModeController::begin(millis());

  Serial.print(F("AUTO_LOGIC_SELF_TEST: "));
  Serial.println(pass ? F("PASS") : F("FAIL"));
  return pass;
}

}  // namespace AutoLogic
