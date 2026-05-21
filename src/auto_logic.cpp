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
SystemMode g_lastAutoPumpOwnerMode = SystemMode::Boot;

// --- Pump State Machine ---
enum class PumpState : uint8_t {
  Idle,          // OFF, waiting for dry trigger
  Running,       // ON, irrigating
  SafetyPause,   // After safety timeout, waiting cooldown before retry
  Locked         // Max retries exceeded, needs wet-soil reset
};
PumpState g_pumpState = PumpState::Idle;
uint8_t g_pumpSafetyRetryCount = 0;
static constexpr uint8_t PUMP_MAX_SAFETY_RETRIES = 3;
enum class RoofState { Open, Closed };

RoofState g_roofState = RoofState::Open;
uint32_t g_lastServoCmdMs = 0;
bool g_rainWasDetected = false;

static constexpr uint8_t AUTO_CONFIRM_SAMPLES = 2;
static constexpr uint32_t ROOF_MIN_COMMAND_INTERVAL_MS = 800;
static constexpr uint32_t AUTO_PUMP_MIN_ON_MS = 5000;
static constexpr uint32_t AUTO_PUMP_MIN_OFF_MS = 10000;
static constexpr uint32_t AUTO_PUMP_SOIL_INVALID_HOLD_MS = 8000;
static constexpr uint32_t BOOT_GUARD_MS = 2000; // 2 seconds stabilization
uint32_t g_bootGuardUntilMs = 0;

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
  Serial.print(F("RAIN_STATE state="));
  if (!data.rain_ok) {
    Serial.print(F("UNKNOWN"));
  } else {
    Serial.print(rainDetected(data) ? F("WET") : F("DRY"));
  }
  Serial.print(F(" valid="));
  Serial.print(data.rain_ok ? 1 : 0);
  Serial.print(F(" rainDO="));
  Serial.println(data.rainDO);
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
  Serial.print(F("ROOF_CMD_DECISION reason="));
  Serial.print(reason);
  Serial.print(F(" action="));
  Serial.print(opening ? F("OPEN") : F("CLOSE"));
  Serial.print(F(" target="));
  Serial.print(target);
  Serial.print(F(" cmd="));
  Serial.print(opening ? 70 : 110);
  Serial.print(F(" dur_ms="));
  Serial.print(1800);
  Serial.print(F(" sent="));
  Serial.print(sent ? 1 : 0);
  Serial.print(F(" last_servo_cmd_ms="));
  Serial.print(g_lastServoCmdMs);
  Serial.print(F(" roof_motion_active="));
  Serial.println(ActuatorManager::roofMotionActive() ? 1 : 0);
}

void printPumpDecision(const bool oldState, const bool newState, const SensorData &data,
                       const __FlashStringHelper *reason, const ActuatorCommandStatus status) {
  const bool sensorInvalid = !data.soil_ok;
  Serial.print(F("PUMP_DECISION mode="));
  Serial.print(ModeController::modeName());
  Serial.print(F(" soil_ok="));
  Serial.print(data.soil_ok ? 1 : 0);
  Serial.print(F(" sensor_invalid="));
  Serial.print(sensorInvalid ? 1 : 0);
  Serial.print(F(" soilPct="));
  Serial.print(data.soilPct, 1);
  Serial.print(F(" soilDO="));
  Serial.print(data.soilDO);
  Serial.print(F(" old_state="));
  Serial.print(oldState ? F("ON") : F("OFF"));
  Serial.print(F(" new_state="));
  Serial.print(newState ? F("ON") : F("OFF"));
  Serial.print(F(" rainDO="));
  Serial.print(data.rainDO);
  Serial.print(F(" rainAO="));
  Serial.print(data.rainAO);
  Serial.print(F(" reason="));
  Serial.print(reason);
  Serial.print(F(" status="));
  Serial.println(status == ActuatorCommandStatus::Ok ? F("ok") : ActuatorManager::statusReason(status));
}

void resetAutoPumpRuntimeState() {
  g_pumpOnConfirm = 0;
  g_pumpOffConfirm = 0;
  g_pumpLastOnMs = 0;
  g_pumpLastOffMs = 0;
  g_pumpInvalidSinceMs = 0;
  g_pumpInvalidHoldLogged = false;
  g_pumpInvalidForceLogged = false;
  g_pumpState = PumpState::Idle;
  g_pumpSafetyRetryCount = 0;
  ActuatorManager::clearPumpSafetyTimeoutFlag();
  ActuatorManager::resetPumpSafetyTimeoutCount();
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

  // Hold the roof while rain is not valid. Closing is allowed only for valid wet rain.
  if (!data.rain_ok) {
    if (nowMs < g_bootGuardUntilMs) {
      return;
    }
    printRoofCmd(ActuatorManager::servoAngle(), false, F("rain_sensor_invalid_hold"));
    return;
  }

  if (rainDetected(data)) {
    g_rainClearSinceMs = 0;
    g_rainWasDetected = true;
    ActuatorManager::setRoofOpenInterlock(true);
    sendRoofCommand(AUTO_ROOF_SAFE_ANGLE, nowMs, F("rain_detected"));
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
  // --- Handle sensor invalid path (unchanged logic) ---
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
      if (g_pumpState == PumpState::Running) {
        g_pumpState = PumpState::Idle;
      }
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

  // --- Check for safety timeout from tickSafety() ---
  if (ActuatorManager::pumpSafetyTimeoutFlag()) {
    ActuatorManager::clearPumpSafetyTimeoutFlag();
    g_pumpLastOffMs = nowMs;
    ++g_pumpSafetyRetryCount;
    if (g_pumpSafetyRetryCount >= PUMP_MAX_SAFETY_RETRIES) {
      g_pumpState = PumpState::Locked;
      printPumpDecision(true, false, data, F("SAFETY_LOCKED"), ActuatorCommandStatus::Ok);
      Serial.print(F("PUMP_SM state=Locked retries="));
      Serial.println(g_pumpSafetyRetryCount);
    } else {
      g_pumpState = PumpState::SafetyPause;
      printPumpDecision(true, false, data, F("SAFETY_PAUSE"), ActuatorCommandStatus::Ok);
      Serial.print(F("PUMP_SM state=SafetyPause retry="));
      Serial.print(g_pumpSafetyRetryCount);
      Serial.print(F("/"));
      Serial.println(PUMP_MAX_SAFETY_RETRIES);
    }
    g_pumpOnConfirm = 0;
    g_pumpOffConfirm = 0;
    return;
  }

  // --- Wet-soil reset: clear lockout/pause if soil is wet enough ---
  if (data.soilPct > AUTO_SOIL_PUMP_OFF_PCT &&
      (g_pumpState == PumpState::Locked || g_pumpState == PumpState::SafetyPause)) {
    printPumpDecision(false, false, data, F("WET_RESET"), ActuatorCommandStatus::Ok);
    g_pumpState = PumpState::Idle;
    g_pumpSafetyRetryCount = 0;
    g_pumpOnConfirm = 0;
    g_pumpOffConfirm = 0;
    return;
  }

  // --- State machine transitions ---
  switch (g_pumpState) {
    case PumpState::Locked: {
      // Locked: do nothing until wet-soil reset (handled above)
      g_pumpOnConfirm = 0;
      return;
    }

    case PumpState::SafetyPause: {
      // Wait for cooldown, then transition to Idle to allow retry
      g_pumpOnConfirm = 0;
      if (data.soilPct < AUTO_SOIL_PUMP_ON_PCT) {
        const bool cooldownClear = g_pumpLastOffMs == 0 ||
                                   elapsedAtLeast(nowMs, g_pumpLastOffMs, AUTO_PUMP_MIN_OFF_MS);
        if (cooldownClear) {
          g_pumpState = PumpState::Idle;
          Serial.println(F("PUMP_SM SafetyPause->Idle cooldown_clear"));
        }
      }
      return;
    }

    case PumpState::Idle: {
      // Pump is OFF, check if soil is dry enough to turn ON
      g_pumpOffConfirm = 0;
      if (data.soilPct < AUTO_SOIL_PUMP_ON_PCT) {
        ++g_pumpOnConfirm;
        Serial.print(F("AUTO_PUMP_CONFIRM reason=soil_dry pct="));
        Serial.print(data.soilPct, 1);
        Serial.print(F(" count="));
        Serial.println(g_pumpOnConfirm);
        if (g_pumpOnConfirm >= AUTO_CONFIRM_SAMPLES) {
          const bool offGuardClear = g_pumpLastOffMs == 0 ||
                                     elapsedAtLeast(nowMs, g_pumpLastOffMs, AUTO_PUMP_MIN_OFF_MS);
          const bool oldState = ActuatorManager::pumpOn();
          const ActuatorCommandStatus status = offGuardClear
                                                 ? ActuatorManager::requestPump(true, nowMs)
                                                 : ActuatorCommandStatus::CooldownActive;
          if (status == ActuatorCommandStatus::Ok) {
            g_pumpLastOnMs = nowMs;
            g_pumpState = PumpState::Running;
          }
          printPumpDecision(oldState, ActuatorManager::pumpOn(), data,
                            offGuardClear ? F("SOIL_DRY_ON") : F("MIN_OFF_HOLD"), status);
          g_pumpOnConfirm = 0;
        }
      } else {
        g_pumpOnConfirm = 0;
      }
      return;
    }

    case PumpState::Running: {
      // Pump is ON, check if soil is wet enough to turn OFF
      g_pumpOnConfirm = 0;
      if (!ActuatorManager::pumpOn()) {
        // External kill (shouldn't happen if safety flag handled above)
        g_pumpState = PumpState::Idle;
        g_pumpLastOffMs = nowMs;
        printPumpDecision(true, false, data, F("EXTERNAL_OFF"), ActuatorCommandStatus::Ok);
        return;
      }
      if (data.soilPct > AUTO_SOIL_PUMP_OFF_PCT) {
        ++g_pumpOffConfirm;
        Serial.print(F("AUTO_PUMP_CONFIRM reason=soil_wet pct="));
        Serial.print(data.soilPct, 1);
        Serial.print(F(" count="));
        Serial.println(g_pumpOffConfirm);
        if (g_pumpOffConfirm >= AUTO_CONFIRM_SAMPLES) {
          const bool onGuardClear = g_pumpLastOnMs == 0 ||
                                    elapsedAtLeast(nowMs, g_pumpLastOnMs, AUTO_PUMP_MIN_ON_MS);
          const bool oldState = ActuatorManager::pumpOn();
          const ActuatorCommandStatus status = onGuardClear
                                                 ? ActuatorManager::requestPump(false, nowMs)
                                                 : ActuatorCommandStatus::CooldownActive;
          if (status == ActuatorCommandStatus::Ok) {
            g_pumpLastOffMs = nowMs;
            g_pumpState = PumpState::Idle;
            g_pumpSafetyRetryCount = 0;
          }
          printPumpDecision(oldState, ActuatorManager::pumpOn(), data,
                            onGuardClear ? F("SOIL_WET_OFF") : F("MIN_ON_HOLD"), status);
          g_pumpOffConfirm = 0;
        }
      } else {
        g_pumpOffConfirm = 0;
      }
      return;
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
  resetAutoPumpRuntimeState();
  g_lastAutoPumpOwnerMode = ModeController::mode();
  g_lightOnConfirm = 0;
  g_lightOffConfirm = 0;
  g_bootGuardUntilMs = nowMs + BOOT_GUARD_MS;
}

void tick(const uint32_t nowMs, const SensorData &data) {
  const SystemMode currentMode = ModeController::mode();
  if (currentMode != g_lastAutoPumpOwnerMode) {
    if (currentMode == SystemMode::Auto || g_lastAutoPumpOwnerMode == SystemMode::Auto) {
      resetAutoPumpRuntimeState();
    }
    g_lastAutoPumpOwnerMode = currentMode;
  }

  if (currentMode != SystemMode::Auto) {
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

  // --- T01: Dry soil turns pump ON ---
  begin(1000);
  g_roofState = RoofState::Open;
  ActuatorManager::requestServoAngleCommand(AUTO_ROOF_OPEN_ANGLE, 1000);

  tick(1000, makeSample(29.0f, 3.0f, 30.0f, 1));
  ActuatorManager::updateServoSweep(3000);
  tick(3000, makeSample(29.0f, 3.0f, 30.0f, 1));
  pass &= ActuatorManager::fanOn();
  pass &= ActuatorManager::lightOn();
  pass &= ActuatorManager::pumpOn();

  // --- T02: Wet soil turns pump OFF ---
  tick(34000, makeSample(23.0f, 7.0f, 80.0f, 1));
  tick(36000, makeSample(23.0f, 7.0f, 80.0f, 1));
  pass &= !ActuatorManager::fanOn();
  pass &= !ActuatorManager::lightOn();
  pass &= !ActuatorManager::pumpOn();

  // --- T05: Sensor invalid ---
  SensorData invalid = makeSample(30.0f, 1000.0f, 20.0f, 1);
  invalid.dht_ok = false;
  invalid.bh1750_ok = false;
  invalid.soil_ok = false;
  tick(38000, invalid);
  pass &= !ActuatorManager::fanOn();
  pass &= !ActuatorManager::lightOn();
  pass &= !ActuatorManager::pumpOn();

  // --- T06: Boot rain invalid/dry must not close an open roof ---
  ActuatorManager::restoreSafeDefaults();
  ActuatorManager::requestServoAngleCommand(AUTO_ROOF_OPEN_ANGLE, 39000);
  begin(39000);
  SensorData invalidRain = makeSample(25.0f, 1000.0f, 60.0f, 0);
  invalidRain.rain_ok = false;
  tick(39000, invalidRain);
  pass &= ActuatorManager::servoAngle() == AUTO_ROOF_OPEN_ANGLE;
  tick(41000, makeSample(25.0f, 1000.0f, 60.0f, 1));
  pass &= ActuatorManager::servoAngle() == AUTO_ROOF_OPEN_ANGLE;
  tick(43000, makeSample(25.0f, 1000.0f, 60.0f, 0));
  pass &= ActuatorManager::servoAngle() == AUTO_ROOF_SAFE_ANGLE;
  Serial.println(F("T06_BOOT_RAIN_GUARD: invalid/dry holds open, valid wet closes"));

  // --- Rain roof test (unchanged) ---
  tick(45000, makeSample(25.0f, 7.0f, 60.0f, 0));
  pass &= ActuatorManager::servoAngle() == AUTO_ROOF_SAFE_ANGLE;
  ActuatorManager::updateServoSweep(47000);
  tick(47000, makeSample(25.0f, 7.0f, 60.0f, 1));
  tick(47000 + RAIN_CLEAR_REOPEN_DELAY_MS + 1, makeSample(25.0f, 7.0f, 60.0f, 1));
  pass &= ActuatorManager::servoAngle() == AUTO_ROOF_OPEN_ANGLE;

  // --- T03: Safety timeout → pump re-engages (THE BUG FIX TEST) ---
  ActuatorManager::restoreSafeDefaults();
  begin(100000);
  // Pump ON with dry soil (confirm at t=100000, ON at t=102000)
  tick(100000, makeSample(29.0f, 500.0f, 10.0f, 1));
  tick(102000, makeSample(29.0f, 500.0f, 10.0f, 1));
  pass &= ActuatorManager::pumpOn();
  Serial.println(F("T03_PUMP_ON_CHECK: pump should be ON"));

  // Simulate safety timeout: pump ON since t=102000, timeout at 102000+60001
  ActuatorManager::tickSafety(102000 + 60001);
  pass &= !ActuatorManager::pumpOn();
  pass &= ActuatorManager::pumpSafetyTimeoutFlag();
  Serial.println(F("T03_SAFETY_TIMEOUT: pump killed by tickSafety"));

  // Next auto tick detects flag → SafetyPause, sets pumpLastOffMs=163000
  tick(163000, makeSample(29.0f, 500.0f, 10.0f, 1));
  pass &= !ActuatorManager::pumpOn();
  pass &= (g_pumpState == PumpState::SafetyPause);
  Serial.println(F("T03_SAFETY_PAUSE: pump in SafetyPause state"));

  // After cooldown (10s from t=163000) → Idle
  tick(174000, makeSample(29.0f, 500.0f, 10.0f, 1));
  pass &= (g_pumpState == PumpState::Idle);
  Serial.println(F("T03_COOLDOWN_CLEAR: pump back to Idle"));

  // Pump re-engages: confirm at t=175000, ON at t=177000
  tick(175000, makeSample(29.0f, 500.0f, 10.0f, 1));
  tick(177000, makeSample(29.0f, 500.0f, 10.0f, 1));
  pass &= ActuatorManager::pumpOn();
  Serial.println(F("T03_RE_ENGAGE: pump ON again after safety timeout"));

  // --- T04: Max retries → Locked ---
  // 2nd safety timeout: pump ON since t=177000
  ActuatorManager::tickSafety(177000 + 60001);
  tick(238000, makeSample(29.0f, 500.0f, 10.0f, 1));
  pass &= (g_pumpState == PumpState::SafetyPause);

  // Cooldown (10s from t=238000) + re-engage
  tick(249000, makeSample(29.0f, 500.0f, 10.0f, 1));
  tick(250000, makeSample(29.0f, 500.0f, 10.0f, 1));
  tick(252000, makeSample(29.0f, 500.0f, 10.0f, 1));
  pass &= ActuatorManager::pumpOn();

  // 3rd safety timeout → Locked: pump ON since t=252000
  ActuatorManager::tickSafety(252000 + 60001);
  tick(313000, makeSample(29.0f, 500.0f, 10.0f, 1));
  pass &= (g_pumpState == PumpState::Locked);
  pass &= !ActuatorManager::pumpOn();
  Serial.println(F("T04_LOCKED: pump locked after max retries"));

  // Wet soil resets lockout
  tick(315000, makeSample(29.0f, 500.0f, 60.0f, 1));
  pass &= (g_pumpState == PumpState::Idle);
  Serial.println(F("T04_WET_RESET: pump unlocked by wet soil"));

  ActuatorManager::restoreSafeDefaults();
  begin(millis());
  ActuatorManager::setSelfTestMode(false);

  Serial.print(F("AUTO_LOGIC_SELF_TEST: "));
  Serial.println(pass ? F("PASS") : F("FAIL"));
  return pass;
}

}  // namespace AutoLogic
