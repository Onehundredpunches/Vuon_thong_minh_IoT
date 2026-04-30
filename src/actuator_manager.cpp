#include "actuator_manager.h"

#include <ESP32Servo.h>

#include "config.h"

namespace ActuatorManager {
namespace {

Servo g_roofServo;

const bool RELAY_ACTIVE_HIGH = true;

static constexpr int SERVO_MIN_ANGLE = 0;
static constexpr int SERVO_MAX_ANGLE = 180;
static constexpr int SERVO_STOP = 90;
static constexpr int SERVO_OPEN_CMD = 70;
static constexpr int SERVO_CLOSE_CMD = 110;
static constexpr uint32_t ROOF_MOVE_OPEN_MS = 1800;
static constexpr uint32_t ROOF_MOVE_CLOSE_MS = 1800;
static constexpr int SERVO_STEP_DEG = 2;
static constexpr uint32_t SERVO_STEP_INTERVAL_MS = 20;
static constexpr uint8_t LIGHT_BLINK_COUNT = 5;
static constexpr uint32_t LIGHT_BLINK_INTERVAL_MS = 200;
static constexpr uint32_t PUMP_MAX_ON_MS = 60000;
static constexpr uint32_t PUMP_COOLDOWN_MS = 10000;
static constexpr uint32_t FAN_MAX_ON_MS = 1800000;
static constexpr uint32_t FAN_COOLDOWN_MS = 10000;
static constexpr uint32_t LIGHT_COOLDOWN_MS = 5000;

bool g_selfTestMode = false;
bool g_servoEnabled = true;
bool g_servoSweepEnabled = false;
int g_servoAngle = AUTO_ROOF_SAFE_ANGLE;
int g_servoPulse = SERVO_STOP;
int g_servoDirection = 1;
uint32_t g_nextServoStepMs = 0;
bool g_roofMotionActive = false;
uint32_t g_roofMotionStopMs = 0;
bool g_autoSafetyEnabled = true;
bool g_lightOn = false;
bool g_fanOn = false;
bool g_pumpOn = false;
bool g_lightBlinkActive = false;
uint8_t g_lightBlinkTransitionsRemaining = 0;
uint32_t g_nextLightBlinkMs = 0;
uint32_t g_lightCooldownUntilMs = 0;
uint32_t g_fanOnSinceMs = 0;
uint32_t g_fanCooldownUntilMs = 0;
uint32_t g_pumpOnSinceMs = 0;
uint32_t g_pumpCooldownUntilMs = 0;
bool g_roofOpenInterlock = false;

bool attachServoIfNeeded() {
#if APP_MODE_SIMULATOR
  return false;
#else
  if (g_selfTestMode) {
    return true;
  }
  if (!g_roofServo.attached()) {
    g_roofServo.setPeriodHertz(50);
    g_roofServo.attach(PIN_SERVO_ROOF, 500, 2500);
  }
  return true;
#endif
}

void writeServoAngle() {
#if APP_MODE_SIMULATOR
  return;
#else
  if (!g_servoEnabled) {
    return;
  }
  if (attachServoIfNeeded()) {
    g_roofServo.write(g_servoPulse);
  }
#endif
}

void writeServoPulse(const int pulse) {
  g_servoPulse = constrain(pulse, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  writeServoAngle();
}

void writeRelayPin(const uint8_t pin, const bool on) {
  if (!g_selfTestMode) {
    const uint8_t level = (on == RELAY_ACTIVE_HIGH) ? HIGH : LOW;
    digitalWrite(pin, level);
  }
}

const __FlashStringHelper *relayGpioLevelName(const bool on) {
  const bool high = (on == RELAY_ACTIVE_HIGH);
  return high ? F("HIGH") : F("LOW");
}

void printRelayState(const char *name, const bool on) {
  Serial.print(name);
  Serial.print(F(": "));
  Serial.println(on ? F("ON") : F("OFF"));
}

bool cooldownActive(const uint32_t nowMs, const uint32_t cooldownUntilMs) {
  return static_cast<int32_t>(nowMs - cooldownUntilMs) < 0;
}

bool roofOpenAngle(const int angle) {
  return angle == AUTO_ROOF_OPEN_ANGLE;
}

bool elapsedAtLeast(const uint32_t nowMs, const uint32_t sinceMs, const uint32_t durationMs) {
  const int32_t elapsedMs = static_cast<int32_t>(nowMs - sinceMs);
  return elapsedMs >= 0 && static_cast<uint32_t>(elapsedMs) >= durationMs;
}

}  // namespace

void begin() {
  pinMode(PIN_RELAY_LIGHT, OUTPUT);
  pinMode(PIN_RELAY_FAN, OUTPUT);
  pinMode(PIN_RELAY_PUMP, OUTPUT);

  setRelayLight(false);
  setRelayFan(false);
  setRelayPump(false);
}

void setSelfTestMode(const bool enabled) {
  g_selfTestMode = enabled;
}

void printRelayActiveLevel() {
  Serial.print(F("RELAY_ACTIVE_LEVEL: "));
  Serial.println(RELAY_ACTIVE_HIGH ? F("ACTIVE_HIGH") : F("ACTIVE_LOW"));
}

void setRelayLight(const bool on) {
  g_lightOn = on;
  writeRelayPin(PIN_RELAY_LIGHT, on);
  if (!on) {
    stopLightBlink();
  }
}

void setRelayFan(const bool on) {
  const uint32_t nowMs = millis();
  g_fanOn = on;
  writeRelayPin(PIN_RELAY_FAN, on);
  if (on) {
    g_fanOnSinceMs = nowMs;
  } else {
    g_fanOnSinceMs = 0;
  }
}

void setRelayPump(const bool on) {
  const uint32_t nowMs = millis();
  g_pumpOn = on;
  writeRelayPin(PIN_RELAY_PUMP, on);
  if (on) {
    g_pumpOnSinceMs = nowMs;
  } else {
    g_pumpOnSinceMs = 0;
  }
}

void printLightState() {
  printRelayState("LIGHT", g_lightOn);
}

void printFanState() {
  printRelayState("FAN", g_fanOn);
}

void printPumpState() {
  printRelayState("PUMP", g_pumpOn);
}

void printRelayCommandLog(const char *cmd, const char *name, const uint8_t pin, const bool on) {
  Serial.print(F("ACK "));
  Serial.print(cmd);
  Serial.print(F(" | "));
  Serial.print(name);
  Serial.print('=');
  Serial.print(on ? F("ON") : F("OFF"));
  Serial.print(F(" GPIO"));
  Serial.print(pin);
  Serial.print('=');
  Serial.println(relayGpioLevelName(on));
}

void stopLightBlink() {
  g_lightBlinkActive = false;
  g_lightBlinkTransitionsRemaining = 0;
}

void startLightBlink() {
  stopLightBlink();
  setRelayLight(true);
  printLightState();
  g_lightBlinkActive = true;
  g_lightBlinkTransitionsRemaining = (LIGHT_BLINK_COUNT * 2) - 1;
  g_nextLightBlinkMs = millis() + LIGHT_BLINK_INTERVAL_MS;
}

void updateLightBlink(const uint32_t nowMs) {
  if (!g_lightBlinkActive) {
    return;
  }
  if (static_cast<int32_t>(nowMs - g_nextLightBlinkMs) < 0) {
    return;
  }
  g_nextLightBlinkMs += LIGHT_BLINK_INTERVAL_MS;

  setRelayLight(!g_lightOn);
  printLightState();

  if (g_lightBlinkTransitionsRemaining > 0) {
    --g_lightBlinkTransitionsRemaining;
  }
  if (g_lightBlinkTransitionsRemaining == 0) {
    g_lightBlinkActive = false;
    if (g_lightOn) {
      setRelayLight(false);
      printLightState();
    }
  }
}

void completeLightBlinkForTest() {
  uint32_t simNow = millis() + LIGHT_BLINK_INTERVAL_MS;
  uint8_t guard = 32;
  while (g_lightBlinkActive && guard > 0) {
    updateLightBlink(simNow);
    simNow += LIGHT_BLINK_INTERVAL_MS;
    --guard;
  }
}

void setServoOnOff(const bool on) {
#if APP_MODE_SIMULATOR
  g_servoEnabled = on;
  if (!g_servoEnabled) {
    g_servoSweepEnabled = false;
  }
#else
  g_servoEnabled = on;
  if (g_servoEnabled) {
    attachServoIfNeeded();
    writeServoAngle();
  } else {
    g_servoSweepEnabled = false;
    if (!g_selfTestMode && g_roofServo.attached()) {
      g_roofServo.detach();
    }
  }
#endif
}

void printServoAck(const char *cmd) {
  Serial.print(F("ACK "));
  Serial.print(cmd);
  Serial.print(F(" | SERVO="));
  Serial.print(g_servoEnabled ? F("ON") : F("OFF"));
  Serial.print(F(" ANGLE="));
  Serial.print(g_servoAngle);
  Serial.print(F(" PULSE="));
  Serial.print(g_servoPulse);
  Serial.print(F(" SWEEP="));
  Serial.print(g_servoSweepEnabled ? F("ON") : F("OFF"));
  Serial.print(F(" GPIO"));
  Serial.print(PIN_SERVO_ROOF);
  Serial.println(g_servoEnabled ? F("=PWM") : F("=DETACHED"));
}

const __FlashStringHelper *statusReason(const ActuatorCommandStatus status) {
  switch (status) {
    case ActuatorCommandStatus::Ok:
      return F("success");
    case ActuatorCommandStatus::CooldownActive:
      return F("cooldown_active");
    case ActuatorCommandStatus::InterlockViolation:
      return F("interlock_violation");
  }
  return F("internal_error");
}

void printCommandReject(const char *cmd, const ActuatorCommandStatus status) {
  Serial.print(F("REJECT "));
  Serial.print(cmd);
  Serial.print(F(" | "));
  Serial.println(statusReason(status));
}

void setServoSweep(const bool enabled) {
  g_servoSweepEnabled = false;
  if (enabled) {
    Serial.println(F("ROOF_SWEEP_BLOCKED bounded_0_90_only"));
  }
}

void startServoAngleCommand(const int angle, const uint32_t nowMs) {
  if (angle != AUTO_ROOF_OPEN_ANGLE && angle != AUTO_ROOF_SAFE_ANGLE) {
    Serial.print(F("ROOF_ANGLE_REJECT angle="));
    Serial.print(angle);
    Serial.println(F(" allowed=0,90"));
    return;
  }
  if (!g_servoEnabled) {
    setServoOnOff(true);
  }
  if (g_servoAngle == angle && !g_roofMotionActive && g_autoSafetyEnabled) {
    Serial.print(F("ROOF_WRITE_SKIP angle="));
    Serial.println(g_servoAngle);
    return;
  }
  g_servoAngle = angle;
  g_servoSweepEnabled = false;
  g_roofMotionActive = true;
  g_roofMotionStopMs = nowMs + ((angle == AUTO_ROOF_OPEN_ANGLE) ? ROOF_MOVE_OPEN_MS : ROOF_MOVE_CLOSE_MS);
  writeServoPulse((angle == AUTO_ROOF_OPEN_ANGLE) ? SERVO_OPEN_CMD : SERVO_CLOSE_CMD);
  Serial.print(F("ROOF_WRITE angle="));
  Serial.print(g_servoAngle);
  Serial.print(F(" pulse="));
  Serial.print(g_servoPulse);
  Serial.print(F(" dur_ms="));
  Serial.println((angle == AUTO_ROOF_OPEN_ANGLE) ? ROOF_MOVE_OPEN_MS : ROOF_MOVE_CLOSE_MS);
}

void setServoAngleCommand(const int angle) {
  startServoAngleCommand(angle, millis());
}

ActuatorCommandStatus requestLight(const bool on, const uint32_t nowMs) {
  if (!g_selfTestMode && on && cooldownActive(nowMs, g_lightCooldownUntilMs)) {
    return ActuatorCommandStatus::CooldownActive;
  }
  if (!g_selfTestMode && !on) {
    g_lightCooldownUntilMs = nowMs + LIGHT_COOLDOWN_MS;
  }
  stopLightBlink();
  setRelayLight(on);
  return ActuatorCommandStatus::Ok;
}

ActuatorCommandStatus requestLightBlink(const uint32_t nowMs) {
  if (!g_selfTestMode && cooldownActive(nowMs, g_lightCooldownUntilMs)) {
    return ActuatorCommandStatus::CooldownActive;
  }
  stopLightBlink();
  setRelayLight(true);
  g_lightBlinkActive = true;
  g_lightBlinkTransitionsRemaining = (LIGHT_BLINK_COUNT * 2) - 1;
  g_nextLightBlinkMs = millis() + LIGHT_BLINK_INTERVAL_MS;
  return ActuatorCommandStatus::Ok;
}

ActuatorCommandStatus requestFan(const bool on, const uint32_t nowMs) {
  if (!g_selfTestMode && on && cooldownActive(nowMs, g_fanCooldownUntilMs)) {
    return ActuatorCommandStatus::CooldownActive;
  }
  if (on) {
    g_fanOnSinceMs = nowMs;
  } else if (!g_selfTestMode && g_fanOn) {
    g_fanCooldownUntilMs = nowMs + FAN_COOLDOWN_MS;
    g_fanOnSinceMs = 0;
  } else if (!on) {
    g_fanOnSinceMs = 0;
  }
  g_fanOn = on;
  writeRelayPin(PIN_RELAY_FAN, on);
  return ActuatorCommandStatus::Ok;
}

ActuatorCommandStatus requestPump(const bool on, const uint32_t nowMs) {
  if (!g_selfTestMode && on && cooldownActive(nowMs, g_pumpCooldownUntilMs)) {
    return ActuatorCommandStatus::CooldownActive;
  }
  if (on) {
    g_pumpOnSinceMs = nowMs;
  } else if (!g_selfTestMode && g_pumpOn) {
    g_pumpCooldownUntilMs = nowMs + PUMP_COOLDOWN_MS;
    g_pumpOnSinceMs = 0;
  } else if (!on) {
    g_pumpOnSinceMs = 0;
  }
  g_pumpOn = on;
  writeRelayPin(PIN_RELAY_PUMP, on);
  return ActuatorCommandStatus::Ok;
}

ActuatorCommandStatus requestServoSweep(const bool enabled, const uint32_t nowMs) {
  (void)nowMs;
  if (enabled) {
    return ActuatorCommandStatus::InterlockViolation;
  }
  setServoSweep(enabled);
  return ActuatorCommandStatus::Ok;
}

ActuatorCommandStatus requestServoAngleCommand(const int angle, const uint32_t nowMs) {
  (void)nowMs;
  if (g_roofOpenInterlock && roofOpenAngle(angle)) {
    return ActuatorCommandStatus::InterlockViolation;
  }
  startServoAngleCommand(angle, nowMs);
  return ActuatorCommandStatus::Ok;
}

void setRoofOpenInterlock(const bool active) {
  g_roofOpenInterlock = active;
}

void setAutoSafetyEnabled(const bool enabled) {
  g_autoSafetyEnabled = enabled;
}

void initServoControl() {
#if APP_MODE_SIMULATOR
  g_servoEnabled = false;
  g_servoSweepEnabled = false;
  g_servoAngle = AUTO_ROOF_SAFE_ANGLE;
  g_servoPulse = SERVO_STOP;
  g_roofMotionActive = false;
  Serial.println(F("SERVO: OFF"));
#else
  g_servoEnabled = true;
  g_servoSweepEnabled = false;
  g_servoAngle = AUTO_ROOF_SAFE_ANGLE;
  g_servoPulse = SERVO_STOP;
  g_roofMotionActive = false;
  g_servoDirection = 1;
  attachServoIfNeeded();
  writeServoAngle();
  g_nextServoStepMs = millis() + SERVO_STEP_INTERVAL_MS;
  Serial.println(F("SERVO: ON"));
  Serial.println(F("SERVO: SWEEP OFF"));
  Serial.print(F("SERVO: ANGLE="));
  Serial.println(g_servoAngle);
#endif
}

void updateServoSweep(const uint32_t nowMs) {
  g_servoSweepEnabled = false;
  if (!g_roofMotionActive) {
    return;
  }
  if (static_cast<int32_t>(nowMs - g_roofMotionStopMs) < 0) {
    return;
  }
  g_roofMotionActive = false;
  writeServoPulse(SERVO_STOP);
  Serial.println(F("ROOF_STOP sent=1"));
}

void restoreSafeDefaults() {
  stopLightBlink();
  g_lightOn = false;
  g_fanOn = false;
  g_pumpOn = false;
  writeRelayPin(PIN_RELAY_LIGHT, false);
  writeRelayPin(PIN_RELAY_FAN, false);
  writeRelayPin(PIN_RELAY_PUMP, false);
  g_lightCooldownUntilMs = 0;
  g_fanOnSinceMs = 0;
  g_fanCooldownUntilMs = 0;
  g_pumpOnSinceMs = 0;
  g_pumpCooldownUntilMs = 0;
  g_roofOpenInterlock = false;
  g_servoEnabled = true;
  g_servoSweepEnabled = false;
  g_servoAngle = AUTO_ROOF_SAFE_ANGLE;
  g_servoPulse = SERVO_STOP;
  g_roofMotionActive = false;
  g_servoDirection = 1;
  attachServoIfNeeded();
  writeServoAngle();
}

void tickSafety(const uint32_t nowMs) {
  if (g_autoSafetyEnabled && g_pumpOn && g_pumpOnSinceMs > 0 && elapsedAtLeast(nowMs, g_pumpOnSinceMs, PUMP_MAX_ON_MS)) {
    g_pumpOn = false;
    g_pumpOnSinceMs = 0;
    g_pumpCooldownUntilMs = nowMs + PUMP_COOLDOWN_MS;
    writeRelayPin(PIN_RELAY_PUMP, false);
    Serial.println(F("SAFETY pump timeout | PUMP=OFF cooldown_active"));
  }

  if (g_fanOn && g_fanOnSinceMs > 0 && elapsedAtLeast(nowMs, g_fanOnSinceMs, FAN_MAX_ON_MS)) {
    g_fanOn = false;
    g_fanOnSinceMs = 0;
    g_fanCooldownUntilMs = nowMs + FAN_COOLDOWN_MS;
    writeRelayPin(PIN_RELAY_FAN, false);
    Serial.println(F("SAFETY fan timeout | FAN=OFF cooldown_active"));
  }
}

bool runSafetySelfTest() {
  bool allPass = true;

  restoreSafeDefaults();
  const uint32_t t0 = 100000;

  allPass &= (requestPump(true, t0) == ActuatorCommandStatus::Ok);
  tickSafety(t0 + PUMP_MAX_ON_MS + 1);
  allPass &= !g_pumpOn;
  allPass &= (requestPump(true, t0 + PUMP_MAX_ON_MS + 2) == ActuatorCommandStatus::CooldownActive);

  restoreSafeDefaults();
  allPass &= (requestFan(true, t0) == ActuatorCommandStatus::Ok);
  tickSafety(t0 + FAN_MAX_ON_MS + 1);
  allPass &= !g_fanOn;
  allPass &= (requestFan(true, t0 + FAN_MAX_ON_MS + 2) == ActuatorCommandStatus::CooldownActive);

  restoreSafeDefaults();
  allPass &= (requestLight(true, t0) == ActuatorCommandStatus::Ok);
  allPass &= (requestLight(false, t0 + 10) == ActuatorCommandStatus::Ok);
  allPass &= (requestLight(true, t0 + 20) == ActuatorCommandStatus::CooldownActive);

  restoreSafeDefaults();
  setRoofOpenInterlock(true);
  allPass &= (requestServoAngleCommand(AUTO_ROOF_OPEN_ANGLE, t0) == ActuatorCommandStatus::InterlockViolation);
  allPass &= (requestServoAngleCommand(AUTO_ROOF_SAFE_ANGLE, t0) == ActuatorCommandStatus::Ok);
  setRoofOpenInterlock(false);

  restoreSafeDefaults();
  Serial.print(F("ACTUATOR_SAFETY_SELF_TEST: "));
  Serial.println(allPass ? F("PASS") : F("FAIL"));
  return allPass;
}

bool lightOn() {
  return g_lightOn;
}

bool fanOn() {
  return g_fanOn;
}

bool pumpOn() {
  return g_pumpOn;
}

bool lightBlinkActive() {
  return g_lightBlinkActive;
}

bool servoEnabled() {
  return g_servoEnabled;
}

bool servoSweepEnabled() {
  return g_servoSweepEnabled;
}

bool roofMotionActive() {
  return g_roofMotionActive;
}

int servoAngle() {
  return g_servoAngle;
}

bool servoDetachedOrSelfTest() {
#if APP_MODE_SIMULATOR
  return true;
#else
  return g_selfTestMode || !g_roofServo.attached();
#endif
}

ActuatorSnapshot snapshot() {
  return {g_lightOn, g_fanOn, g_pumpOn, g_servoEnabled, g_servoSweepEnabled, g_servoAngle};
}

}  // namespace ActuatorManager
