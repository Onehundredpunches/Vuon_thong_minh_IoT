#include "actuator_manager.h"

#include <ESP32Servo.h>

#include "config.h"

namespace ActuatorManager {
namespace {

Servo g_roofServo;

const bool RELAY_ACTIVE_HIGH = true;

static constexpr int SERVO_MIN_ANGLE = 0;
static constexpr int SERVO_MAX_ANGLE = 180;
static constexpr int SERVO_STEP_DEG = 2;
static constexpr uint32_t SERVO_STEP_INTERVAL_MS = 20;
static constexpr uint8_t LIGHT_BLINK_COUNT = 5;
static constexpr uint32_t LIGHT_BLINK_INTERVAL_MS = 200;

bool g_selfTestMode = false;
bool g_servoEnabled = true;
bool g_servoSweepEnabled = false;
int g_servoAngle = SERVO_MIN_ANGLE;
int g_servoDirection = 1;
uint32_t g_nextServoStepMs = 0;
bool g_lightOn = false;
bool g_fanOn = false;
bool g_pumpOn = false;
bool g_lightBlinkActive = false;
uint8_t g_lightBlinkTransitionsRemaining = 0;
uint32_t g_nextLightBlinkMs = 0;

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
    g_roofServo.write(g_servoAngle);
  }
#endif
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
}

void setRelayFan(const bool on) {
  g_fanOn = on;
  writeRelayPin(PIN_RELAY_FAN, on);
}

void setRelayPump(const bool on) {
  g_pumpOn = on;
  writeRelayPin(PIN_RELAY_PUMP, on);
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
  Serial.print(F(" SWEEP="));
  Serial.print(g_servoSweepEnabled ? F("ON") : F("OFF"));
  Serial.print(F(" GPIO"));
  Serial.print(PIN_SERVO_ROOF);
  Serial.println(g_servoEnabled ? F("=PWM") : F("=DETACHED"));
}

void setServoSweep(const bool enabled) {
  g_servoSweepEnabled = enabled;
  if (!g_servoSweepEnabled) {
    return;
  }
  if (!g_servoEnabled) {
    setServoOnOff(true);
  }
  g_servoDirection = 1;
  g_nextServoStepMs = millis() + SERVO_STEP_INTERVAL_MS;
}

void setServoAngleCommand(const int angle) {
  if (!g_servoEnabled) {
    setServoOnOff(true);
  }
  g_servoAngle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  g_servoSweepEnabled = false;
  writeServoAngle();
}

void initServoControl() {
#if APP_MODE_SIMULATOR
  g_servoEnabled = false;
  g_servoSweepEnabled = false;
  g_servoAngle = SERVO_MIN_ANGLE;
  Serial.println(F("SERVO: OFF"));
#else
  g_servoEnabled = true;
  g_servoSweepEnabled = false;
  g_servoAngle = SERVO_MIN_ANGLE;
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
#if APP_MODE_SIMULATOR
  (void)nowMs;
#else
  if (!g_servoEnabled || !g_servoSweepEnabled || (!g_selfTestMode && !g_roofServo.attached())) {
    return;
  }
  if (static_cast<int32_t>(nowMs - g_nextServoStepMs) < 0) {
    return;
  }
  g_nextServoStepMs += SERVO_STEP_INTERVAL_MS;

  g_servoAngle += (SERVO_STEP_DEG * g_servoDirection);
  if (g_servoAngle >= SERVO_MAX_ANGLE) {
    g_servoAngle = SERVO_MAX_ANGLE;
    g_servoDirection = -1;
  } else if (g_servoAngle <= SERVO_MIN_ANGLE) {
    g_servoAngle = SERVO_MIN_ANGLE;
    g_servoDirection = 1;
  }
  if (!g_selfTestMode) {
    g_roofServo.write(g_servoAngle);
  }
#endif
}

void restoreSafeDefaults() {
  stopLightBlink();
  setRelayLight(false);
  setRelayFan(false);
  setRelayPump(false);
  g_servoEnabled = true;
  g_servoSweepEnabled = false;
  g_servoAngle = SERVO_MIN_ANGLE;
  g_servoDirection = 1;
  attachServoIfNeeded();
  writeServoAngle();
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
