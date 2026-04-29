#include <Arduino.h>
#include <Wire.h>
#include <DHT.h>
#include <BH1750.h>
#include <ESP32Servo.h>
#include <math.h>
#include <ctype.h>

#include "config.h"
#include "sensor_types.h"
#include "actuator_types.h"
#include "system_mode.h"
#include "pins_validate.h"
#include "log_config.h"
#include "simulator.h"
#include "lcd_display.h"

namespace {

DHT g_dht(PIN_DHT22_DATA, DHT22);
BH1750 g_bh1750;

Servo g_roofServo;
const bool RELAY_ACTIVE_HIGH = true;

static constexpr int SERVO_MIN_ANGLE = 0;
static constexpr int SERVO_MAX_ANGLE = 180;
static constexpr int SERVO_STEP_DEG = 2;
static constexpr uint32_t SERVO_STEP_INTERVAL_MS = 20;
static constexpr uint8_t LIGHT_BLINK_COUNT = 5;
static constexpr uint32_t LIGHT_BLINK_INTERVAL_MS = 200;
static constexpr bool ENABLE_BOOT_SELF_TEST = true;

bool servoEnabled = true;
bool servoSweepEnabled = false;
int servoAngle = SERVO_MIN_ANGLE;
int g_servoDirection = 1;
uint32_t g_nextServoStepMs = 0;
bool g_lightOn = false;
bool g_fanOn = false;
bool g_pumpOn = false;
bool g_lightBlinkActive = false;
uint8_t g_lightBlinkTransitionsRemaining = 0;
uint32_t g_nextLightBlinkMs = 0;

char g_serialCmdBuf[48] = {0};
size_t g_serialCmdLen = 0;

uint32_t g_nextSampleMs = 0;
uint32_t g_sampleIndex = 0;
bool g_selfTestMode = false;
bool g_relaySelfTestPass = false;
bool g_lightSelfTestPass = false;
bool g_fanSelfTestPass = false;
bool g_pumpSelfTestPass = false;
bool g_servoSelfTestPass = false;
bool g_pinMapSelfTestPass = false;

float adcToVoltage(const uint16_t raw) {
  return (static_cast<float>(raw) * ADC_REF_VOLTAGE) / static_cast<float>(ADC_MAX);
}

bool probeI2cAddress(const uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void printPinMap() {
  Serial.println(F("PINMAP I2C_SDA=21 I2C_SCL=22 DHT=27 SoilAO=34 SoilDO=26 RainAO=35 RainDO=25 SERVO=19 LIGHT=18 FAN=17 PUMP=16"));
}

void printRelayActiveLevel() {
  Serial.print(F("RELAY_ACTIVE_LEVEL: "));
  Serial.println(RELAY_ACTIVE_HIGH ? F("ACTIVE_HIGH") : F("ACTIVE_LOW"));
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
  if (!servoEnabled) {
    return;
  }
  if (attachServoIfNeeded()) {
    g_roofServo.write(servoAngle);
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

void printRelayState(const char *name, const bool on) {
  Serial.print(name);
  Serial.print(F(": "));
  Serial.println(on ? F("ON") : F("OFF"));
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

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  help"));
  Serial.println(F("  servo on"));
  Serial.println(F("  servo off"));
  Serial.println(F("  servo sweep on"));
  Serial.println(F("  servo sweep off"));
  Serial.println(F("  servo stop"));
  Serial.println(F("  servo 0"));
  Serial.println(F("  servo 90"));
  Serial.println(F("  servo 180"));
  Serial.println(F("  light on"));
  Serial.println(F("  light off"));
  Serial.println(F("  light toggle"));
  Serial.println(F("  light blink"));
  Serial.println(F("  fan on"));
  Serial.println(F("  fan off"));
  Serial.println(F("  pump on"));
  Serial.println(F("  pump off"));
}

void setServoOnOff(const bool on) {
#if APP_MODE_SIMULATOR
  servoEnabled = on;
  if (!servoEnabled) {
    servoSweepEnabled = false;
  }
#else
  servoEnabled = on;
  if (servoEnabled) {
    attachServoIfNeeded();
    writeServoAngle();
  } else {
    servoSweepEnabled = false;
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
  Serial.print(servoEnabled ? F("ON") : F("OFF"));
  Serial.print(F(" ANGLE="));
  Serial.print(servoAngle);
  Serial.print(F(" SWEEP="));
  Serial.print(servoSweepEnabled ? F("ON") : F("OFF"));
  Serial.print(F(" GPIO"));
  Serial.print(PIN_SERVO_ROOF);
  Serial.println(servoEnabled ? F("=PWM") : F("=DETACHED"));
}

void setServoSweep(const bool enabled) {
  servoSweepEnabled = enabled;
  if (!servoSweepEnabled) {
    return;
  }
  if (!servoEnabled) {
    setServoOnOff(true);
  }
  g_servoDirection = 1;
  g_nextServoStepMs = millis() + SERVO_STEP_INTERVAL_MS;
}

void setServoAngleCommand(const int angle) {
  if (!servoEnabled) {
    setServoOnOff(true);
  }
  servoAngle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  servoSweepEnabled = false;
  writeServoAngle();
}

void processSerialCommand(char *cmd) {
  // Normalize whitespace and lower-case input so commands are robust to spacing/line ending variations.
  size_t writeIdx = 0;
  bool prevSpace = true;
  for (size_t i = 0; cmd[i] != '\0'; ++i) {
    char c = cmd[i];
    if (c == '\t') {
      c = ' ';
    }
    c = static_cast<char>(tolower(static_cast<unsigned char>(c)));

    if (c == ' ') {
      if (!prevSpace) {
        cmd[writeIdx++] = ' ';
      }
      prevSpace = true;
      continue;
    }

    cmd[writeIdx++] = c;
    prevSpace = false;
  }
  if (writeIdx > 0 && cmd[writeIdx - 1] == ' ') {
    --writeIdx;
  }
  cmd[writeIdx] = '\0';

  if (writeIdx == 0) {
    return;
  }

  if (strcmp(cmd, "help") == 0) {
    printHelp();
    return;
  }
  if (strcmp(cmd, "servo off") == 0) {
    setServoOnOff(false);
    printServoAck(cmd);
    return;
  }
  if (strcmp(cmd, "servo on") == 0) {
    setServoOnOff(true);
    printServoAck(cmd);
    return;
  }
  if (strcmp(cmd, "servo sweep on") == 0) {
    setServoSweep(true);
    printServoAck(cmd);
    return;
  }
  if (strcmp(cmd, "servo sweep off") == 0 || strcmp(cmd, "servo stop") == 0) {
    setServoSweep(false);
    printServoAck(cmd);
    return;
  }

  int targetAngle = -1;
  if (sscanf(cmd, "servo %d", &targetAngle) == 1 &&
      (targetAngle == 0 || targetAngle == 90 || targetAngle == 180)) {
    setServoAngleCommand(targetAngle);
    printServoAck(cmd);
    return;
  }

  if (strcmp(cmd, "light on") == 0) {
    printRelayCommandLog(cmd, "LIGHT", PIN_RELAY_LIGHT, true);
    stopLightBlink();
    setRelayLight(true);
    printLightState();
    return;
  }
  if (strcmp(cmd, "light off") == 0) {
    printRelayCommandLog(cmd, "LIGHT", PIN_RELAY_LIGHT, false);
    stopLightBlink();
    setRelayLight(false);
    printLightState();
    return;
  }
  if (strcmp(cmd, "light toggle") == 0) {
    printRelayCommandLog(cmd, "LIGHT", PIN_RELAY_LIGHT, !g_lightOn);
    stopLightBlink();
    setRelayLight(!g_lightOn);
    printLightState();
    return;
  }
  if (strcmp(cmd, "light blink") == 0) {
    printRelayCommandLog(cmd, "LIGHT", PIN_RELAY_LIGHT, true);
    startLightBlink();
    return;
  }
  if (strcmp(cmd, "fan on") == 0) {
    printRelayCommandLog(cmd, "FAN", PIN_RELAY_FAN, true);
    setRelayFan(true);
    printFanState();
    return;
  }
  if (strcmp(cmd, "fan off") == 0) {
    printRelayCommandLog(cmd, "FAN", PIN_RELAY_FAN, false);
    setRelayFan(false);
    printFanState();
    return;
  }
  if (strcmp(cmd, "pump on") == 0) {
    printRelayCommandLog(cmd, "PUMP", PIN_RELAY_PUMP, true);
    setRelayPump(true);
    printPumpState();
    return;
  }
  if (strcmp(cmd, "pump off") == 0) {
    printRelayCommandLog(cmd, "PUMP", PIN_RELAY_PUMP, false);
    setRelayPump(false);
    printPumpState();
    return;
  }

  Serial.println(F("CMD: UNKNOWN"));
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      g_serialCmdBuf[g_serialCmdLen] = '\0';
      if (g_serialCmdLen > 0) {
        processSerialCommand(g_serialCmdBuf);
      }
      g_serialCmdLen = 0;
      g_serialCmdBuf[0] = '\0';
      continue;
    }

    if (g_serialCmdLen < (sizeof(g_serialCmdBuf) - 1)) {
      g_serialCmdBuf[g_serialCmdLen++] = c;
      g_serialCmdBuf[g_serialCmdLen] = '\0';
    }
  }
}

void initServoControl() {
#if APP_MODE_SIMULATOR
  servoEnabled = false;
  servoSweepEnabled = false;
  servoAngle = SERVO_MIN_ANGLE;
  Serial.println(F("SERVO: OFF"));
#else
  servoEnabled = true;
  servoSweepEnabled = false;
  servoAngle = SERVO_MIN_ANGLE;
  g_servoDirection = 1;
  attachServoIfNeeded();
  writeServoAngle();
  g_nextServoStepMs = millis() + SERVO_STEP_INTERVAL_MS;
  Serial.println(F("SERVO: ON"));
  Serial.println(F("SERVO: SWEEP OFF"));
  Serial.print(F("SERVO: ANGLE="));
  Serial.println(servoAngle);
#endif
}

void updateServoSweep(const uint32_t nowMs) {
#if APP_MODE_SIMULATOR
  (void)nowMs;
#else
  if (!servoEnabled || !servoSweepEnabled || (!g_selfTestMode && !g_roofServo.attached())) {
    return;
  }
  if (static_cast<int32_t>(nowMs - g_nextServoStepMs) < 0) {
    return;
  }
  g_nextServoStepMs += SERVO_STEP_INTERVAL_MS;

  servoAngle += (SERVO_STEP_DEG * g_servoDirection);
  if (servoAngle >= SERVO_MAX_ANGLE) {
    servoAngle = SERVO_MAX_ANGLE;
    g_servoDirection = -1;
  } else if (servoAngle <= SERVO_MIN_ANGLE) {
    servoAngle = SERVO_MIN_ANGLE;
    g_servoDirection = 1;
  }
  if (!g_selfTestMode) {
    g_roofServo.write(servoAngle);
  }
#endif
}

void executeCommandForTest(const char *cmd) {
  char local[48] = {0};
  strncpy(local, cmd, sizeof(local) - 1);
  processSerialCommand(local);
}

bool reportTestStep(const char *group, const char *step, const bool pass) {
  Serial.print(group);
  Serial.print(F(" "));
  Serial.print(step);
  Serial.print(F(": "));
  Serial.println(pass ? F("PASS") : F("FAIL"));
  return pass;
}

void runRelaySelfTest() {
  bool allPass = true;
  stopLightBlink();
  setRelayLight(false);
  g_lightSelfTestPass = reportTestStep("LIGHT_TEST", "boot_off", !g_lightOn);
  allPass &= g_lightSelfTestPass;

  executeCommandForTest("light on");
  g_lightSelfTestPass &= reportTestStep("LIGHT_TEST", "light_on", g_lightOn);
  allPass &= g_lightSelfTestPass;

  executeCommandForTest("light off");
  g_lightSelfTestPass &= reportTestStep("LIGHT_TEST", "light_off", !g_lightOn);
  allPass &= g_lightSelfTestPass;

  executeCommandForTest("light toggle");
  g_lightSelfTestPass &= reportTestStep("LIGHT_TEST", "toggle_on", g_lightOn);
  allPass &= g_lightSelfTestPass;

  executeCommandForTest("light toggle");
  g_lightSelfTestPass &= reportTestStep("LIGHT_TEST", "toggle_off", !g_lightOn);
  allPass &= g_lightSelfTestPass;

  executeCommandForTest("light blink");
  uint32_t simNow = g_nextLightBlinkMs;
  uint8_t guard = 32;
  while (g_lightBlinkActive && guard > 0) {
    updateLightBlink(simNow);
    simNow += LIGHT_BLINK_INTERVAL_MS;
    --guard;
  }
  g_lightSelfTestPass &= reportTestStep("LIGHT_TEST", "blink_complete_off", !g_lightBlinkActive && !g_lightOn);
  allPass &= g_lightSelfTestPass;

  setRelayFan(false);
  g_fanSelfTestPass = reportTestStep("FAN_TEST", "boot_off", !g_fanOn);
  executeCommandForTest("fan on");
  g_fanSelfTestPass &= reportTestStep("FAN_TEST", "fan_on", g_fanOn);
  executeCommandForTest("fan off");
  g_fanSelfTestPass &= reportTestStep("FAN_TEST", "fan_off", !g_fanOn);
  allPass &= g_fanSelfTestPass;

  setRelayPump(false);
  g_pumpSelfTestPass = reportTestStep("PUMP_TEST", "boot_off", !g_pumpOn);
  executeCommandForTest("pump on");
  g_pumpSelfTestPass &= reportTestStep("PUMP_TEST", "pump_on", g_pumpOn);
  executeCommandForTest("pump off");
  g_pumpSelfTestPass &= reportTestStep("PUMP_TEST", "pump_off", !g_pumpOn);
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

void runServoSelfTest() {
  bool allPass = true;

  executeCommandForTest("servo on");
  allPass &= reportTestStep("SERVO_TEST", "servo_on", servoEnabled);

  executeCommandForTest("servo sweep on");
  allPass &= reportTestStep("SERVO_TEST", "sweep_on", servoEnabled && servoSweepEnabled);

  executeCommandForTest("servo sweep off");
  allPass &= reportTestStep("SERVO_TEST", "sweep_off", !servoSweepEnabled);

  executeCommandForTest("servo 180");
  allPass &= reportTestStep("SERVO_TEST", "angle_180", servoAngle == 180 && !servoSweepEnabled);

  executeCommandForTest("servo off");
  bool detachedOrSelfTest = g_selfTestMode || !g_roofServo.attached();
  allPass &= reportTestStep("SERVO_TEST", "servo_off", !servoEnabled && detachedOrSelfTest);

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

void runSelfTests() {
  if (!ENABLE_BOOT_SELF_TEST) {
    return;
  }
  Serial.println(F("SELF_TEST: START"));
  g_selfTestMode = true;
  runPinMapSelfTest();
  runRelaySelfTest();
  runServoSelfTest();
  g_selfTestMode = false;

  // Restore safe default runtime states after tests.
  stopLightBlink();
  setRelayLight(false);
  setRelayFan(false);
  setRelayPump(false);
  servoEnabled = true;
  servoSweepEnabled = false;
  servoAngle = SERVO_MIN_ANGLE;
  g_servoDirection = 1;
  attachServoIfNeeded();
  writeServoAngle();

  const bool allPass = g_pinMapSelfTestPass && g_relaySelfTestPass && g_servoSelfTestPass;
  Serial.print(F("SELF_TEST: "));
  Serial.println(allPass ? F("PASS") : F("FAIL"));
}

void readHardware(SensorData &d) {
  const float humidity = g_dht.readHumidity();
  const float temperatureC = g_dht.readTemperature();
  d.dhtOk = !(isnan(humidity) || isnan(temperatureC));
  if (d.dhtOk) {
    d.humidityPct = humidity;
    d.temperatureC = temperatureC;
  }

  const float lux = g_bh1750.readLightLevel();
  d.bh1750Ok = (lux >= 0.0f && !isnan(lux));
  if (d.bh1750Ok) {
    d.lux = lux;
  }

  d.soilAO = analogRead(PIN_SOIL_AO);
  d.rainAO = analogRead(PIN_RAIN_AO);
  d.soilDO = static_cast<uint8_t>(digitalRead(PIN_SOIL_DO));
  d.rainDO = static_cast<uint8_t>(digitalRead(PIN_RAIN_DO));
}

void printCompactBlock(const SensorData &d) {
  const float soilV = adcToVoltage(d.soilAO);
  const float rainV = adcToVoltage(d.rainAO);

  Serial.print(F("DATA Temp:"));
  if (d.dhtOk) {
    Serial.print(d.temperatureC, 1);
    Serial.print(F("C"));
  } else {
    Serial.print(F("DHT_ERR"));
  }

  Serial.print(F(" Hum:"));
  if (d.dhtOk) {
    Serial.print(d.humidityPct, 1);
    Serial.print(F("%RH"));
  } else {
    Serial.print(F("DHT_ERR"));
  }

  Serial.print(F(" Lux:"));
  if (d.bh1750Ok) {
    Serial.print(d.lux, 1);
    Serial.print(F("lx"));
  } else {
    Serial.print(F("BH1750_ERR"));
  }

  Serial.print(F(" SoilAO:"));
  Serial.print(d.soilAO);
  Serial.print(F("("));
  Serial.print(soilV, 3);
  Serial.print(F("V)"));

  Serial.print(F(" RainAO:"));
  Serial.print(d.rainAO);
  Serial.print(F("("));
  Serial.print(rainV, 3);
  Serial.print(F("V)"));

  Serial.print(F(" SoilDO:"));
  Serial.print(d.soilDO);
  Serial.print(F(" RainDO:"));
  Serial.println(d.rainDO);
}

void printStartup() {
#if APP_MODE_SIMULATOR
  Serial.println(F("MODE: SIMULATOR"));
#else
  Serial.println(F("MODE: HARDWARE"));
#endif
  printPinMap();
  printRelayActiveLevel();
}

}  // namespace

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);

  pinMode(PIN_DHT22_DATA, INPUT_PULLUP);
  pinMode(PIN_SOIL_AO, INPUT);
  pinMode(PIN_RAIN_AO, INPUT);
  pinMode(PIN_SOIL_DO, INPUT);
  pinMode(PIN_RAIN_DO, INPUT);
  pinMode(PIN_RELAY_LIGHT, OUTPUT);
  pinMode(PIN_RELAY_FAN, OUTPUT);
  pinMode(PIN_RELAY_PUMP, OUTPUT);

  setRelayLight(false);
  setRelayFan(false);
  setRelayPump(false);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  printStartup();
  printLightState();
  printFanState();
  printPumpState();
  scanI2cBus();

#if APP_MODE_SIMULATOR
  // Simulator mode intentionally avoids real sensor init/read calls.
#else
  g_dht.begin();
  const bool bh1750Ok = g_bh1750.begin();
  if (!bh1750Ok) {
    Serial.println(F("BH1750_ERR"));
  }
#endif

  initLcdIfPresent();
  initServoControl();
  runSelfTests();
  printHelp();

  g_nextSampleMs = millis();
}

void loop() {
  const uint32_t now = millis();
  handleSerialCommands();
  updateServoSweep(now);
  updateLightBlink(now);

  if (static_cast<int32_t>(now - g_nextSampleMs) < 0) {
    return;
  }
  g_nextSampleMs += LOOP_INTERVAL_MS;

  SensorData data{};
#if APP_MODE_SIMULATOR
  readSimulator(data, g_sampleIndex);
#else
  readHardware(data);
#endif

  printCompactBlock(data);
  updateLcd(data);

  ++g_sampleIndex;
}
