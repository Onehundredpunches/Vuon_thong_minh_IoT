#ifndef ACTUATOR_MANAGER_H
#define ACTUATOR_MANAGER_H

#include <Arduino.h>

#include "actuator_types.h"

namespace ActuatorManager {

void begin();
void setSelfTestMode(bool enabled);
void printRelayActiveLevel();

void initServoControl();
void updateServoSweep(uint32_t nowMs);
void updateLightBlink(uint32_t nowMs);
void completeLightBlinkForTest();
void restoreSafeDefaults();

void setRelayLight(bool on);
void setRelayFan(bool on);
void setRelayPump(bool on);
void stopLightBlink();
void startLightBlink();

void setServoOnOff(bool on);
void setServoSweep(bool enabled);
void setServoAngleCommand(int angle);

void printLightState();
void printFanState();
void printPumpState();
void printRelayCommandLog(const char *cmd, const char *name, uint8_t pin, bool on);
void printServoAck(const char *cmd);

bool lightOn();
bool fanOn();
bool pumpOn();
bool lightBlinkActive();
bool servoEnabled();
bool servoSweepEnabled();
int servoAngle();
bool servoDetachedOrSelfTest();

ActuatorSnapshot snapshot();

}  // namespace ActuatorManager

#endif
