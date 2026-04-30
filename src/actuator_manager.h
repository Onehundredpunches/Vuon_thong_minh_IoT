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
void tickSafety(uint32_t nowMs);

void setRelayLight(bool on);
void setRelayFan(bool on);
void setRelayPump(bool on);
void stopLightBlink();
void startLightBlink();

void setServoOnOff(bool on);
void setServoSweep(bool enabled);
void setServoAngleCommand(int angle);
bool roofMotionActive();
ActuatorCommandStatus requestLight(bool on, uint32_t nowMs);
ActuatorCommandStatus requestLightBlink(uint32_t nowMs);
ActuatorCommandStatus requestFan(bool on, uint32_t nowMs);
ActuatorCommandStatus requestPump(bool on, uint32_t nowMs);
ActuatorCommandStatus requestServoSweep(bool enabled, uint32_t nowMs);
ActuatorCommandStatus requestServoAngleCommand(int angle, uint32_t nowMs);
void setRoofOpenInterlock(bool active);

void printLightState();
void printFanState();
void printPumpState();
void printRelayCommandLog(const char *cmd, const char *name, uint8_t pin, bool on);
void printServoAck(const char *cmd);
void printCommandReject(const char *cmd, ActuatorCommandStatus status);
const __FlashStringHelper *statusReason(ActuatorCommandStatus status);

bool lightOn();
bool fanOn();
bool pumpOn();
bool lightBlinkActive();
bool servoEnabled();
bool servoSweepEnabled();
int servoAngle();
bool servoDetachedOrSelfTest();
bool runSafetySelfTest();

ActuatorSnapshot snapshot();

}  // namespace ActuatorManager

#endif
