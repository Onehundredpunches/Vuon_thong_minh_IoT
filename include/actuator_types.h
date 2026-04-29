#ifndef ACTUATOR_TYPES_H
#define ACTUATOR_TYPES_H

#include <Arduino.h>

enum class ActuatorTarget : uint8_t {
  Light,
  Fan,
  Pump,
  Roof
};

enum class ActuatorState : uint8_t {
  Off,
  On
};

enum class ActuatorCommandStatus : uint8_t {
  Ok,
  CooldownActive,
  InterlockViolation
};

struct ActuatorSnapshot {
  bool lightOn;
  bool fanOn;
  bool pumpOn;
  bool servoEnabled;
  bool servoSweepEnabled;
  int servoAngle;
};

#endif
