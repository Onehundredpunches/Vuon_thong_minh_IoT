#ifndef ACTUATOR_TYPES_H
#define ACTUATOR_TYPES_H

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

#endif
