#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <Arduino.h>

#include "actuator_types.h"
#include "sensor_types.h"

namespace MqttManager {

void begin(uint32_t nowMs);
void tick(uint32_t nowMs);
void updateState(uint32_t nowMs, const SensorData &sensors, const ActuatorSnapshot &actuators);
bool wifiConnected();
bool mqttConnected();
bool runReconnectSelfTest();
bool runCommandAckSelfTest();
bool runRetainedStateSelfTest();

}  // namespace MqttManager

#endif
