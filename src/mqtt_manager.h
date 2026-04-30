#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <Arduino.h>

namespace MqttManager {

void begin(uint32_t nowMs);
void tick(uint32_t nowMs);
bool wifiConnected();
bool mqttConnected();
bool runReconnectSelfTest();

}  // namespace MqttManager

#endif
