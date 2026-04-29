#ifndef MQTT_TOPICS_H
#define MQTT_TOPICS_H

#include <stdint.h>

namespace MqttTopics {

static constexpr const char *kTelemetry = "vuon-iot/zone1/gateway1/telemetry";
static constexpr const char *kCommand = "vuon-iot/zone1/gateway1/command";
static constexpr const char *kCommandAck = "vuon-iot/zone1/gateway1/command/ack";
static constexpr const char *kStateSensor = "vuon-iot/zone1/gateway1/state/sensor";
static constexpr const char *kStateActuator = "vuon-iot/zone1/gateway1/state/actuator";
static constexpr const char *kStatus = "vuon-iot/zone1/gateway1/status";

static constexpr uint8_t kTelemetryQos = 0;
static constexpr uint8_t kCommandQos = 1;
static constexpr uint8_t kCommandAckQos = 1;
static constexpr uint8_t kStateSensorQos = 1;
static constexpr uint8_t kStateActuatorQos = 1;
static constexpr uint8_t kStatusQos = 1;

static constexpr bool kTelemetryRetain = false;
static constexpr bool kCommandAckRetain = false;
static constexpr bool kStateSensorRetain = true;
static constexpr bool kStateActuatorRetain = true;
static constexpr bool kStatusRetain = true;

}  // namespace MqttTopics

#endif
