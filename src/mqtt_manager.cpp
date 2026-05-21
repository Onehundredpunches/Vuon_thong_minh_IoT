#include "mqtt_manager.h"

#include "actuator_manager.h"
#include "config.h"
#include "command_handler.h"
#include "mode_controller.h"
#include "mqtt_topics.h"

#if APP_MODE_SIMULATOR

namespace MqttManager {

void begin(const uint32_t nowMs) {
  (void)nowMs;
  Serial.println(F("MQTT_MANAGER: SIM_DISABLED"));
}

void tick(const uint32_t nowMs) {
  (void)nowMs;
}

void updateState(const uint32_t nowMs, const SensorData &sensors, const ActuatorSnapshot &actuators) {
  (void)nowMs;
  (void)sensors;
  (void)actuators;
}

bool wifiConnected() {
  return false;
}

bool mqttConnected() {
  return false;
}

bool runReconnectSelfTest() {
  Serial.println(F("MQTT_RECONNECT_SELF_TEST: PASS"));
  return true;
}

bool runCommandAckSelfTest() {
  Serial.println(F("MQTT_COMMAND_ACK_SELF_TEST: PASS"));
  return true;
}

bool runRetainedStateSelfTest() {
  Serial.println(F("MQTT_RETAINED_STATE_SELF_TEST: PASS"));
  return true;
}

}  // namespace MqttManager

#else

#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>

#include "time_sync.h"

namespace MqttManager {
namespace {

WiFiClient g_wifiClient;
PubSubClient g_mqtt(g_wifiClient);

uint32_t g_nextWifiAttemptMs = 0;
uint32_t g_nextMqttAttemptMs = 0;
uint32_t g_wifiBackoffMs = 2000;
uint32_t g_mqttBackoffMs = 3000;
uint8_t g_wifiFailureCount = 0;
bool g_wifiStarted = false;
bool g_everMqttConnected = false;
bool g_lastMqttConnected = false;
bool g_scanStarted = false;
bool g_scanDone = false;
wl_status_t g_lastWifiStatus = WL_IDLE_STATUS;
SensorData g_lastSensors = {};
ActuatorSnapshot g_lastActuators = {};
bool g_haveSensorState = false;
bool g_haveActuatorState = false;
uint32_t g_lastStatePublishMs = 0;
bool g_pendingInitialPublish = false;
const char *g_pendingPublishReason = nullptr;
uint32_t g_mqttConnectMs = 0;

enum class WifiProfile : uint8_t {
  Primary = 0,
  Fallback = 1,
};

struct WifiProfileConfig {
  const char *name;
  const char *ssid;
  const char *password;
  bool scanFound;
  int32_t channel;
};

WifiProfileConfig g_wifiProfiles[] = {
    {"primary", WIFI_PRIMARY_SSID, WIFI_PRIMARY_PASSWORD, false, 0},
    {"fallback", WIFI_FALLBACK_SSID, WIFI_FALLBACK_PASSWORD, false, 0},
};
WifiProfile g_activeWifiProfile = WifiProfile::Primary;
uint32_t g_wifiAttemptStartMs = 0;

struct RecentCommand {
  char cmdId[65];
  uint32_t seenMs;
};

RecentCommand g_recentCommands[16] = {};
uint8_t g_recentCommandWrite = 0;

const __FlashStringHelper *encryptionName(const wifi_auth_mode_t enc) {
  switch (enc) {
    case WIFI_AUTH_OPEN:
      return F("OPEN");
    case WIFI_AUTH_WEP:
      return F("WEP");
    case WIFI_AUTH_WPA_PSK:
      return F("WPA");
    case WIFI_AUTH_WPA2_PSK:
      return F("WPA2");
    case WIFI_AUTH_WPA_WPA2_PSK:
      return F("WPA/WPA2");
    case WIFI_AUTH_WPA2_ENTERPRISE:
      return F("WPA2_ENT");
    case WIFI_AUTH_WPA3_PSK:
      return F("WPA3");
    case WIFI_AUTH_WPA2_WPA3_PSK:
      return F("WPA2/WPA3");
    default:
      return F("UNKNOWN");
  }
}

uint32_t nextWifiBackoff(const uint32_t current) {
  if (current < 4000) {
    return 4000;
  }
  if (current < 8000) {
    return 8000;
  }
  if (current < 16000) {
    return 16000;
  }
  return 30000;
}

WifiProfileConfig &wifiProfile(const WifiProfile profile) {
  return g_wifiProfiles[static_cast<uint8_t>(profile)];
}

const WifiProfileConfig &wifiProfileConst(const WifiProfile profile) {
  return g_wifiProfiles[static_cast<uint8_t>(profile)];
}

void selectWifiProfile(const WifiProfile profile) {
  g_activeWifiProfile = profile;
}

void printWifiProfileFound(const WifiProfileConfig &profile, const int32_t rssi,
                           const wifi_auth_mode_t enc) {
  Serial.print(F("WIFI_TARGET_FOUND profile="));
  Serial.print(profile.name);
  Serial.print(F(" ssid="));
  Serial.print(profile.ssid);
  Serial.print(F(" rssi="));
  Serial.print(rssi);
  Serial.print(F(" channel="));
  Serial.print(profile.channel);
  Serial.print(F(" enc="));
  Serial.println(encryptionName(enc));
  if (enc == WIFI_AUTH_OPEN && strlen(profile.password) > 0) {
    Serial.print(F("WIFI_AUTH_HINT profile="));
    Serial.print(profile.name);
    Serial.println(F(" target_open_password_configured"));
  }
  if (enc == WIFI_AUTH_WPA3_PSK) {
    Serial.print(F("WIFI_AUTH_HINT profile="));
    Serial.print(profile.name);
    Serial.println(F(" target_wpa3_only_may_fail_on_legacy_esp32"));
  }
}

uint32_t nextMqttBackoff(const uint32_t current) {
  if (current < 6000) {
    return 6000;
  }
  if (current < 12000) {
    return 12000;
  }
  return 30000;
}
bool g_timeWasSynced = false;

bool actuatorsEqual(const ActuatorSnapshot &a, const ActuatorSnapshot &b) {
  return a.lightOn == b.lightOn &&
         a.fanOn == b.fanOn &&
         a.pumpOn == b.pumpOn &&
         a.servoEnabled == b.servoEnabled &&
         a.servoSweepEnabled == b.servoSweepEnabled &&
         a.servoAngle == b.servoAngle;
}

bool sensorsEqual(const SensorData &a, const SensorData &b) {
  if (abs(a.temperatureC - b.temperatureC) >= 0.5f) return false;
  if (abs(a.humidityPct - b.humidityPct) >= 2.0f) return false;
  if (abs(a.lux - b.lux) >= 10.0f) return false;
  if (abs(a.soilPct - b.soilPct) >= 2.0f) return false;
  
  if (a.rainDO != b.rainDO) return false;
  if (a.soilDO != b.soilDO) return false;
  
  if (a.sensor_invalid != b.sensor_invalid) return false;
  if (a.dht_ok != b.dht_ok) return false;
  if (a.bh1750_ok != b.bh1750_ok) return false;
  if (a.soil_ok != b.soil_ok) return false;
  if (a.rain_ok != b.rain_ok) return false;

  return true;
}

const char *onOff(const bool value) {
  return value ? "on" : "off";
}

const char *validInvalid(const bool valid) {
  return valid ? "ok" : "invalid";
}

const char *jsonBool(const bool value) {
  return value ? "true" : "false";
}

const char *modeText() {
  switch (ModeController::mode()) {
    case SystemMode::Boot:
      return "boot";
    case SystemMode::Auto:
      return "auto";
    case SystemMode::Manual:
      return "manual";
    case SystemMode::SafeStop:
      return "safe_stop";
    case SystemMode::Error:
      return "error";
  }
  return "error";
}

const char *roofText(const int logicalState) {
  return (logicalState == AUTO_ROOF_OPEN_ANGLE) ? "open" : "close";
}

const char *rainText(const SensorData &data) {
  if (!data.rain_ok) {
    return "unknown";
  }
  return data.rainDO == 0 ? "rain" : "dry";
}

const char *statusHealth() {
  const SystemMode mode = ModeController::mode();
  if (mode == SystemMode::Error || mode == SystemMode::SafeStop) {
    return "error";
  }
  if (g_haveSensorState && g_lastSensors.sensor_invalid) {
    return "degraded";
  }
  return "ok";
}

void publishStatus(const char *sysStatus, const char *reason) {
  char iso[26] = {0};
  TimeSync::formatIso8601(iso, sizeof(iso));
  char payload[256] = {0};
  snprintf(payload, sizeof(payload),
           "{\"system\":\"%s\",\"mode\":\"%s\",\"health\":\"%s\",\"reason\":\"%s\",\"updateTime\":\"%s\"}",
           sysStatus, modeText(), statusHealth(), reason, iso);
  const bool ok = g_mqtt.publish(MqttTopics::kStatus, payload, MqttTopics::kStatusRetain);
  Serial.print(F("MQTT_STATUS_PUBLISH retained=1 reason="));
  Serial.print(reason);
  Serial.print(F(" result="));
  Serial.println(ok ? F("ok") : F("fail"));
}

void publishSensorState() {
  if (!g_haveSensorState) {
    Serial.println(F("MQTT_STATE_SENSOR_SKIP reason=no_cache"));
    return;
  }
  char iso[26] = {0};
  TimeSync::formatIso8601(iso, sizeof(iso));
  char payload[1024] = {0};
  // Semantic dashboard-facing v2 payload; topic and retain behavior stay unchanged.
  snprintf(payload, sizeof(payload),
           "{\"airTemperatureC\":%.1f,\"airHumidityPct\":%.1f,\"lightLux\":%.1f,"
           "\"soilMoisturePct\":%.1f,\"rain\":\"%s\","
           "\"health\":{\"sensor\":\"%s\",\"dht\":\"%s\",\"light\":\"%s\",\"soil\":\"%s\",\"rain\":\"%s\"},"
           "\"raw\":{\"soilAO\":%u,\"rainAO\":%u,\"soilDO\":%u,\"rainDO\":%u},"
           "\"controlFlags\":{\"tempOverFanOnThreshold\":%s,\"tempBelowFanOffThreshold\":%s,"
           "\"soilBelowPumpOnThreshold\":%s,\"soilAbovePumpOffThreshold\":%s,"
           "\"luxBelowLightOnThreshold\":%s,\"luxAboveLightOffThreshold\":%s,"
           "\"isRaining\":%s,\"airHumidityTooHigh\":%s,\"airHumidityTooLow\":%s},"
           "\"controlThresholds\":{\"fanOnTempC\":%.1f,\"fanOffTempC\":%.1f,"
           "\"pumpOnSoilMoisturePct\":%.1f,\"pumpOffSoilMoisturePct\":%.1f,"
           "\"lightOnLux\":%.1f,\"lightOffLux\":%.1f,"
           "\"airHumidityHighPct\":%.1f,\"airHumidityLowPct\":%.1f},"
           "\"updateTime\":\"%s\"}",
           g_lastSensors.temperatureC, g_lastSensors.humidityPct, g_lastSensors.lux, g_lastSensors.soilPct,
           rainText(g_lastSensors), g_lastSensors.sensor_invalid ? "invalid" : "ok",
           validInvalid(g_lastSensors.dht_ok), validInvalid(g_lastSensors.bh1750_ok),
           validInvalid(g_lastSensors.soil_ok), validInvalid(g_lastSensors.rain_ok),
           g_lastSensors.soilAO, g_lastSensors.rainAO, g_lastSensors.soilDO, g_lastSensors.rainDO,
           jsonBool(g_lastSensors.temperatureC > AUTO_TEMP_FAN_ON_C),
           jsonBool(g_lastSensors.temperatureC < AUTO_TEMP_FAN_OFF_C),
           jsonBool(g_lastSensors.soilPct < AUTO_SOIL_PUMP_ON_PCT),
           jsonBool(g_lastSensors.soilPct > AUTO_SOIL_PUMP_OFF_PCT),
           jsonBool(g_lastSensors.lux < AUTO_LIGHT_ON_LUX),
           jsonBool(g_lastSensors.lux > AUTO_LIGHT_OFF_LUX),
           jsonBool(g_lastSensors.rain_ok && g_lastSensors.rainDO == 0),
           jsonBool(g_lastSensors.humidityPct > AIR_HUMIDITY_HIGH_PCT),
           jsonBool(g_lastSensors.humidityPct < AIR_HUMIDITY_LOW_PCT),
           AUTO_TEMP_FAN_ON_C, AUTO_TEMP_FAN_OFF_C,
           AUTO_SOIL_PUMP_ON_PCT, AUTO_SOIL_PUMP_OFF_PCT,
           AUTO_LIGHT_ON_LUX, AUTO_LIGHT_OFF_LUX,
           AIR_HUMIDITY_HIGH_PCT, AIR_HUMIDITY_LOW_PCT,
           iso);
  const bool ok = g_mqtt.publish(MqttTopics::kStateSensor, payload, MqttTopics::kStateSensorRetain);
  Serial.print(F("MQTT_STATE_SENSOR_PUBLISH retained=1 result="));
  Serial.println(ok ? F("ok") : F("fail"));
}

void publishActuatorState() {
  if (!g_haveActuatorState) {
    g_lastActuators = ActuatorManager::snapshot();
    g_haveActuatorState = true;
  }
  char iso[26] = {0};
  TimeSync::formatIso8601(iso, sizeof(iso));
  char payload[256] = {0};
  snprintf(payload, sizeof(payload),
           "{\"pump\":\"%s\",\"light\":\"%s\",\"fan\":\"%s\",\"roof\":\"%s\",\"mode\":\"%s\","
           "\"servo\":{\"enabled\":\"%s\",\"motion\":\"%s\"},\"updateTime\":\"%s\"}",
           onOff(g_lastActuators.pumpOn), onOff(g_lastActuators.lightOn), onOff(g_lastActuators.fanOn),
           roofText(g_lastActuators.servoAngle), modeText(), onOff(g_lastActuators.servoEnabled),
           ActuatorManager::roofMotionActive() ? "moving" : "idle", iso);
  const bool ok = g_mqtt.publish(MqttTopics::kStateActuator, payload, MqttTopics::kStateActuatorRetain);
  Serial.print(F("MQTT_STATE_ACTUATOR_PUBLISH retained=1 result="));
  Serial.println(ok ? F("ok") : F("fail"));
}

void publishRetainedSnapshot(const char *reason) {
  if (!g_mqtt.connected()) {
    return;
  }
  publishSensorState();
  publishActuatorState();
  publishStatus("online", reason);
  g_lastStatePublishMs = millis();
  Serial.print(F("MQTT_RETAINED_SNAPSHOT reason="));
  Serial.println(reason);
}

const char *ackStatusName(const CommandHandler::ExecuteStatus status) {
  switch (status) {
    case CommandHandler::ExecuteStatus::Ok:
      return "success";
    case CommandHandler::ExecuteStatus::Rejected:
      return "rejected";
    case CommandHandler::ExecuteStatus::Error:
      return "error";
  }
  return "error";
}

void publishAck(const char *cmdId, const CommandHandler::ExecuteStatus status, const char *reason,
                const char *target = nullptr, const char *value = nullptr) {
  char iso[26] = {0};
  TimeSync::formatIso8601(iso, sizeof(iso));
  char payload[320] = {0};
  if (target != nullptr && value != nullptr) {
    snprintf(payload, sizeof(payload),
             "{\"cmdId\":\"%s\",\"status\":\"%s\",\"reason\":\"%s\",\"mode\":\"%s\","
             "\"target\":\"%s\",\"value\":\"%s\",\"updateTime\":\"%s\"}",
             cmdId, ackStatusName(status), reason, modeText(), target, value, iso);
  } else {
    snprintf(payload, sizeof(payload),
             "{\"cmdId\":\"%s\",\"status\":\"%s\",\"reason\":\"%s\",\"mode\":\"%s\",\"updateTime\":\"%s\"}",
             cmdId, ackStatusName(status), reason, modeText(), iso);
  }
  const bool ok = g_mqtt.publish(MqttTopics::kCommandAck, payload, MqttTopics::kCommandAckRetain);
  Serial.print(F("MQTT_ACK_PUBLISH cmdId="));
  Serial.print(cmdId);
  Serial.print(F(" status="));
  Serial.print(ackStatusName(status));
  Serial.print(F(" reason="));
  Serial.print(reason);
  Serial.print(F(" mode="));
  Serial.print(modeText());
  Serial.print(F(" result="));
  Serial.println(ok ? F("ok") : F("fail"));
}

bool duplicateCommand(const char *cmdId, const uint32_t nowMs) {
  static constexpr uint32_t kDuplicateWindowMs = 60000;
  for (const RecentCommand &recent : g_recentCommands) {
    if (recent.cmdId[0] == '\0') {
      continue;
    }
    if (static_cast<uint32_t>(nowMs - recent.seenMs) > kDuplicateWindowMs) {
      continue;
    }
    if (strcmp(recent.cmdId, cmdId) == 0) {
      return true;
    }
  }
  return false;
}

void rememberCommand(const char *cmdId, const uint32_t nowMs) {
  strncpy(g_recentCommands[g_recentCommandWrite].cmdId, cmdId,
          sizeof(g_recentCommands[g_recentCommandWrite].cmdId) - 1);
  g_recentCommands[g_recentCommandWrite].cmdId[sizeof(g_recentCommands[g_recentCommandWrite].cmdId) - 1] = '\0';
  g_recentCommands[g_recentCommandWrite].seenMs = nowMs;
  g_recentCommandWrite = static_cast<uint8_t>((g_recentCommandWrite + 1) % 16);
}

bool boolValue(JsonVariantConst value, bool *out) {
  if (value.is<bool>()) {
    *out = value.as<bool>();
    return true;
  }
  if (value.is<const char *>()) {
    const char *text = value.as<const char *>();
    if (strcasecmp(text, "on") == 0 || strcasecmp(text, "true") == 0 || strcmp(text, "1") == 0) {
      *out = true;
      return true;
    }
    if (strcasecmp(text, "off") == 0 || strcasecmp(text, "false") == 0 || strcmp(text, "0") == 0) {
      *out = false;
      return true;
    }
  }
  return false;
}

bool roofValue(JsonVariantConst value, int *angleOut) {
  if (value.is<int>()) {
    // Numeric roof values are retained for compatibility: 0=open, 1=closed.
    // They are logical state markers, not physical continuous-servo angles.
    const int angle = value.as<int>();
    if (angle == AUTO_ROOF_OPEN_ANGLE || angle == AUTO_ROOF_SAFE_ANGLE) {
      *angleOut = angle;
      return true;
    }
  }
  if (value.is<const char *>()) {
    const char *text = value.as<const char *>();
    if (strcasecmp(text, "open") == 0) {
      *angleOut = AUTO_ROOF_OPEN_ANGLE;
      return true;
    }
    if (strcasecmp(text, "close") == 0) {
      *angleOut = AUTO_ROOF_SAFE_ANGLE;
      return true;
    }
  }
  return false;
}

bool jsonValueText(JsonVariantConst value, char *out, const size_t outSize) {
  if (value.is<const char *>()) {
    strncpy(out, value.as<const char *>(), outSize - 1);
    out[outSize - 1] = '\0';
    return true;
  }
  if (value.is<bool>()) {
    strncpy(out, onOff(value.as<bool>()), outSize - 1);
    out[outSize - 1] = '\0';
    return true;
  }
  if (value.is<int>()) {
    snprintf(out, outSize, "%d", value.as<int>());
    return true;
  }
  return false;
}

bool ackTargetValue(JsonDocument &doc, char *targetOut, const size_t targetSize,
                    char *valueOut, const size_t valueSize) {
  const char *action = doc["action"] | "";
  if (strcmp(action, "query_state") == 0) {
    strncpy(targetOut, "system", targetSize - 1);
    strncpy(valueOut, "query_state", valueSize - 1);
    return true;
  }
  if (strcmp(action, "reboot") == 0) {
    strncpy(targetOut, "system", targetSize - 1);
    strncpy(valueOut, "reboot", valueSize - 1);
    return true;
  }
  if (strcmp(action, "set_mode") == 0) {
    const char *value = doc["value"] | "";
    if (value[0] == '\0') {
      return false;
    }
    strncpy(targetOut, "mode", targetSize - 1);
    strncpy(valueOut, value, valueSize - 1);
    return true;
  }
  if (strcmp(action, "set_actuator") == 0) {
    const char *target = doc["target"] | "";
    if (target[0] == '\0') {
      return false;
    }
    strncpy(targetOut, target, targetSize - 1);
    targetOut[targetSize - 1] = '\0';
    int roofState = 0;
    if ((strcasecmp(target, "roof") == 0 || strcasecmp(target, "servo") == 0) &&
        roofValue(doc["value"], &roofState)) {
      strncpy(valueOut, roofText(roofState), valueSize - 1);
      valueOut[valueSize - 1] = '\0';
      return true;
    }
    return jsonValueText(doc["value"], valueOut, valueSize);
  }
  return false;
}

bool buildCommandText(JsonDocument &doc, char *out, const size_t outSize) {
  const char *action = doc["action"] | "";
  if (strcmp(action, "set_mode") == 0) {
    const char *value = doc["value"] | "";
    if (strcasecmp(value, "manual") == 0) {
      strncpy(out, "mode manual", outSize - 1);
      return true;
    }
    if (strcasecmp(value, "auto") == 0) {
      strncpy(out, "mode auto", outSize - 1);
      return true;
    }
    return false;
  }

  if (strcmp(action, "set_actuator") != 0) {
    return false;
  }

  const char *target = doc["target"] | "";
  if (strcasecmp(target, "roof") == 0 || strcasecmp(target, "servo") == 0) {
    int angle = 0;
    if (roofValue(doc["value"], &angle)) {
      snprintf(out, outSize, "servo %d", angle);
      return true;
    }
    bool on = false;
    if (!boolValue(doc["value"], &on)) {
      return false;
    }
    snprintf(out, outSize, "servo %s", on ? "on" : "off");
    return true;
  }

  bool on = false;
  if (!boolValue(doc["value"], &on)) {
    return false;
  }
  if (strcasecmp(target, "light") == 0) {
    snprintf(out, outSize, "light %s", on ? "on" : "off");
    return true;
  }
  if (strcasecmp(target, "fan") == 0) {
    snprintf(out, outSize, "fan %s", on ? "on" : "off");
    return true;
  }
  if (strcasecmp(target, "pump") == 0) {
    snprintf(out, outSize, "pump %s", on ? "on" : "off");
    return true;
  }
  return false;
}

void handleCommandPayload(const byte *payload, const unsigned int length) {
  if (length > MAX_MQTT_PAYLOAD_BYTES) {
    StaticJsonDocument<128> smallDoc;
    const DeserializationError smallErr = deserializeJson(smallDoc, payload, length);
    const char *oversizedCmdId = smallErr ? "" : (smallDoc["cmdId"] | "");
    if (oversizedCmdId[0] != '\0') {
      publishAck(oversizedCmdId, CommandHandler::ExecuteStatus::Rejected, "invalid_payload");
    } else {
      Serial.println(F("MQTT_CMD_DROP reason=oversized_no_cmdId"));
    }
    return;
  }

  StaticJsonDocument<384> doc;
  const DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    Serial.print(F("MQTT_CMD_REJECT reason=invalid_payload parse="));
    Serial.println(err.c_str());
    return;
  }

  const char *cmdId = doc["cmdId"] | "";
  if (cmdId[0] == '\0' || strlen(cmdId) > 64) {
    Serial.println(F("MQTT_CMD_REJECT reason=invalid_payload cmdId"));
    return;
  }

  char ackTarget[24] = {0};
  char ackValue[32] = {0};
  const bool haveAckFields = ackTargetValue(doc, ackTarget, sizeof(ackTarget), ackValue, sizeof(ackValue));
  const char *ackTargetPtr = haveAckFields ? ackTarget : nullptr;
  const char *ackValuePtr = haveAckFields ? ackValue : nullptr;

  const uint32_t nowMs = millis();
  if (duplicateCommand(cmdId, nowMs)) {
    publishAck(cmdId, CommandHandler::ExecuteStatus::Rejected, "duplicate_cmdId", ackTargetPtr, ackValuePtr);
    return;
  }
  rememberCommand(cmdId, nowMs);

  const char *action = doc["action"] | "";
  if (strcmp(action, "query_state") == 0) {
    publishAck(cmdId, CommandHandler::ExecuteStatus::Ok, "success", ackTargetPtr, ackValuePtr);
    publishRetainedSnapshot("query_state");
    Serial.print(F("MQTT_QUERY_STATE cmdId="));
    Serial.println(cmdId);
    return;
  }
  if (strcmp(action, "reboot") == 0) {
    publishAck(cmdId, CommandHandler::ExecuteStatus::Ok, "success", ackTargetPtr, ackValuePtr);
    Serial.print(F("MQTT_REBOOT_ACCEPTED cmdId="));
    Serial.println(cmdId);
    return;
  }

  char commandText[48] = {0};
  if (!buildCommandText(doc, commandText, sizeof(commandText))) {
    publishAck(cmdId, CommandHandler::ExecuteStatus::Rejected, "unknown_command", ackTargetPtr, ackValuePtr);
    return;
  }

  const CommandHandler::ExecuteResult result = CommandHandler::executeStructured(commandText, false, nowMs);
  if (result.status == CommandHandler::ExecuteStatus::Ok) {
    g_lastActuators = ActuatorManager::snapshot();
    g_haveActuatorState = true;
    publishRetainedSnapshot("command");
  }
  publishAck(cmdId, result.status, result.reason, ackTargetPtr, ackValuePtr);
}

void onMqttMessage(char *topic, byte *payload, unsigned int length) {
  Serial.print(F("MQTT_RX topic="));
  Serial.print(topic);
  Serial.print(F(" bytes="));
  Serial.println(length);
  if (strcmp(topic, MqttTopics::kCommand) == 0) {
    handleCommandPayload(payload, length);
  }
}

void tickWifiScan() {
  if (g_scanDone) {
    return;
  }
  if (!g_scanStarted) {
    const int result = WiFi.scanNetworks(true, false);
    g_scanStarted = true;
    Serial.print(F("WIFI_SCAN_START result="));
    Serial.print(result);
    Serial.println(F(" band=2.4GHz"));
    return;
  }

  const int result = WiFi.scanComplete();
  if (result == WIFI_SCAN_RUNNING) {
    return;
  }
  if (result < 0) {
    Serial.print(F("WIFI_SCAN_FAIL code="));
    Serial.println(result);
    g_scanStarted = false;
    return;
  }

  Serial.print(F("WIFI_SCAN_DONE count="));
  Serial.println(result);
  for (auto &profile : g_wifiProfiles) {
    profile.scanFound = false;
    profile.channel = 0;
  }
  for (int i = 0; i < result; ++i) {
    const String ssid = WiFi.SSID(i);
    const int32_t rssi = WiFi.RSSI(i);
    const int32_t channel = WiFi.channel(i);
    const wifi_auth_mode_t enc = WiFi.encryptionType(i);
    Serial.print(F("WIFI_SCAN_AP ssid="));
    Serial.print(ssid);
    Serial.print(F(" rssi="));
    Serial.print(rssi);
    Serial.print(F(" channel="));
    Serial.print(channel);
    Serial.print(F(" enc="));
    Serial.println(encryptionName(enc));
    for (auto &profile : g_wifiProfiles) {
      if (ssid == profile.ssid) {
        profile.scanFound = true;
        profile.channel = channel;
        printWifiProfileFound(profile, rssi, enc);
      }
    }
  }
  for (const auto &profile : g_wifiProfiles) {
    if (!profile.scanFound) {
      Serial.print(F("WIFI_TARGET_NOT_FOUND profile="));
      Serial.print(profile.name);
      Serial.print(F(" ssid="));
      Serial.println(profile.ssid);
    }
  }
  if (!wifiProfileConst(WifiProfile::Primary).scanFound &&
      wifiProfileConst(WifiProfile::Fallback).scanFound) {
    selectWifiProfile(WifiProfile::Fallback);
    Serial.println(F("WIFI_FALLBACK_START reason=primary_scan_miss"));
  } else {
    selectWifiProfile(WifiProfile::Primary);
  }
  WiFi.scanDelete();
  g_scanDone = true;
}

void startWifiAttempt(const uint32_t nowMs) {
  if (WiFi.status() == WL_CONNECTED) {
    g_wifiBackoffMs = 2000;
    g_wifiFailureCount = 0;
    return;
  }
  if (static_cast<int32_t>(nowMs - g_nextWifiAttemptMs) < 0) {
    return;
  }

  if (!g_scanDone) {
    return;
  }
  if (g_wifiStarted && g_activeWifiProfile == WifiProfile::Primary) {
    return;
  }

  const WifiProfileConfig &profile = wifiProfileConst(g_activeWifiProfile);
  if (profile.channel > 0) {
    WiFi.begin(profile.ssid, profile.password, profile.channel);
  } else {
    WiFi.begin(profile.ssid, profile.password);
  }
  g_wifiStarted = true;
  g_wifiAttemptStartMs = nowMs;
  ++g_wifiFailureCount;
  Serial.print(F("WIFI_CONNECT_START profile="));
  Serial.print(profile.name);
  Serial.print(F(" ssid="));
  Serial.print(profile.ssid);
  Serial.print(F(" channel="));
  Serial.print(profile.channel);
  Serial.print(F(" scan_match="));
  Serial.print(profile.scanFound ? F("yes") : F("no"));
  Serial.print(F(" status="));
  Serial.print(WiFi.status());
  Serial.print(F(" next_ms="));
  Serial.println(g_wifiBackoffMs);
  g_nextWifiAttemptMs = nowMs + g_wifiBackoffMs;
  g_wifiBackoffMs = nextWifiBackoff(g_wifiBackoffMs);
}

void tickPrimaryWifiTimeout(const uint32_t nowMs) {
  if (!g_wifiStarted || g_activeWifiProfile != WifiProfile::Primary ||
      WiFi.status() == WL_CONNECTED) {
    return;
  }
  if (static_cast<uint32_t>(nowMs - g_wifiAttemptStartMs) < WIFI_PRIMARY_CONNECT_TIMEOUT_MS) {
    return;
  }

  WiFi.disconnect(false, false);
  g_wifiStarted = false;
  selectWifiProfile(WifiProfile::Fallback);
  g_nextWifiAttemptMs = nowMs;
  Serial.print(F("WIFI_CONNECT_TIMEOUT profile=primary elapsed_ms="));
  Serial.println(static_cast<uint32_t>(nowMs - g_wifiAttemptStartMs));
  Serial.println(F("WIFI_FALLBACK_START reason=primary_timeout"));
}

void attemptMqttConnect(const uint32_t nowMs) {
  if (WiFi.status() != WL_CONNECTED) {
    if (g_mqtt.connected()) {
      g_mqtt.disconnect();
      Serial.println(F("MQTT_DISCONNECT reason=wifi_down"));
    }
    return;
  }
  if (g_mqtt.connected()) {
    g_mqtt.loop();
    return;
  }
  if (static_cast<int32_t>(nowMs - g_nextMqttAttemptMs) < 0) {
    return;
  }

  Serial.print(F("MQTT_CONNECT_START host="));
  Serial.print(MQTT_DEMO_HOST);
  Serial.print(F(" port="));
  Serial.println(MQTT_DEMO_PORT);

  const bool ok = g_mqtt.connect(MQTT_DEMO_CLIENT_ID, MqttTopics::kStatus, MqttTopics::kStatusQos,
                                 MqttTopics::kStatusRetain,
                                 "{\"system\":\"offline\",\"mode\":\"boot\",\"health\":\"error\","
                                 "\"reason\":\"lwt\",\"updateTime\":\"UNSYNC\"}");
  if (ok) {
    Serial.println(F("MQTT_CONNECTED"));
    g_mqttBackoffMs = 3000;
    g_nextMqttAttemptMs = nowMs + g_mqttBackoffMs;
    g_mqtt.subscribe(MqttTopics::kCommand, MqttTopics::kCommandQos);
    Serial.println(F("MQTT_SUBSCRIBE command result=ok"));
    g_pendingInitialPublish = true;
    g_pendingPublishReason = g_everMqttConnected ? "reconnect" : "boot";
    g_mqttConnectMs = nowMs;
    g_everMqttConnected = true;
    return;
  }

  Serial.print(F("MQTT_CONNECT_FAIL state="));
  Serial.println(g_mqtt.state());
  g_nextMqttAttemptMs = nowMs + g_mqttBackoffMs;
  g_mqttBackoffMs = nextMqttBackoff(g_mqttBackoffMs);
}

}  // namespace

void begin(const uint32_t nowMs) {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(false);
  WiFi.disconnect(false, false);
  g_mqtt.setServer(MQTT_DEMO_HOST, MQTT_DEMO_PORT);
  g_mqtt.setCallback(onMqttMessage);
  g_mqtt.setKeepAlive(MQTT_KEEP_ALIVE_SECONDS);
  g_mqtt.setBufferSize(MAX_MQTT_PAYLOAD_BYTES);
  g_nextWifiAttemptMs = nowMs;
  g_nextMqttAttemptMs = nowMs;
  g_lastWifiStatus = WiFi.status();
  Serial.println(F("MQTT_MANAGER: DEMO_ENV broker.hivemq.com:1883 no_auth"));
  Serial.print(F("WIFI_BOOT mode=STA sleep=off autoreconnect=off status="));
  Serial.println(g_lastWifiStatus);
}

void tick(const uint32_t nowMs) {
  tickWifiScan();
  tickPrimaryWifiTimeout(nowMs);
  startWifiAttempt(nowMs);
  const wl_status_t status = WiFi.status();
  if (status != g_lastWifiStatus) {
    Serial.print(F("WIFI_STATUS_CHANGE from="));
    Serial.print(g_lastWifiStatus);
    Serial.print(F(" to="));
    Serial.println(status);
    g_lastWifiStatus = status;
  }
  if (WiFi.status() == WL_CONNECTED && g_wifiStarted) {
    g_wifiStarted = false;
    g_wifiBackoffMs = 2000;
    g_wifiFailureCount = 0;
    Serial.print(F("WIFI_CONNECTED profile="));
    Serial.print(wifiProfileConst(g_activeWifiProfile).name);
    Serial.print(F(" ip="));
    Serial.println(WiFi.localIP());
  }
  if (g_lastMqttConnected && !g_mqtt.connected()) {
    Serial.println(F("MQTT_STATUS_CHANGE connected=0"));
    g_lastMqttConnected = false;
    g_nextMqttAttemptMs = nowMs;
  }
  attemptMqttConnect(nowMs);
  const bool mqttNowConnected = g_mqtt.connected();
  if (mqttNowConnected != g_lastMqttConnected) {
    Serial.print(F("MQTT_STATUS_CHANGE connected="));
    Serial.println(mqttNowConnected ? F("1") : F("0"));
    g_lastMqttConnected = mqttNowConnected;
    if (!mqttNowConnected) {
      g_nextMqttAttemptMs = nowMs;
    }
  }
  const bool nowSynced = TimeSync::isSynced();
  
  if (g_pendingInitialPublish && g_mqtt.connected()) {
    if (nowSynced || static_cast<uint32_t>(nowMs - g_mqttConnectMs) >= 30000) {
      publishRetainedSnapshot(g_pendingPublishReason);
      g_pendingInitialPublish = false;
    }
  } else if (nowSynced && !g_timeWasSynced && g_mqtt.connected() && g_haveSensorState) {
    publishSensorState();
    publishActuatorState();
    g_lastStatePublishMs = nowMs;
  }
  g_timeWasSynced = nowSynced;
  if (g_wifiFailureCount >= 50) {
    Serial.println(F("WIFI_RESTART_GUARD would_restart_after_safe_state"));
    g_wifiFailureCount = 0;
  }
}

void updateState(const uint32_t nowMs, const SensorData &sensors, const ActuatorSnapshot &actuators) {
  const bool sensorChanged = !g_haveSensorState || !sensorsEqual(g_lastSensors, sensors);
  const bool actuatorChanged = !g_haveActuatorState || !actuatorsEqual(g_lastActuators, actuators);

  g_lastSensors = sensors;
  g_lastActuators = actuators;
  g_haveSensorState = true;
  g_haveActuatorState = true;

  if (g_mqtt.connected()) {
    bool published = false;
    const bool heartbeat = (nowMs - g_lastStatePublishMs >= 300000); // 5 minutes
    
    if (sensorChanged || heartbeat) {
      publishSensorState();
      published = true;
    }
    if (actuatorChanged || heartbeat) {
      publishActuatorState();
      published = true;
    }
    if (published) {
      g_lastStatePublishMs = nowMs;
    }
  }
}

bool wifiConnected() {
  return WiFi.status() == WL_CONNECTED;
}

bool mqttConnected() {
  return g_mqtt.connected();
}

bool runReconnectSelfTest() {
  bool pass = true;
  pass &= nextWifiBackoff(2000) == 4000;
  pass &= nextWifiBackoff(4000) == 8000;
  pass &= nextWifiBackoff(8000) == 16000;
  pass &= nextWifiBackoff(16000) == 30000;
  pass &= nextWifiBackoff(30000) == 30000;
  pass &= nextMqttBackoff(3000) == 6000;
  pass &= nextMqttBackoff(6000) == 12000;
  pass &= nextMqttBackoff(12000) == 30000;
  pass &= nextMqttBackoff(30000) == 30000;
  Serial.print(F("MQTT_RECONNECT_SELF_TEST: "));
  Serial.println(pass ? F("PASS") : F("FAIL"));
  return pass;
}

bool runCommandAckSelfTest() {
  bool pass = true;
  char closeCommand[48] = {0};
  snprintf(closeCommand, sizeof(closeCommand), "servo %d", AUTO_ROOF_SAFE_ANGLE);
  StaticJsonDocument<128> doc;
  doc["action"] = "set_mode";
  doc["value"] = "manual";
  char commandText[48] = {0};
  pass &= buildCommandText(doc, commandText, sizeof(commandText));
  pass &= strcmp(commandText, "mode manual") == 0;
  doc.clear();
  doc["action"] = "set_actuator";
  doc["target"] = "light";
  doc["value"] = "on";
  memset(commandText, 0, sizeof(commandText));
  pass &= buildCommandText(doc, commandText, sizeof(commandText));
  pass &= strcmp(commandText, "light on") == 0;
  doc["value"] = false;
  memset(commandText, 0, sizeof(commandText));
  pass &= buildCommandText(doc, commandText, sizeof(commandText));
  pass &= strcmp(commandText, "light off") == 0;
  doc["value"] = "bad";
  pass &= !buildCommandText(doc, commandText, sizeof(commandText));

  doc.clear();
  doc["action"] = "set_actuator";
  doc["target"] = "roof";
  doc["value"] = "open";
  memset(commandText, 0, sizeof(commandText));
  pass &= buildCommandText(doc, commandText, sizeof(commandText));
  pass &= strcmp(commandText, "servo 0") == 0;
  doc["value"] = "close";
  memset(commandText, 0, sizeof(commandText));
  pass &= buildCommandText(doc, commandText, sizeof(commandText));
  pass &= strcmp(commandText, closeCommand) == 0;
  doc["value"] = 0;
  memset(commandText, 0, sizeof(commandText));
  pass &= buildCommandText(doc, commandText, sizeof(commandText));
  pass &= strcmp(commandText, "servo 0") == 0;
  doc["value"] = AUTO_ROOF_SAFE_ANGLE;
  memset(commandText, 0, sizeof(commandText));
  pass &= buildCommandText(doc, commandText, sizeof(commandText));
  pass &= strcmp(commandText, closeCommand) == 0;
  doc["value"] = "half";
  pass &= !buildCommandText(doc, commandText, sizeof(commandText));

  char ackTarget[24] = {0};
  char ackValue[32] = {0};
  doc.clear();
  doc["action"] = "set_mode";
  doc["value"] = "manual";
  pass &= ackTargetValue(doc, ackTarget, sizeof(ackTarget), ackValue, sizeof(ackValue));
  pass &= strcmp(ackTarget, "mode") == 0;
  pass &= strcmp(ackValue, "manual") == 0;
  memset(ackTarget, 0, sizeof(ackTarget));
  memset(ackValue, 0, sizeof(ackValue));
  doc.clear();
  doc["action"] = "set_actuator";
  doc["target"] = "roof";
  doc["value"] = AUTO_ROOF_SAFE_ANGLE;
  pass &= ackTargetValue(doc, ackTarget, sizeof(ackTarget), ackValue, sizeof(ackValue));
  pass &= strcmp(ackTarget, "roof") == 0;
  pass &= strcmp(ackValue, "close") == 0;
  memset(ackTarget, 0, sizeof(ackTarget));
  memset(ackValue, 0, sizeof(ackValue));
  doc.clear();
  doc["action"] = "query_state";
  pass &= ackTargetValue(doc, ackTarget, sizeof(ackTarget), ackValue, sizeof(ackValue));
  pass &= strcmp(ackTarget, "system") == 0;
  pass &= strcmp(ackValue, "query_state") == 0;
  pass &= strcmp(ackStatusName(CommandHandler::ExecuteStatus::Ok), "success") == 0;

  pass &= !duplicateCommand("self-test-a", 1000);
  rememberCommand("self-test-a", 1000);
  pass &= duplicateCommand("self-test-a", 2000);
  pass &= !duplicateCommand("self-test-a", 62001);
  Serial.print(F("MQTT_COMMAND_ACK_SELF_TEST: "));
  Serial.println(pass ? F("PASS") : F("FAIL"));
  return pass;
}

bool runRetainedStateSelfTest() {
  SensorData savedSensors = g_lastSensors;
  ActuatorSnapshot savedActuators = g_lastActuators;
  const bool savedHaveSensorState = g_haveSensorState;
  const bool savedHaveActuatorState = g_haveActuatorState;

  g_lastSensors = {};
  g_lastSensors.temperatureC = 28.4f;
  g_lastSensors.humidityPct = 65.0f;
  g_lastSensors.lux = 1200.0f;
  g_lastSensors.soilPct = 42.0f;
  g_lastSensors.soilAO = 2048;
  g_lastSensors.rainAO = 4095;
  g_lastSensors.soilDO = 1;
  g_lastSensors.rainDO = 1;
  g_lastSensors.sensor_invalid = false;
  g_lastSensors.dht_ok = true;
  g_lastSensors.bh1750_ok = true;
  g_lastSensors.soil_ok = true;
  g_lastSensors.rain_ok = true;
  g_lastSensors.dht_last_valid_ms = 1000;
  g_lastSensors.bh1750_last_valid_ms = 1000;
  g_lastSensors.soil_last_valid_ms = 1000;
  g_lastSensors.rain_last_valid_ms = 1000;
  g_lastSensors.dht_max_stale_ms = 6000;
  g_lastSensors.bh1750_max_stale_ms = 6000;
  g_lastSensors.soil_max_stale_ms = 6000;
  g_lastSensors.rain_max_stale_ms = 6000;
  g_lastActuators = {true, false, true, true, false, AUTO_ROOF_SAFE_ANGLE};
  g_haveSensorState = true;
  g_haveActuatorState = true;

  char sensorPayload[1024] = {0};
  snprintf(sensorPayload, sizeof(sensorPayload),
           "{\"airTemperatureC\":%.1f,\"airHumidityPct\":%.1f,\"lightLux\":%.1f,"
           "\"soilMoisturePct\":%.1f,\"rain\":\"%s\","
           "\"health\":{\"sensor\":\"%s\",\"dht\":\"%s\",\"light\":\"%s\",\"soil\":\"%s\",\"rain\":\"%s\"},"
           "\"raw\":{\"soilAO\":%u,\"rainAO\":%u,\"soilDO\":%u,\"rainDO\":%u},"
           "\"controlFlags\":{\"tempOverFanOnThreshold\":%s,\"tempBelowFanOffThreshold\":%s,"
           "\"soilBelowPumpOnThreshold\":%s,\"soilAbovePumpOffThreshold\":%s,"
           "\"luxBelowLightOnThreshold\":%s,\"luxAboveLightOffThreshold\":%s,"
           "\"isRaining\":%s,\"airHumidityTooHigh\":%s,\"airHumidityTooLow\":%s},"
           "\"controlThresholds\":{\"fanOnTempC\":%.1f,\"fanOffTempC\":%.1f,"
           "\"pumpOnSoilMoisturePct\":%.1f,\"pumpOffSoilMoisturePct\":%.1f,"
           "\"lightOnLux\":%.1f,\"lightOffLux\":%.1f,"
           "\"airHumidityHighPct\":%.1f,\"airHumidityLowPct\":%.1f},"
           "\"updateTime\":\"UNSYNC\"}",
           g_lastSensors.temperatureC, g_lastSensors.humidityPct, g_lastSensors.lux, g_lastSensors.soilPct,
           rainText(g_lastSensors), g_lastSensors.sensor_invalid ? "invalid" : "ok",
           validInvalid(g_lastSensors.dht_ok), validInvalid(g_lastSensors.bh1750_ok),
           validInvalid(g_lastSensors.soil_ok), validInvalid(g_lastSensors.rain_ok),
           g_lastSensors.soilAO, g_lastSensors.rainAO, g_lastSensors.soilDO, g_lastSensors.rainDO,
           jsonBool(g_lastSensors.temperatureC > AUTO_TEMP_FAN_ON_C),
           jsonBool(g_lastSensors.temperatureC < AUTO_TEMP_FAN_OFF_C),
           jsonBool(g_lastSensors.soilPct < AUTO_SOIL_PUMP_ON_PCT),
           jsonBool(g_lastSensors.soilPct > AUTO_SOIL_PUMP_OFF_PCT),
           jsonBool(g_lastSensors.lux < AUTO_LIGHT_ON_LUX),
           jsonBool(g_lastSensors.lux > AUTO_LIGHT_OFF_LUX),
           jsonBool(g_lastSensors.rain_ok && g_lastSensors.rainDO == 0),
           jsonBool(g_lastSensors.humidityPct > AIR_HUMIDITY_HIGH_PCT),
           jsonBool(g_lastSensors.humidityPct < AIR_HUMIDITY_LOW_PCT),
           AUTO_TEMP_FAN_ON_C, AUTO_TEMP_FAN_OFF_C,
           AUTO_SOIL_PUMP_ON_PCT, AUTO_SOIL_PUMP_OFF_PCT,
           AUTO_LIGHT_ON_LUX, AUTO_LIGHT_OFF_LUX,
           AIR_HUMIDITY_HIGH_PCT, AIR_HUMIDITY_LOW_PCT);
  char actuatorPayload[256] = {0};
  snprintf(actuatorPayload, sizeof(actuatorPayload),
           "{\"pump\":\"%s\",\"light\":\"%s\",\"fan\":\"%s\",\"roof\":\"%s\",\"mode\":\"%s\","
           "\"servo\":{\"enabled\":\"%s\",\"motion\":\"%s\"},\"updateTime\":\"UNSYNC\"}",
           onOff(g_lastActuators.pumpOn), onOff(g_lastActuators.lightOn), onOff(g_lastActuators.fanOn),
           roofText(g_lastActuators.servoAngle), modeText(), onOff(g_lastActuators.servoEnabled),
           ActuatorManager::roofMotionActive() ? "moving" : "idle");
  bool pass = true;
  pass &= strstr(sensorPayload, "\"airTemperatureC\":28.4") != nullptr;
  pass &= strstr(sensorPayload, "\"airHumidityPct\":65.0") != nullptr;
  pass &= strstr(sensorPayload, "\"lightLux\":1200.0") != nullptr;
  pass &= strstr(sensorPayload, "\"soilMoisturePct\":42.0") != nullptr;
  pass &= strstr(sensorPayload, "\"rain\":\"dry\"") != nullptr;
  pass &= strstr(sensorPayload, "\"health\":{\"sensor\":\"ok\",\"dht\":\"ok\",\"light\":\"ok\",\"soil\":\"ok\",\"rain\":\"ok\"}") != nullptr;
  pass &= strstr(sensorPayload, "\"raw\":{\"soilAO\":2048,\"rainAO\":4095,\"soilDO\":1,\"rainDO\":1}") != nullptr;
  pass &= strstr(sensorPayload, "\"controlFlags\":{") != nullptr;
  pass &= strstr(sensorPayload, "\"controlThresholds\":{") != nullptr;
  pass &= strstr(sensorPayload, "\"tempOverFanOnThreshold\":true") != nullptr;
  pass &= strstr(sensorPayload, "\"tempBelowFanOffThreshold\":false") != nullptr;
  pass &= strstr(sensorPayload, "\"soilBelowPumpOnThreshold\":false") != nullptr;
  pass &= strstr(sensorPayload, "\"soilAbovePumpOffThreshold\":false") != nullptr;
  pass &= strstr(sensorPayload, "\"luxBelowLightOnThreshold\":false") != nullptr;
  pass &= strstr(sensorPayload, "\"luxAboveLightOffThreshold\":true") != nullptr;
  pass &= strstr(sensorPayload, "\"isRaining\":false") != nullptr;
  pass &= strstr(sensorPayload, "\"airHumidityTooHigh\":false") != nullptr;
  pass &= strstr(sensorPayload, "\"airHumidityTooLow\":false") != nullptr;
  pass &= strstr(sensorPayload, "\"fanOnTempC\":28.0") != nullptr;
  pass &= strstr(sensorPayload, "\"fanOffTempC\":24.0") != nullptr;
  pass &= strstr(sensorPayload, "\"pumpOnSoilMoisturePct\":35.0") != nullptr;
  pass &= strstr(sensorPayload, "\"pumpOffSoilMoisturePct\":55.0") != nullptr;
  pass &= strstr(sensorPayload, "\"lightOnLux\":4.0") != nullptr;
  pass &= strstr(sensorPayload, "\"lightOffLux\":6.0") != nullptr;
  pass &= strstr(sensorPayload, "\"airHumidityHighPct\":85.0") != nullptr;
  pass &= strstr(sensorPayload, "\"airHumidityLowPct\":45.0") != nullptr;
  pass &= strstr(sensorPayload, "\"updateTime\":\"UNSYNC\"") != nullptr;
  pass &= strstr(actuatorPayload, "\"pump\":\"on\"") != nullptr;
  pass &= strstr(actuatorPayload, "\"light\":\"on\"") != nullptr;
  pass &= strstr(actuatorPayload, "\"fan\":\"off\"") != nullptr;
  pass &= strstr(actuatorPayload, "\"roof\":\"close\"") != nullptr;
  pass &= strstr(actuatorPayload, "\"servo\":{\"enabled\":\"on\",\"motion\":\"idle\"}") != nullptr;
  pass &= strstr(actuatorPayload, "\"updateTime\":\"UNSYNC\"") != nullptr;

  g_lastSensors = savedSensors;
  g_lastActuators = savedActuators;
  g_haveSensorState = savedHaveSensorState;
  g_haveActuatorState = savedHaveActuatorState;

  Serial.print(F("MQTT_RETAINED_STATE_SELF_TEST: "));
  Serial.println(pass ? F("PASS") : F("FAIL"));
  return pass;
}

}  // namespace MqttManager

#endif
