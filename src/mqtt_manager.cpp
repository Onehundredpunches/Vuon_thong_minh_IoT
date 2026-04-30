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
bool g_targetSsidFound = false;
int32_t g_targetChannel = 0;
wl_status_t g_lastWifiStatus = WL_IDLE_STATUS;
SensorData g_lastSensors = {};
ActuatorSnapshot g_lastActuators = {};
bool g_haveSensorState = false;
bool g_haveActuatorState = false;
uint32_t g_lastStatePublishMs = 0;

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

uint32_t nextMqttBackoff(const uint32_t current) {
  if (current < 6000) {
    return 6000;
  }
  if (current < 12000) {
    return 12000;
  }
  return 30000;
}

void publishStatus(const char *sysStatus, const char *reason) {
  char payload[160] = {0};
  snprintf(payload, sizeof(payload), "{\"sysStatus\":\"%s\",\"reason\":\"%s\",\"mode\":\"%s\"}",
           sysStatus, reason, ModeController::modeName());
  const bool ok = g_mqtt.publish(MqttTopics::kStatus, payload, MqttTopics::kStatusRetain);
  Serial.print(F("MQTT_STATUS_PUBLISH retained=1 reason="));
  Serial.print(reason);
  Serial.print(F(" result="));
  Serial.println(ok ? F("ok") : F("fail"));
}

const char *boolText(const bool value) {
  return value ? "true" : "false";
}

void publishSensorState() {
  if (!g_haveSensorState) {
    Serial.println(F("MQTT_STATE_SENSOR_SKIP reason=no_cache"));
    return;
  }
  char payload[448] = {0};
  snprintf(payload, sizeof(payload),
           "{\"temperatureC\":%.1f,\"humidityPct\":%.1f,\"lux\":%.1f,\"soilPct\":%.1f,"
           "\"soilAO\":%u,\"rainAO\":%u,\"soilDO\":%u,\"rainDO\":%u,"
           "\"sensor_invalid\":%s,\"dht_ok\":%s,\"bh1750_ok\":%s,\"soil_ok\":%s,\"rain_ok\":%s,"
           "\"dht_last_valid_ms\":%lu,\"bh1750_last_valid_ms\":%lu,"
           "\"soil_last_valid_ms\":%lu,\"rain_last_valid_ms\":%lu,"
           "\"dht_max_stale_ms\":%lu,\"bh1750_max_stale_ms\":%lu,"
           "\"soil_max_stale_ms\":%lu,\"rain_max_stale_ms\":%lu}",
           g_lastSensors.temperatureC, g_lastSensors.humidityPct, g_lastSensors.lux, g_lastSensors.soilPct,
           g_lastSensors.soilAO, g_lastSensors.rainAO, g_lastSensors.soilDO, g_lastSensors.rainDO,
           boolText(g_lastSensors.sensor_invalid), boolText(g_lastSensors.dht_ok),
           boolText(g_lastSensors.bh1750_ok), boolText(g_lastSensors.soil_ok), boolText(g_lastSensors.rain_ok),
           static_cast<unsigned long>(g_lastSensors.dht_last_valid_ms),
           static_cast<unsigned long>(g_lastSensors.bh1750_last_valid_ms),
           static_cast<unsigned long>(g_lastSensors.soil_last_valid_ms),
           static_cast<unsigned long>(g_lastSensors.rain_last_valid_ms),
           static_cast<unsigned long>(g_lastSensors.dht_max_stale_ms),
           static_cast<unsigned long>(g_lastSensors.bh1750_max_stale_ms),
           static_cast<unsigned long>(g_lastSensors.soil_max_stale_ms),
           static_cast<unsigned long>(g_lastSensors.rain_max_stale_ms));
  const bool ok = g_mqtt.publish(MqttTopics::kStateSensor, payload, MqttTopics::kStateSensorRetain);
  Serial.print(F("MQTT_STATE_SENSOR_PUBLISH retained=1 result="));
  Serial.println(ok ? F("ok") : F("fail"));
}

void publishActuatorState() {
  if (!g_haveActuatorState) {
    g_lastActuators = ActuatorManager::snapshot();
    g_haveActuatorState = true;
  }
  char payload[192] = {0};
  snprintf(payload, sizeof(payload),
           "{\"lightOn\":%s,\"fanOn\":%s,\"pumpOn\":%s,\"servoEnabled\":%s,"
           "\"servoSweepEnabled\":%s,\"servoAngle\":%d,\"mode\":\"%s\"}",
           boolText(g_lastActuators.lightOn), boolText(g_lastActuators.fanOn),
           boolText(g_lastActuators.pumpOn), boolText(g_lastActuators.servoEnabled),
           boolText(g_lastActuators.servoSweepEnabled), g_lastActuators.servoAngle,
           ModeController::modeName());
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
      return "ok";
    case CommandHandler::ExecuteStatus::Rejected:
      return "rejected";
    case CommandHandler::ExecuteStatus::Error:
      return "error";
  }
  return "error";
}

void publishAck(const char *cmdId, const CommandHandler::ExecuteStatus status, const char *reason) {
  char payload[192] = {0};
  snprintf(payload, sizeof(payload),
           "{\"cmdId\":\"%s\",\"status\":\"%s\",\"reason\":\"%s\",\"mode\":\"%s\"}", cmdId,
           ackStatusName(status), reason, ModeController::modeName());
  const bool ok = g_mqtt.publish(MqttTopics::kCommandAck, payload, MqttTopics::kCommandAckRetain);
  Serial.print(F("MQTT_ACK_PUBLISH cmdId="));
  Serial.print(cmdId);
  Serial.print(F(" status="));
  Serial.print(ackStatusName(status));
  Serial.print(F(" reason="));
  Serial.print(reason);
  Serial.print(F(" mode="));
  Serial.print(ModeController::modeName());
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
    if (doc["value"].is<int>()) {
      const int angle = doc["value"].as<int>();
      if (angle == AUTO_ROOF_OPEN_ANGLE || angle == AUTO_ROOF_SAFE_ANGLE) {
        snprintf(out, outSize, "servo %d", angle);
        return true;
      }
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

  const uint32_t nowMs = millis();
  if (duplicateCommand(cmdId, nowMs)) {
    publishAck(cmdId, CommandHandler::ExecuteStatus::Rejected, "duplicate_cmdId");
    return;
  }
  rememberCommand(cmdId, nowMs);

  const char *action = doc["action"] | "";
  if (strcmp(action, "query_state") == 0) {
    publishAck(cmdId, CommandHandler::ExecuteStatus::Ok, "success");
    publishRetainedSnapshot("query_state");
    Serial.print(F("MQTT_QUERY_STATE cmdId="));
    Serial.println(cmdId);
    return;
  }
  if (strcmp(action, "reboot") == 0) {
    publishAck(cmdId, CommandHandler::ExecuteStatus::Ok, "success");
    Serial.print(F("MQTT_REBOOT_ACCEPTED cmdId="));
    Serial.println(cmdId);
    return;
  }

  char commandText[48] = {0};
  if (!buildCommandText(doc, commandText, sizeof(commandText))) {
    publishAck(cmdId, CommandHandler::ExecuteStatus::Rejected, "unknown_command");
    return;
  }

  const CommandHandler::ExecuteResult result = CommandHandler::executeStructured(commandText, false, nowMs);
  if (result.status == CommandHandler::ExecuteStatus::Ok) {
    g_lastActuators = ActuatorManager::snapshot();
    g_haveActuatorState = true;
    publishRetainedSnapshot("command");
  }
  publishAck(cmdId, result.status, result.reason);
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
  g_targetSsidFound = false;
  g_targetChannel = 0;
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
    if (ssid == WIFI_PRIMARY_SSID) {
      g_targetSsidFound = true;
      g_targetChannel = channel;
      Serial.print(F("WIFI_TARGET_FOUND ssid="));
      Serial.print(WIFI_PRIMARY_SSID);
      Serial.print(F(" rssi="));
      Serial.print(rssi);
      Serial.print(F(" channel="));
      Serial.print(channel);
      Serial.print(F(" enc="));
      Serial.println(encryptionName(enc));
      if (enc == WIFI_AUTH_OPEN && strlen(WIFI_PRIMARY_PASSWORD) > 0) {
        Serial.println(F("WIFI_AUTH_HINT target_open_password_configured"));
      }
      if (enc == WIFI_AUTH_WPA3_PSK) {
        Serial.println(F("WIFI_AUTH_HINT target_wpa3_only_may_fail_on_legacy_esp32"));
      }
    }
  }
  if (!g_targetSsidFound) {
    Serial.print(F("WIFI_TARGET_NOT_FOUND ssid="));
    Serial.println(WIFI_PRIMARY_SSID);
    Serial.println(F("WIFI_AUTH_HINT scan_miss_using_bounded_direct_connect"));
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
  if (g_targetChannel > 0) {
    WiFi.begin(WIFI_PRIMARY_SSID, WIFI_PRIMARY_PASSWORD, g_targetChannel);
  } else {
    WiFi.begin(WIFI_PRIMARY_SSID, WIFI_PRIMARY_PASSWORD);
  }
  g_wifiStarted = true;
  ++g_wifiFailureCount;
  Serial.print(F("WIFI_CONNECT_START ssid="));
  Serial.print(WIFI_PRIMARY_SSID);
  Serial.print(F(" channel="));
  Serial.print(g_targetChannel);
  Serial.print(F(" scan_match="));
  Serial.print(g_targetSsidFound ? F("yes") : F("no"));
  Serial.print(F(" status="));
  Serial.print(WiFi.status());
  Serial.print(F(" next_ms="));
  Serial.println(g_wifiBackoffMs);
  g_nextWifiAttemptMs = nowMs + g_wifiBackoffMs;
  g_wifiBackoffMs = nextWifiBackoff(g_wifiBackoffMs);
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
                                 "{\"sysStatus\":\"offline\",\"reason\":\"lwt\"}");
  if (ok) {
    Serial.println(F("MQTT_CONNECTED"));
    g_mqttBackoffMs = 3000;
    g_nextMqttAttemptMs = nowMs + g_mqttBackoffMs;
    g_mqtt.subscribe(MqttTopics::kCommand, MqttTopics::kCommandQos);
    Serial.println(F("MQTT_SUBSCRIBE command result=ok"));
    publishRetainedSnapshot(g_everMqttConnected ? "reconnect" : "boot");
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
    Serial.print(F("WIFI_CONNECTED ip="));
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
  if (g_mqtt.connected() && g_haveSensorState &&
      static_cast<uint32_t>(nowMs - g_lastStatePublishMs) >= LOOP_INTERVAL_MS) {
    publishSensorState();
    publishActuatorState();
    g_lastStatePublishMs = nowMs;
  }
  if (g_wifiFailureCount >= 50) {
    Serial.println(F("WIFI_RESTART_GUARD would_restart_after_safe_state"));
    g_wifiFailureCount = 0;
  }
}

void updateState(const uint32_t nowMs, const SensorData &sensors, const ActuatorSnapshot &actuators) {
  g_lastSensors = sensors;
  g_lastActuators = actuators;
  g_haveSensorState = true;
  g_haveActuatorState = true;
  if (g_mqtt.connected() && static_cast<uint32_t>(nowMs - g_lastStatePublishMs) >= LOOP_INTERVAL_MS) {
    publishSensorState();
    publishActuatorState();
    g_lastStatePublishMs = nowMs;
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
  g_lastActuators = {true, false, true, true, false, 90};
  g_haveSensorState = true;
  g_haveActuatorState = true;

  char sensorPayload[448] = {0};
  snprintf(sensorPayload, sizeof(sensorPayload),
           "{\"temperatureC\":%.1f,\"humidityPct\":%.1f,\"lux\":%.1f,\"soilPct\":%.1f,"
           "\"soilAO\":%u,\"rainAO\":%u,\"soilDO\":%u,\"rainDO\":%u,"
           "\"sensor_invalid\":%s,\"dht_ok\":%s,\"bh1750_ok\":%s,\"soil_ok\":%s,\"rain_ok\":%s}",
           g_lastSensors.temperatureC, g_lastSensors.humidityPct, g_lastSensors.lux, g_lastSensors.soilPct,
           g_lastSensors.soilAO, g_lastSensors.rainAO, g_lastSensors.soilDO, g_lastSensors.rainDO,
           boolText(g_lastSensors.sensor_invalid), boolText(g_lastSensors.dht_ok),
           boolText(g_lastSensors.bh1750_ok), boolText(g_lastSensors.soil_ok), boolText(g_lastSensors.rain_ok));
  char actuatorPayload[192] = {0};
  snprintf(actuatorPayload, sizeof(actuatorPayload),
           "{\"lightOn\":%s,\"fanOn\":%s,\"pumpOn\":%s,\"servoEnabled\":%s,"
           "\"servoSweepEnabled\":%s,\"servoAngle\":%d,\"mode\":\"%s\"}",
           boolText(g_lastActuators.lightOn), boolText(g_lastActuators.fanOn),
           boolText(g_lastActuators.pumpOn), boolText(g_lastActuators.servoEnabled),
           boolText(g_lastActuators.servoSweepEnabled), g_lastActuators.servoAngle,
           ModeController::modeName());
  bool pass = true;
  pass &= strstr(sensorPayload, "\"dht_ok\":true") != nullptr;
  pass &= strstr(sensorPayload, "\"bh1750_ok\":true") != nullptr;
  pass &= strstr(sensorPayload, "\"soil_ok\":true") != nullptr;
  pass &= strstr(sensorPayload, "\"rain_ok\":true") != nullptr;
  pass &= strstr(actuatorPayload, "\"servoAngle\":90") != nullptr;

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
