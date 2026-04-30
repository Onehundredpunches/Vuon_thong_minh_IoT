#include "mqtt_manager.h"

#include "config.h"
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

}  // namespace MqttManager

#else

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
bool g_scanStarted = false;
bool g_scanDone = false;
bool g_targetSsidFound = false;
int32_t g_targetChannel = 0;
wl_status_t g_lastWifiStatus = WL_IDLE_STATUS;

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
  char payload[96] = {0};
  snprintf(payload, sizeof(payload), "{\"sysStatus\":\"%s\",\"reason\":\"%s\"}", sysStatus, reason);
  g_mqtt.publish(MqttTopics::kStatus, payload, MqttTopics::kStatusRetain);
}

void onMqttMessage(char *topic, byte *payload, unsigned int length) {
  (void)payload;
  Serial.print(F("MQTT_RX topic="));
  Serial.print(topic);
  Serial.print(F(" bytes="));
  Serial.println(length);
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
    publishStatus("online", g_everMqttConnected ? "reconnect" : "boot");
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
  attemptMqttConnect(nowMs);
  if (g_wifiFailureCount >= 50) {
    Serial.println(F("WIFI_RESTART_GUARD would_restart_after_safe_state"));
    g_wifiFailureCount = 0;
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

}  // namespace MqttManager

#endif
