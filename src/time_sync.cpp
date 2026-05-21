#include "time_sync.h"

#include "config.h"

#if APP_MODE_SIMULATOR

namespace TimeSync {

static uint32_t g_seq = 0;

void begin() { Serial.println(F("TIME_SYNC: SIM_DISABLED")); }
bool isSynced() { return false; }
uint32_t epochUtc() { return 0; }
void formatIso8601(char *buf, size_t bufSize) { strncpy(buf, "UNSYNC", bufSize - 1); }
uint32_t uptimeMs() { return millis(); }
uint32_t nextSeq() { return ++g_seq; }

void writeMetaJson(char *buf, const size_t bufSize, const char *source) {
  snprintf(buf, bufSize,
           "\"meta\":{\"ts_epoch_utc\":0,\"ts_iso8601_utc\":\"UNSYNC\","
           "\"tz\":\"ICT-7\",\"uptime_ms\":%lu,\"seq\":%lu,\"source\":\"%s\",\"fw_ver\":\"2.1.0\"}",
           static_cast<unsigned long>(millis()), static_cast<unsigned long>(++g_seq), source);
}

bool runSelfTest() {
  Serial.println(F("TIME_SYNC_SELF_TEST: PASS"));
  return true;
}

}  // namespace TimeSync

#else

#include <WiFi.h>
#include <time.h>

namespace TimeSync {
namespace {

static constexpr long GMT_OFFSET_SEC = 7 * 3600;
static constexpr int DAYLIGHT_OFFSET_SEC = 0;
static constexpr const char *NTP_SERVER1 = "pool.ntp.org";
static constexpr const char *NTP_SERVER2 = "time.nist.gov";

bool g_synced = false;
uint32_t g_seq = 0;
uint32_t g_bootMs = 0;

}  // namespace

void begin() {
  g_bootMs = millis();
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER1, NTP_SERVER2);
  Serial.println(F("TIME_SYNC: NTP configured ICT-7"));
}

bool isSynced() {
  if (g_synced) {
    return true;
  }
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 0)) {
    if (timeinfo.tm_year > 100) {  // year > 2000
      g_synced = true;
      Serial.println(F("TIME_SYNC: NTP synced"));
    }
  }
  return g_synced;
}

uint32_t epochUtc() {
  if (!isSynced()) {
    return 0;
  }
  time_t now;
  time(&now);
  if (now < 1000000000) {
    return 0;
  }
  return static_cast<uint32_t>(now);
}

void formatIso8601(char *buf, const size_t bufSize) {
  if (!isSynced()) {
    strncpy(buf, "UNSYNC", bufSize - 1);
    buf[bufSize - 1] = '\0';
    return;
  }
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 0)) {
    strncpy(buf, "UNSYNC", bufSize - 1);
    buf[bufSize - 1] = '\0';
    return;
  }
  snprintf(buf, bufSize, "%04d-%02d-%02d %02d:%02d:%02d",
           timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
           timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
}

uint32_t uptimeMs() {
  return millis();
}

uint32_t nextSeq() {
  return ++g_seq;
}

void writeMetaJson(char *buf, const size_t bufSize, const char *source) {
  char iso[26] = {0};
  formatIso8601(iso, sizeof(iso));
  const uint32_t epoch = epochUtc();
  snprintf(buf, bufSize,
           "\"meta\":{\"ts_epoch_utc\":%lu,\"ts_iso8601_utc\":\"%s\","
           "\"tz\":\"ICT-7\",\"uptime_ms\":%lu,\"seq\":%lu,\"source\":\"%s\",\"fw_ver\":\"2.1.0\"}",
           static_cast<unsigned long>(epoch), iso,
           static_cast<unsigned long>(millis()),
           static_cast<unsigned long>(nextSeq()), source);
}

bool runSelfTest() {
  bool pass = true;
  char iso[26] = {0};
  formatIso8601(iso, sizeof(iso));
  pass &= (strlen(iso) > 0);

  char meta[256] = {0};
  writeMetaJson(meta, sizeof(meta), "self_test");
  pass &= (strstr(meta, "\"meta\":{") != nullptr);
  pass &= (strstr(meta, "\"uptime_ms\":") != nullptr);
  pass &= (strstr(meta, "\"fw_ver\":\"2.1.0\"") != nullptr);

  Serial.print(F("TIME_SYNC_SELF_TEST: "));
  Serial.println(pass ? F("PASS") : F("FAIL"));
  return pass;
}

}  // namespace TimeSync

#endif
