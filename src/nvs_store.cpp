#include "nvs_store.h"

#include <Preferences.h>

#include "config.h"

namespace NvsStore {
namespace {

static constexpr const char *kNamespace = "vuon-iot";
static constexpr uint32_t COALESCE_INTERVAL_MS = 30000;

Preferences g_prefs;
bool g_dirty = false;
uint32_t g_lastWriteMs = 0;
PersistedState g_pending{};

uint32_t computeCrc32(const PersistedState &s) {
  const uint8_t *data = reinterpret_cast<const uint8_t *>(&s);
  const size_t len = offsetof(PersistedState, crc32);
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ (0xEDB88320 & (-(crc & 1)));
    }
  }
  return ~crc;
}

PersistedState defaultState() {
  PersistedState s{};
  s.schemaVersion = SCHEMA_VERSION;
  s.mode = static_cast<uint8_t>(SystemMode::Auto);
  s.pumpOn = 0;
  s.fanOn = 0;
  s.lightOn = 0;
  s.roofAngle = AUTO_ROOF_SAFE_ANGLE;
  s.soilOnPct = AUTO_SOIL_PUMP_ON_PCT;
  s.soilOffPct = AUTO_SOIL_PUMP_OFF_PCT;
  s.tempOnC = AUTO_TEMP_FAN_ON_C;
  s.tempOffC = AUTO_TEMP_FAN_OFF_C;
  s.lightOnLux = AUTO_LIGHT_ON_LUX;
  s.lightOffLux = AUTO_LIGHT_OFF_LUX;
  s.lastSoilPct = 50.0f;
  s.lastTempC = 25.0f;
  s.updateTs = 0;
  s.crc32 = computeCrc32(s);
  return s;
}

}  // namespace

void begin() {
  g_prefs.begin(kNamespace, false);
  g_dirty = false;
  g_lastWriteMs = millis();
  Serial.println(F("NVS_STORE: init"));
}

bool load(PersistedState &state) {
  if (!g_prefs.isKey("cfg_ver")) {
    Serial.println(F("NVS_LOAD: no_data using_defaults"));
    state = defaultState();
    save(state);
    return false;
  }
  state.schemaVersion = g_prefs.getUChar("cfg_ver", 0);
  if (state.schemaVersion != SCHEMA_VERSION) {
    Serial.print(F("NVS_LOAD: schema_mismatch got="));
    Serial.print(state.schemaVersion);
    Serial.println(F(" using_defaults"));
    state = defaultState();
    save(state);
    return false;
  }
  state.mode = g_prefs.getUChar("mode", static_cast<uint8_t>(SystemMode::Auto));
  state.pumpOn = g_prefs.getUChar("pump_on", 0);
  state.fanOn = g_prefs.getUChar("fan_on", 0);
  state.lightOn = g_prefs.getUChar("light_on", 0);
  state.roofAngle = static_cast<int16_t>(g_prefs.getShort("roof_angle", AUTO_ROOF_SAFE_ANGLE));
  state.soilOnPct = g_prefs.getFloat("soil_on_pct", AUTO_SOIL_PUMP_ON_PCT);
  state.soilOffPct = g_prefs.getFloat("soil_off_pct", AUTO_SOIL_PUMP_OFF_PCT);
  state.tempOnC = g_prefs.getFloat("temp_on_c", AUTO_TEMP_FAN_ON_C);
  state.tempOffC = g_prefs.getFloat("temp_off_c", AUTO_TEMP_FAN_OFF_C);
  state.lightOnLux = g_prefs.getFloat("light_on_lux", AUTO_LIGHT_ON_LUX);
  state.lightOffLux = g_prefs.getFloat("light_off_lux", AUTO_LIGHT_OFF_LUX);
  state.lastSoilPct = g_prefs.getFloat("last_soil_pct", 50.0f);
  state.lastTempC = g_prefs.getFloat("last_temp_c", 25.0f);
  state.updateTs = g_prefs.getULong("upd_ts", 0);
  state.crc32 = g_prefs.getULong("crc32", 0);

  const uint32_t expected = computeCrc32(state);
  if (state.crc32 != expected) {
    Serial.print(F("NVS_LOAD: crc_mismatch stored=0x"));
    Serial.print(state.crc32, HEX);
    Serial.print(F(" computed=0x"));
    Serial.print(expected, HEX);
    Serial.println(F(" using_defaults"));
    state = defaultState();
    save(state);
    return false;
  }
  Serial.println(F("NVS_LOAD: ok"));
  return true;
}

void save(const PersistedState &state) {
  PersistedState s = state;
  s.crc32 = computeCrc32(s);
  g_prefs.putUChar("cfg_ver", s.schemaVersion);
  g_prefs.putUChar("mode", s.mode);
  g_prefs.putUChar("pump_on", s.pumpOn);
  g_prefs.putUChar("fan_on", s.fanOn);
  g_prefs.putUChar("light_on", s.lightOn);
  g_prefs.putShort("roof_angle", s.roofAngle);
  g_prefs.putFloat("soil_on_pct", s.soilOnPct);
  g_prefs.putFloat("soil_off_pct", s.soilOffPct);
  g_prefs.putFloat("temp_on_c", s.tempOnC);
  g_prefs.putFloat("temp_off_c", s.tempOffC);
  g_prefs.putFloat("light_on_lux", s.lightOnLux);
  g_prefs.putFloat("light_off_lux", s.lightOffLux);
  g_prefs.putFloat("last_soil_pct", s.lastSoilPct);
  g_prefs.putFloat("last_temp_c", s.lastTempC);
  g_prefs.putULong("upd_ts", s.updateTs);
  g_prefs.putULong("crc32", s.crc32);
  g_dirty = false;
  g_lastWriteMs = millis();
  Serial.println(F("NVS_SAVE: ok"));
}

void markDirty() {
  g_dirty = true;
}

void savePending() {
  if (g_dirty) {
    save(g_pending);
  }
}

void tickCoalescedWrite(const uint32_t nowMs) {
  if (!g_dirty) {
    return;
  }
  if (static_cast<uint32_t>(nowMs - g_lastWriteMs) < COALESCE_INTERVAL_MS) {
    return;
  }
  save(g_pending);
}

PersistedState buildCurrentState(const SystemMode mode, const bool pumpOn, const bool fanOn,
                                  const bool lightOn, const int roofAngle,
                                  const float lastSoilPct, const float lastTempC) {
  PersistedState s{};
  s.schemaVersion = SCHEMA_VERSION;
  s.mode = static_cast<uint8_t>(mode);
  s.pumpOn = pumpOn ? 1 : 0;
  s.fanOn = fanOn ? 1 : 0;
  s.lightOn = lightOn ? 1 : 0;
  s.roofAngle = static_cast<int16_t>(roofAngle);
  s.soilOnPct = AUTO_SOIL_PUMP_ON_PCT;
  s.soilOffPct = AUTO_SOIL_PUMP_OFF_PCT;
  s.tempOnC = AUTO_TEMP_FAN_ON_C;
  s.tempOffC = AUTO_TEMP_FAN_OFF_C;
  s.lightOnLux = AUTO_LIGHT_ON_LUX;
  s.lightOffLux = AUTO_LIGHT_OFF_LUX;
  s.lastSoilPct = lastSoilPct;
  s.lastTempC = lastTempC;
  s.updateTs = millis();
  g_pending = s;
  return s;
}

bool runSelfTest() {
  bool pass = true;

  // Test CRC
  PersistedState s = defaultState();
  const uint32_t crc1 = computeCrc32(s);
  s.crc32 = crc1;
  pass &= (computeCrc32(s) == crc1); // CRC should be stable

  // Test corruption detection
  PersistedState corrupted = s;
  corrupted.pumpOn = 1; // Modify without recalculating CRC
  pass &= (computeCrc32(corrupted) != corrupted.crc32);

  Serial.print(F("NVS_STORE_SELF_TEST: "));
  Serial.println(pass ? F("PASS") : F("FAIL"));
  return pass;
}

}  // namespace NvsStore
