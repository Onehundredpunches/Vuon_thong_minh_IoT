#ifndef NVS_STORE_H
#define NVS_STORE_H

#include <Arduino.h>

#include "actuator_types.h"
#include "system_mode.h"

namespace NvsStore {

static constexpr uint8_t SCHEMA_VERSION = 1;

struct PersistedState {
  uint8_t schemaVersion;
  uint8_t mode;          // SystemMode as uint8_t
  uint8_t pumpOn;
  uint8_t fanOn;
  uint8_t lightOn;
  int16_t roofAngle;
  float soilOnPct;
  float soilOffPct;
  float tempOnC;
  float tempOffC;
  float lightOnLux;
  float lightOffLux;
  float lastSoilPct;
  float lastTempC;
  uint32_t updateTs;
  uint32_t crc32;
};

void begin();
bool load(PersistedState &state);
void save(const PersistedState &state);
void markDirty();
void savePending();
void tickCoalescedWrite(uint32_t nowMs);

PersistedState buildCurrentState(SystemMode mode, bool pumpOn, bool fanOn,
                                  bool lightOn, int roofAngle,
                                  float lastSoilPct, float lastTempC);

bool runSelfTest();

}  // namespace NvsStore

#endif
