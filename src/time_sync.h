#ifndef TIME_SYNC_H
#define TIME_SYNC_H

#include <Arduino.h>

namespace TimeSync {

void begin();
bool isSynced();
uint32_t epochUtc();
void formatIso8601(char *buf, size_t bufSize);
uint32_t uptimeMs();
uint32_t nextSeq();

// Writes meta JSON fragment: "meta":{...} (no trailing comma)
void writeMetaJson(char *buf, size_t bufSize, const char *source);

bool runSelfTest();

}  // namespace TimeSync

#endif
