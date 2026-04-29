#ifndef MODE_CONTROLLER_H
#define MODE_CONTROLLER_H

#include <Arduino.h>

#include "system_mode.h"

namespace ModeController {

void begin(uint32_t nowMs);
SystemMode mode();
const char *modeName();
const char *modeToken();
const char *lastReason();
uint32_t lastManualActivityMs();

bool setManual(const char *reason, uint32_t nowMs);
bool setAuto(const char *reason, uint32_t nowMs);
void enterSafeStop(const char *reason, uint32_t nowMs);
void enterError(const char *reason, uint32_t nowMs);
void noteManualActivity(uint32_t nowMs);
bool tick(uint32_t nowMs);

bool actuatorCommandAllowed(const char **rejectReason);
bool runSelfTest();

}  // namespace ModeController

#endif
