#include "mode_controller.h"

#include <string.h>

#include "actuator_manager.h"
#include "config.h"

namespace ModeController {
namespace {

static constexpr uint32_t MANUAL_TIMEOUT_MS = 900000;

SystemMode g_mode = SystemMode::Boot;
const char *g_reason = "boot";
uint32_t g_lastManualActivityMs = 0;

void publishModeLine() {
  Serial.print(F("MODE_STATE: "));
  Serial.print(modeName());
  Serial.print(F(" reason="));
  Serial.println(g_reason);
}

void setMode(const SystemMode next, const char *reason, const uint32_t nowMs) {
  g_mode = next;
  g_reason = reason;
  if (next == SystemMode::Manual) {
    g_lastManualActivityMs = nowMs;
  }
  publishModeLine();
}

bool transitionAllowed(const SystemMode from, const SystemMode to) {
  if (from == SystemMode::Error) {
    return false;
  }
  if (from == SystemMode::Boot) {
    return to == SystemMode::Auto || to == SystemMode::Error;
  }
  if (from == SystemMode::SafeStop) {
    return to == SystemMode::Auto || to == SystemMode::Error;
  }
  return to == SystemMode::Auto || to == SystemMode::Manual || to == SystemMode::SafeStop ||
         to == SystemMode::Error;
}

}  // namespace

void begin(const uint32_t nowMs) {
  g_mode = SystemMode::Boot;
  g_reason = "boot";
  g_lastManualActivityMs = nowMs;
}

SystemMode mode() {
  return g_mode;
}

const char *modeName() {
  switch (g_mode) {
    case SystemMode::Boot:
      return "BOOT";
    case SystemMode::Auto:
      return "AUTO";
    case SystemMode::Manual:
      return "MANUAL";
    case SystemMode::SafeStop:
      return "SAFE_STOP";
    case SystemMode::Error:
      return "ERROR";
  }
  return "ERROR";
}

const char *modeToken() {
  switch (g_mode) {
    case SystemMode::Boot:
      return "ER";
    case SystemMode::Auto:
      return "AU";
    case SystemMode::Manual:
      return "MN";
    case SystemMode::SafeStop:
      return "SS";
    case SystemMode::Error:
      return "ER";
  }
  return "ER";
}

const char *lastReason() {
  return g_reason;
}

uint32_t lastManualActivityMs() {
  return g_lastManualActivityMs;
}

bool setManual(const char *reason, const uint32_t nowMs) {
  if (!transitionAllowed(g_mode, SystemMode::Manual)) {
    return false;
  }
  setMode(SystemMode::Manual, reason, nowMs);
  return true;
}

bool setAuto(const char *reason, const uint32_t nowMs) {
  if (!transitionAllowed(g_mode, SystemMode::Auto)) {
    return false;
  }
  setMode(SystemMode::Auto, reason, nowMs);
  return true;
}

void enterSafeStop(const char *reason, const uint32_t nowMs) {
  (void)nowMs;
  if (!transitionAllowed(g_mode, SystemMode::SafeStop)) {
    return;
  }
  ActuatorManager::restoreSafeDefaults();
  setMode(SystemMode::SafeStop, reason, millis());
}

void enterError(const char *reason, const uint32_t nowMs) {
  (void)nowMs;
  ActuatorManager::restoreSafeDefaults();
  setMode(SystemMode::Error, reason, millis());
}

void noteManualActivity(const uint32_t nowMs) {
  if (g_mode == SystemMode::Manual) {
    g_lastManualActivityMs = nowMs;
  }
}

bool tick(const uint32_t nowMs) {
  if (g_mode != SystemMode::Manual || g_lastManualActivityMs == 0) {
    return false;
  }
  if (static_cast<uint32_t>(nowMs - g_lastManualActivityMs) < MANUAL_TIMEOUT_MS) {
    return false;
  }
  return setAuto("manual_timeout", nowMs);
}

bool actuatorCommandAllowed(const char **rejectReason) {
  if (g_mode == SystemMode::Manual) {
    return true;
  }
  if (g_mode == SystemMode::Auto) {
    *rejectReason = "invalid_mode";
    return false;
  }
  if (g_mode == SystemMode::Boot) {
    *rejectReason = "system_not_ready";
    return false;
  }
  if (g_mode == SystemMode::SafeStop) {
    *rejectReason = "sensor_fault_active";
    return false;
  }
  *rejectReason = "reboot_required";
  return false;
}

bool runSelfTest() {
  const SystemMode savedMode = g_mode;
  const char *savedReason = g_reason;
  const uint32_t savedManualActivity = g_lastManualActivityMs;
  bool pass = true;
  const char *rejectReason = nullptr;

  begin(1000);
  pass &= !setManual("boot_reject", 1000);
  pass &= !actuatorCommandAllowed(&rejectReason) && strcmp(rejectReason, "system_not_ready") == 0;
  pass &= setAuto("self_test_pass", 1100);
  pass &= !actuatorCommandAllowed(&rejectReason) && strcmp(rejectReason, "invalid_mode") == 0;
  pass &= setManual("test_manual", 1200);
  pass &= actuatorCommandAllowed(&rejectReason);
  enterSafeStop("test_fault", 1300);
  pass &= !actuatorCommandAllowed(&rejectReason) && strcmp(rejectReason, "sensor_fault_active") == 0;
  pass &= setAuto("test_recovery", 1400);
  enterError("test_error", 1500);
  pass &= !setAuto("error_reject", 1600);
  pass &= !actuatorCommandAllowed(&rejectReason) && strcmp(rejectReason, "reboot_required") == 0;

  g_mode = savedMode;
  g_reason = savedReason;
  g_lastManualActivityMs = savedManualActivity;

  Serial.print(F("MODE_FSM_SELF_TEST: "));
  Serial.println(pass ? F("PASS") : F("FAIL"));
  return pass;
}

}  // namespace ModeController
