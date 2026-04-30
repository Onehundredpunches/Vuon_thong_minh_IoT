#include "command_handler.h"

#include <Arduino.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "actuator_manager.h"
#include "config.h"
#include "mode_controller.h"

namespace CommandHandler {
namespace {

enum class CommandKind : uint8_t {
  ServoOff,
  ServoOn,
  ServoSweepOn,
  ServoSweepOff,
  ServoAngle,
  LightOn,
  LightOff,
  LightToggle,
  LightBlink,
  FanOn,
  FanOff,
  PumpOn,
  PumpOff,
  SetModeAuto,
  SetModeManual,
  Unknown
};

struct ParsedCommand {
  CommandKind kind;
  int servoAngle;
};

void normalize(char *cmd) {
  size_t writeIdx = 0;
  bool prevSpace = true;
  for (size_t i = 0; cmd[i] != '\0'; ++i) {
    char c = cmd[i];
    if (c == '\t') {
      c = ' ';
    }
    c = static_cast<char>(tolower(static_cast<unsigned char>(c)));

    if (c == ' ') {
      if (!prevSpace) {
        cmd[writeIdx++] = ' ';
      }
      prevSpace = true;
      continue;
    }

    cmd[writeIdx++] = c;
    prevSpace = false;
  }
  if (writeIdx > 0 && cmd[writeIdx - 1] == ' ') {
    --writeIdx;
  }
  cmd[writeIdx] = '\0';
}

ParsedCommand parseCommand(const char *cmd) {
  if (strcmp(cmd, "servo off") == 0) {
    return {CommandKind::ServoOff, -1};
  }
  if (strcmp(cmd, "servo on") == 0) {
    return {CommandKind::ServoOn, -1};
  }
  if (strcmp(cmd, "servo sweep on") == 0) {
    return {CommandKind::ServoSweepOn, -1};
  }
  if (strcmp(cmd, "servo sweep off") == 0 || strcmp(cmd, "servo stop") == 0) {
    return {CommandKind::ServoSweepOff, -1};
  }

  int targetAngle = -1;
  if (sscanf(cmd, "servo %d", &targetAngle) == 1 &&
      (targetAngle == 0 || targetAngle == 90 || targetAngle == 180)) {
    return {CommandKind::ServoAngle, targetAngle};
  }

  if (strcmp(cmd, "light on") == 0) {
    return {CommandKind::LightOn, -1};
  }
  if (strcmp(cmd, "light off") == 0) {
    return {CommandKind::LightOff, -1};
  }
  if (strcmp(cmd, "light toggle") == 0) {
    return {CommandKind::LightToggle, -1};
  }
  if (strcmp(cmd, "light blink") == 0) {
    return {CommandKind::LightBlink, -1};
  }
  if (strcmp(cmd, "fan on") == 0) {
    return {CommandKind::FanOn, -1};
  }
  if (strcmp(cmd, "fan off") == 0) {
    return {CommandKind::FanOff, -1};
  }
  if (strcmp(cmd, "pump on") == 0) {
    return {CommandKind::PumpOn, -1};
  }
  if (strcmp(cmd, "pump off") == 0) {
    return {CommandKind::PumpOff, -1};
  }
  if (strcmp(cmd, "mode auto") == 0 || strcmp(cmd, "set_mode auto") == 0) {
    return {CommandKind::SetModeAuto, -1};
  }
  if (strcmp(cmd, "mode manual") == 0 || strcmp(cmd, "set_mode manual") == 0) {
    return {CommandKind::SetModeManual, -1};
  }

  return {CommandKind::Unknown, -1};
}

bool expectKind(const char *input, const CommandKind expectedKind, const int expectedAngle = -1) {
  char local[48] = {0};
  strncpy(local, input, sizeof(local) - 1);
  normalize(local);
  const ParsedCommand parsed = parseCommand(local);
  return parsed.kind == expectedKind && parsed.servoAngle == expectedAngle;
}

}  // namespace

bool isActuatorCommand(const CommandKind kind) {
  return kind != CommandKind::SetModeAuto && kind != CommandKind::SetModeManual &&
         kind != CommandKind::Unknown;
}

void printModeAck(const char *cmd) {
  Serial.print(F("ACK "));
  Serial.print(cmd);
  Serial.print(F(" | MODE="));
  Serial.println(ModeController::modeName());
}

void printModeReject(const char *cmd, const char *reason) {
  Serial.print(F("REJECT "));
  Serial.print(cmd);
  Serial.print(F(" | "));
  Serial.println(reason);
}

void handleInternal(char *cmd, const bool bypassModeGate) {
  normalize(cmd);
  if (cmd[0] == '\0') {
    return;
  }

  const ParsedCommand parsed = parseCommand(cmd);
  if (parsed.kind == CommandKind::SetModeManual) {
    if (!ModeController::setManual("serial", millis())) {
      printModeReject(cmd, "invalid_transition");
      return;
    }
    printModeAck(cmd);
    return;
  }
  if (parsed.kind == CommandKind::SetModeAuto) {
    if (!ModeController::setAuto("serial", millis())) {
      printModeReject(cmd, "invalid_transition");
      return;
    }
    printModeAck(cmd);
    return;
  }

  if (!bypassModeGate && isActuatorCommand(parsed.kind)) {
    const char *rejectReason = nullptr;
    if (!ModeController::actuatorCommandAllowed(&rejectReason)) {
      printModeReject(cmd, rejectReason);
      return;
    }
    ModeController::noteManualActivity(millis());
  }

  switch (parsed.kind) {
    case CommandKind::ServoOff:
      ActuatorManager::setServoOnOff(false);
      ActuatorManager::printServoAck(cmd);
      return;
    case CommandKind::ServoOn:
      ActuatorManager::setServoOnOff(true);
      ActuatorManager::printServoAck(cmd);
      return;
    case CommandKind::ServoSweepOn: {
      const ActuatorCommandStatus status = ActuatorManager::requestServoSweep(true, millis());
      if (status != ActuatorCommandStatus::Ok) {
        ActuatorManager::printCommandReject(cmd, status);
        return;
      }
      ActuatorManager::printServoAck(cmd);
      return;
    }
    case CommandKind::ServoSweepOff: {
      const ActuatorCommandStatus status = ActuatorManager::requestServoSweep(false, millis());
      if (status != ActuatorCommandStatus::Ok) {
        ActuatorManager::printCommandReject(cmd, status);
        return;
      }
      ActuatorManager::printServoAck(cmd);
      return;
    }
    case CommandKind::ServoAngle: {
      const ActuatorCommandStatus status = ActuatorManager::requestServoAngleCommand(parsed.servoAngle, millis());
      if (status != ActuatorCommandStatus::Ok) {
        ActuatorManager::printCommandReject(cmd, status);
        return;
      }
      ActuatorManager::printServoAck(cmd);
      return;
    }
    case CommandKind::LightOn: {
      const ActuatorCommandStatus status = ActuatorManager::requestLight(true, millis());
      if (status != ActuatorCommandStatus::Ok) {
        ActuatorManager::printCommandReject(cmd, status);
        return;
      }
      ActuatorManager::printRelayCommandLog(cmd, "LIGHT", PIN_RELAY_LIGHT, true);
      ActuatorManager::printLightState();
      return;
    }
    case CommandKind::LightOff: {
      const ActuatorCommandStatus status = ActuatorManager::requestLight(false, millis());
      if (status != ActuatorCommandStatus::Ok) {
        ActuatorManager::printCommandReject(cmd, status);
        return;
      }
      ActuatorManager::printRelayCommandLog(cmd, "LIGHT", PIN_RELAY_LIGHT, false);
      ActuatorManager::printLightState();
      return;
    }
    case CommandKind::LightToggle: {
      const bool targetOn = !ActuatorManager::lightOn();
      const ActuatorCommandStatus status = ActuatorManager::requestLight(targetOn, millis());
      if (status != ActuatorCommandStatus::Ok) {
        ActuatorManager::printCommandReject(cmd, status);
        return;
      }
      ActuatorManager::printRelayCommandLog(cmd, "LIGHT", PIN_RELAY_LIGHT, targetOn);
      ActuatorManager::printLightState();
      return;
    }
    case CommandKind::LightBlink: {
      const ActuatorCommandStatus status = ActuatorManager::requestLightBlink(millis());
      if (status != ActuatorCommandStatus::Ok) {
        ActuatorManager::printCommandReject(cmd, status);
        return;
      }
      ActuatorManager::printRelayCommandLog(cmd, "LIGHT", PIN_RELAY_LIGHT, true);
      ActuatorManager::printLightState();
      return;
    }
    case CommandKind::FanOn: {
      const ActuatorCommandStatus status = ActuatorManager::requestFan(true, millis());
      if (status != ActuatorCommandStatus::Ok) {
        ActuatorManager::printCommandReject(cmd, status);
        return;
      }
      ActuatorManager::printRelayCommandLog(cmd, "FAN", PIN_RELAY_FAN, true);
      ActuatorManager::printFanState();
      return;
    }
    case CommandKind::FanOff: {
      const ActuatorCommandStatus status = ActuatorManager::requestFan(false, millis());
      if (status != ActuatorCommandStatus::Ok) {
        ActuatorManager::printCommandReject(cmd, status);
        return;
      }
      ActuatorManager::printRelayCommandLog(cmd, "FAN", PIN_RELAY_FAN, false);
      ActuatorManager::printFanState();
      return;
    }
    case CommandKind::PumpOn: {
      const ActuatorCommandStatus status = ActuatorManager::requestPump(true, millis());
      if (status != ActuatorCommandStatus::Ok) {
        ActuatorManager::printCommandReject(cmd, status);
        return;
      }
      ActuatorManager::printRelayCommandLog(cmd, "PUMP", PIN_RELAY_PUMP, true);
      ActuatorManager::printPumpState();
      return;
    }
    case CommandKind::PumpOff: {
      const ActuatorCommandStatus status = ActuatorManager::requestPump(false, millis());
      if (status != ActuatorCommandStatus::Ok) {
        ActuatorManager::printCommandReject(cmd, status);
        return;
      }
      ActuatorManager::printRelayCommandLog(cmd, "PUMP", PIN_RELAY_PUMP, false);
      ActuatorManager::printPumpState();
      return;
    }
    case CommandKind::Unknown:
      Serial.println(F("CMD: UNKNOWN"));
      return;
  }
}

ExecuteResult executeParsed(const ParsedCommand &parsed, const char *cmd, const bool bypassModeGate,
                            const uint32_t nowMs, const bool printSerial) {
  if (parsed.kind == CommandKind::Unknown) {
    if (printSerial) {
      Serial.println(F("CMD: UNKNOWN"));
    }
    return {ExecuteStatus::Rejected, "unknown_command"};
  }

  if (parsed.kind == CommandKind::SetModeManual) {
    if (!ModeController::setManual("command", nowMs)) {
      if (printSerial) {
        printModeReject(cmd, "invalid_transition");
      }
      return {ExecuteStatus::Rejected, "invalid_transition"};
    }
    if (printSerial) {
      printModeAck(cmd);
    }
    return {ExecuteStatus::Ok, "success"};
  }
  if (parsed.kind == CommandKind::SetModeAuto) {
    if (!ModeController::setAuto("command", nowMs)) {
      if (printSerial) {
        printModeReject(cmd, "invalid_transition");
      }
      return {ExecuteStatus::Rejected, "invalid_transition"};
    }
    if (printSerial) {
      printModeAck(cmd);
    }
    return {ExecuteStatus::Ok, "success"};
  }

  if (!bypassModeGate && isActuatorCommand(parsed.kind)) {
    const char *rejectReason = nullptr;
    if (!ModeController::actuatorCommandAllowed(&rejectReason)) {
      if (printSerial) {
        printModeReject(cmd, rejectReason);
      }
      return {ExecuteStatus::Rejected, rejectReason};
    }
    ModeController::noteManualActivity(nowMs);
  }

  ActuatorCommandStatus status = ActuatorCommandStatus::Ok;
  switch (parsed.kind) {
    case CommandKind::ServoOff:
      ActuatorManager::setServoOnOff(false);
      if (printSerial) {
        ActuatorManager::printServoAck(cmd);
      }
      return {ExecuteStatus::Ok, "success"};
    case CommandKind::ServoOn:
      ActuatorManager::setServoOnOff(true);
      if (printSerial) {
        ActuatorManager::printServoAck(cmd);
      }
      return {ExecuteStatus::Ok, "success"};
    case CommandKind::ServoSweepOn:
      status = ActuatorManager::requestServoSweep(true, nowMs);
      break;
    case CommandKind::ServoSweepOff:
      status = ActuatorManager::requestServoSweep(false, nowMs);
      break;
    case CommandKind::ServoAngle:
      status = ActuatorManager::requestServoAngleCommand(parsed.servoAngle, nowMs);
      break;
    case CommandKind::LightOn:
      status = ActuatorManager::requestLight(true, nowMs);
      break;
    case CommandKind::LightOff:
      status = ActuatorManager::requestLight(false, nowMs);
      break;
    case CommandKind::LightToggle:
      status = ActuatorManager::requestLight(!ActuatorManager::lightOn(), nowMs);
      break;
    case CommandKind::LightBlink:
      status = ActuatorManager::requestLightBlink(nowMs);
      break;
    case CommandKind::FanOn:
      status = ActuatorManager::requestFan(true, nowMs);
      break;
    case CommandKind::FanOff:
      status = ActuatorManager::requestFan(false, nowMs);
      break;
    case CommandKind::PumpOn:
      status = ActuatorManager::requestPump(true, nowMs);
      break;
    case CommandKind::PumpOff:
      status = ActuatorManager::requestPump(false, nowMs);
      break;
    case CommandKind::SetModeAuto:
    case CommandKind::SetModeManual:
    case CommandKind::Unknown:
      return {ExecuteStatus::Rejected, "unknown_command"};
  }

  if (status != ActuatorCommandStatus::Ok) {
    if (printSerial) {
      ActuatorManager::printCommandReject(cmd, status);
    }
    if (status == ActuatorCommandStatus::CooldownActive) {
      return {ExecuteStatus::Rejected, "cooldown_active"};
    }
    if (status == ActuatorCommandStatus::InterlockViolation) {
      return {ExecuteStatus::Rejected, "interlock_violation"};
    }
    return {ExecuteStatus::Error, "internal_error"};
  }

  if (printSerial) {
    switch (parsed.kind) {
      case CommandKind::ServoSweepOn:
      case CommandKind::ServoSweepOff:
      case CommandKind::ServoAngle:
        ActuatorManager::printServoAck(cmd);
        break;
      case CommandKind::LightOn:
      case CommandKind::LightOff:
      case CommandKind::LightToggle:
      case CommandKind::LightBlink:
        ActuatorManager::printRelayCommandLog(cmd, "LIGHT", PIN_RELAY_LIGHT, ActuatorManager::lightOn());
        ActuatorManager::printLightState();
        break;
      case CommandKind::FanOn:
      case CommandKind::FanOff:
        ActuatorManager::printRelayCommandLog(cmd, "FAN", PIN_RELAY_FAN, ActuatorManager::fanOn());
        ActuatorManager::printFanState();
        break;
      case CommandKind::PumpOn:
      case CommandKind::PumpOff:
        ActuatorManager::printRelayCommandLog(cmd, "PUMP", PIN_RELAY_PUMP, ActuatorManager::pumpOn());
        ActuatorManager::printPumpState();
        break;
      default:
        break;
    }
  }
  return {ExecuteStatus::Ok, "success"};
}

void handle(char *cmd) {
  handleInternal(cmd, false);
}

void executeForTest(const char *cmd) {
  char local[48] = {0};
  strncpy(local, cmd, sizeof(local) - 1);
  handleInternal(local, true);
}

ExecuteResult executeStructured(const char *cmd, const bool bypassModeGate, const unsigned long nowMs) {
  char local[48] = {0};
  strncpy(local, cmd, sizeof(local) - 1);
  normalize(local);
  if (local[0] == '\0') {
    return {ExecuteStatus::Rejected, "unknown_command"};
  }
  const ParsedCommand parsed = parseCommand(local);
  return executeParsed(parsed, local, bypassModeGate, static_cast<uint32_t>(nowMs), false);
}

bool runParserSelfTest() {
  bool pass = true;
  pass &= expectKind("  LIGHT   ON  ", CommandKind::LightOn);
  pass &= expectKind("Fan\tOff", CommandKind::FanOff);
  pass &= expectKind("pump on", CommandKind::PumpOn);
  pass &= expectKind("servo 90", CommandKind::ServoAngle, 90);
  pass &= expectKind("servo stop", CommandKind::ServoSweepOff);
  pass &= expectKind("mode manual", CommandKind::SetModeManual);
  pass &= expectKind("set_mode auto", CommandKind::SetModeAuto);
  pass &= expectKind("servo 45", CommandKind::Unknown);

  Serial.print(F("COMMAND_HANDLER_SELF_TEST: "));
  Serial.println(pass ? F("PASS") : F("FAIL"));
  return pass;
}

}  // namespace CommandHandler
