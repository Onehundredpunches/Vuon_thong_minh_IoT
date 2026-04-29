#include "command_handler.h"

#include <Arduino.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "actuator_manager.h"
#include "config.h"

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

void handle(char *cmd) {
  normalize(cmd);
  if (cmd[0] == '\0') {
    return;
  }

  const ParsedCommand parsed = parseCommand(cmd);
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

void executeForTest(const char *cmd) {
  char local[48] = {0};
  strncpy(local, cmd, sizeof(local) - 1);
  handle(local);
}

bool runParserSelfTest() {
  bool pass = true;
  pass &= expectKind("  LIGHT   ON  ", CommandKind::LightOn);
  pass &= expectKind("Fan\tOff", CommandKind::FanOff);
  pass &= expectKind("pump on", CommandKind::PumpOn);
  pass &= expectKind("servo 90", CommandKind::ServoAngle, 90);
  pass &= expectKind("servo stop", CommandKind::ServoSweepOff);
  pass &= expectKind("servo 45", CommandKind::Unknown);

  Serial.print(F("COMMAND_HANDLER_SELF_TEST: "));
  Serial.println(pass ? F("PASS") : F("FAIL"));
  return pass;
}

}  // namespace CommandHandler
