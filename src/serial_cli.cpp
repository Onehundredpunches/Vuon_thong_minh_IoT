#include "serial_cli.h"

#include <ctype.h>
#include <string.h>

#include "log_config.h"

namespace SerialCLI {
namespace {

CommandHandler g_handler = nullptr;
char g_cmdBuf[48] = {0};
size_t g_cmdLen = 0;

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

bool parseLogLevel(const char *value, LogLevel &level) {
  if (strcmp(value, "debug") == 0) {
    level = LogLevel::Debug;
    return true;
  }
  if (strcmp(value, "info") == 0) {
    level = LogLevel::Info;
    return true;
  }
  if (strcmp(value, "warn") == 0) {
    level = LogLevel::Warn;
    return true;
  }
  if (strcmp(value, "error") == 0) {
    level = LogLevel::Error;
    return true;
  }
  return false;
}

bool handleLogCommand(char *cmd) {
  if (strcmp(cmd, "log") == 0) {
    Serial.print(F("LOG_LEVEL: "));
    Serial.println(logLevelName(appLogLevel()));
    return true;
  }

  if (strncmp(cmd, "log ", 4) != 0) {
    return false;
  }

  LogLevel level = LogLevel::Debug;
  if (!parseLogLevel(cmd + 4, level)) {
    Serial.println(F("REJECT log | invalid_log_level"));
    return true;
  }
  setAppLogLevel(level);
  Serial.print(F("ACK log "));
  Serial.print(logLevelName(level));
  Serial.print(F(" | LOG_LEVEL="));
  Serial.println(logLevelName(level));
  return true;
}

void dispatchCommand(char *cmd) {
  normalize(cmd);
  if (cmd[0] == '\0') {
    return;
  }

  if (strcmp(cmd, "help") == 0) {
    printHelp();
    return;
  }

  if (handleLogCommand(cmd)) {
    return;
  }

  if (g_handler != nullptr) {
    g_handler(cmd);
  }
}

}  // namespace

void begin(const CommandHandler handler) {
  g_handler = handler;
  g_cmdLen = 0;
  g_cmdBuf[0] = '\0';
}

void tick() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      g_cmdBuf[g_cmdLen] = '\0';
      if (g_cmdLen > 0) {
        dispatchCommand(g_cmdBuf);
      }
      g_cmdLen = 0;
      g_cmdBuf[0] = '\0';
      continue;
    }

    if (g_cmdLen < (sizeof(g_cmdBuf) - 1)) {
      g_cmdBuf[g_cmdLen++] = c;
      g_cmdBuf[g_cmdLen] = '\0';
    }
  }
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  help"));
  Serial.println(F("  log debug"));
  Serial.println(F("  log info"));
  Serial.println(F("  log warn"));
  Serial.println(F("  log error"));
  Serial.println(F("  servo on"));
  Serial.println(F("  servo off"));
  Serial.println(F("  servo sweep on"));
  Serial.println(F("  servo sweep off"));
  Serial.println(F("  servo stop"));
  Serial.println(F("  servo 0"));
  Serial.println(F("  servo 90"));
  Serial.println(F("  servo 180"));
  Serial.println(F("  light on"));
  Serial.println(F("  light off"));
  Serial.println(F("  light toggle"));
  Serial.println(F("  light blink"));
  Serial.println(F("  fan on"));
  Serial.println(F("  fan off"));
  Serial.println(F("  pump on"));
  Serial.println(F("  pump off"));
}

}  // namespace SerialCLI
