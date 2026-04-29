#ifndef LOG_CONFIG_H
#define LOG_CONFIG_H

#include <Arduino.h>

enum class LogLevel : uint8_t {
  Debug = 0,
  Info,
  Warn,
  Error,
  Silent
};

#ifndef APP_LOG_LEVEL_DEFAULT
#define APP_LOG_LEVEL_DEFAULT LogLevel::Debug
#endif

inline LogLevel &appLogLevel() {
  static LogLevel level = APP_LOG_LEVEL_DEFAULT;
  return level;
}

inline void setAppLogLevel(const LogLevel level) {
  appLogLevel() = level;
}

inline bool shouldLog(const LogLevel level) {
  return static_cast<uint8_t>(level) >= static_cast<uint8_t>(appLogLevel()) &&
         appLogLevel() != LogLevel::Silent;
}

#define LOG_PRINT(level, value) \
  do { \
    if (shouldLog(level)) { \
      Serial.print(value); \
    } \
  } while (0)

#define LOG_PRINTLN(level, value) \
  do { \
    if (shouldLog(level)) { \
      Serial.println(value); \
    } \
  } while (0)

#define LOG_D(value) LOG_PRINTLN(LogLevel::Debug, value)
#define LOG_I(value) LOG_PRINTLN(LogLevel::Info, value)
#define LOG_W(value) LOG_PRINTLN(LogLevel::Warn, value)
#define LOG_E(value) LOG_PRINTLN(LogLevel::Error, value)

#endif
