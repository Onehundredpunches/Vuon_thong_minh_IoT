#ifndef SYSTEM_MODE_H
#define SYSTEM_MODE_H

enum class SystemMode : uint8_t {
  Boot,
  Auto,
  Manual,
  SafeStop,
  Error
};

#endif
