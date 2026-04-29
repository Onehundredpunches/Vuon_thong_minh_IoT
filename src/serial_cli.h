#ifndef SERIAL_CLI_H
#define SERIAL_CLI_H

#include <Arduino.h>

namespace SerialCLI {

using CommandHandler = void (*)(char *cmd);

void begin(CommandHandler handler);
void tick();
void printHelp();

}  // namespace SerialCLI

#endif
