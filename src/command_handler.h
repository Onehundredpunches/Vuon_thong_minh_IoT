#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

namespace CommandHandler {

enum class ExecuteStatus : unsigned char {
  Ok,
  Rejected,
  Error
};

struct ExecuteResult {
  ExecuteStatus status;
  const char *reason;
};

void handle(char *cmd);
void executeForTest(const char *cmd);
ExecuteResult executeStructured(const char *cmd, bool bypassModeGate, unsigned long nowMs);
bool runParserSelfTest();

}  // namespace CommandHandler

#endif
