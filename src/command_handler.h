#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

namespace CommandHandler {

void handle(char *cmd);
void executeForTest(const char *cmd);
bool runParserSelfTest();

}  // namespace CommandHandler

#endif
