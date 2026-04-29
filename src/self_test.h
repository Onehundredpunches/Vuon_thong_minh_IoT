#ifndef SELF_TEST_H
#define SELF_TEST_H

namespace SelfTest {

using CommandExecutor = void (*)(const char *cmd);

bool run(CommandExecutor executeCommand);

}  // namespace SelfTest

#endif
