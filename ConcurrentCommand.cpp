#include "ConcurrentCommand.h"

#include <stdarg.h>

ConcurrentCommand::ConcurrentCommand(int numCommands, ...)
    : commands_(numCommands) {
  va_list(vl);
  va_start(vl, numCommands);
  for (int i = 0; i < numCommands; i++) {
    AutoCommand* command = va_arg(vl, AutoCommand*);
    CommandPair cPair(command, false);
    commands_.push_back(cPair);
  }
  va_end(vl);
}

ConcurrentCommand::~ConcurrentCommand() {
  for (std::vector<CommandPair>::const_iterator it = commands_.begin(); it < commands_.end(); ++it) {
    CommandPair cPair = *it;
    delete cPair.first;
    cPair.first = NULL;
  }
}

void ConcurrentCommand::Initialize() {
  for (std::vector<CommandPair>::const_iterator it = commands_.begin(); it < commands_.end(); ++it) {
    CommandPair cPair = *it;
    cPair.first->Initialize();
  }
}

bool ConcurrentCommand::Run() {
  bool done = true;
  for (std::vector<CommandPair>::const_iterator it = commands_.begin(); it < commands_.end(); ++it) {
    CommandPair cPair = *it;
    // If the command isn't done yet, update its doneness status.
    if (!cPair.second) {
      cPair.second = cPair.first->Run();
      // If the command isn't done, done will be false.
      done &= cPair.second;
    }
  }

  return done;
}

