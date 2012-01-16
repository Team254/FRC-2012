#include "ConcurrentCommand.h"
#include <stdarg.h>

ConcurrentCommand::ConcurrentCommand(int numCommands, ...) {
  va_list(vl);
  va_start(vl,numCommands);
  for(int i = 0; i < numCommands; i++) {
    AutoCommand* command = va_arg(vl,AutoCommand*);
    commandPair cPair(command,false);
    commands_.push_back(cPair);
  }
  va_end(vl);
}

ConcurrentCommand::~ConcurrentCommand() {
  std::vector<commandPair>::iterator it;
  for(it = commands_.begin(); it < commands_.end(); it++) {
    commandPair cPair = *it;
    delete cPair.first;
    cPair.first=NULL;
  }
}

void ConcurrentCommand::Initialize() {
  std::vector<commandPair>::iterator it;
  for(it = commands_.begin(); it < commands_.end(); it++) {
    commandPair cPair = *it;
    cPair.first->Initialize();
  }

}

bool ConcurrentCommand::Run() {
  std::vector<commandPair>::iterator it;
  bool done=true;
  for(it = commands_.begin(); it < commands_.end(); it++) {
    commandPair cPair = *it;
    // if the command isn't done yet, update its doneness status
    if(!cPair.second) {
      cPair.second=cPair.first->Run();
    }
    // if the command isn't done, done will be false
    done&=cPair.second;
  }

  return done;

}

