#include "SequentialCommand.h"

SequentialCommand::SequentialCommand(int numCommands, ...) {
  va_list(vl);
  va_start(vl,numCommands);
  for(int i = 0; i < numCommands; i++) {
    AutoCommand* command = va_arg(vl,AutoCommand*);
    commands_.push_back(command);
  }
  va_end(vl);
  commandIndex_=0;
}

SequentialCommand::~SequentialCommand() {
  std::vector<AutoCommand*>::iterator it;
  for(it = commands_.begin(); it < commands_.end(); it++) {
    delete *it;
    *it=NULL;
  }
}

void SequentialCommand::Initialize() {
  commands_[0]->Initialize();
}

bool SequentialCommand::Run() {
  // if the current command is done
  if(commands_[commandIndex_]->Run()) {
    commandIndex_++;
    commands_[commandIndex_]->Initialize();
  }
  // return true if we have completed the last command
  return commandIndex_>=commands_.size();
}
