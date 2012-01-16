#include "SequentialCommand.h"

SequentialCommand::SequentialCommand(int numCommands, ...)
    : commands_(numCommands) {
  va_list(vl);
  va_start(vl, numCommands);
  for (int i = 0; i < numCommands; i++) {
    AutoCommand* command = va_arg(vl, AutoCommand*);
    commands_.push_back(command);
  }
  va_end(vl);
  commandIndex_ = 0;
}

SequentialCommand::~SequentialCommand() {
  for (std::vector<AutoCommand*>::const_iterator it = commands_.begin(); it < commands_.end(); ++it) {
    delete *it;
  }
}

void SequentialCommand::Initialize() {
  commands_[0]->Initialize();
}

bool SequentialCommand::Run() {
  // If the current command is done
  if (commands_[commandIndex_]->Run()) {
    commandIndex_++;

    if(commandIndex_ == commands_.size()) {
      return true;
    } else {
      commands_[commandIndex_]->Initialize();
    }
  }

  return false;
}
