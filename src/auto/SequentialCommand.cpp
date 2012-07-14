#include "auto/SequentialCommand.h"

#include <stdarg.h>

SequentialCommand::SequentialCommand(int dummy, ...) {
  va_list vl;
  va_start(vl, dummy);

  // Stops once it sees the NULL parameter that must follow the desired commands.
  AutoCommand* command = NULL;
  while (command = va_arg(vl, AutoCommand*)) {
    commands_.push_back(command);
  }
  va_end(vl);
  // Start at the first command
  commandIndex_ = 0;
}

SequentialCommand::~SequentialCommand() {
  for (std::vector<AutoCommand*>::const_iterator it = commands_.begin(); it < commands_.end(); ++it) {
    delete *it;
  }
}

void SequentialCommand::Initialize() {
  // Only initialize the first command
  //printf("command size: %d\n", commands_.size());
  if (commands_.size()>0) {
    commands_[0]->Initialize();
  }
}

bool SequentialCommand::Run() {
  // If all the commands are done, the SequentialCommand is done
  if (commandIndex_ == (int)commands_.size()) {
    return true;
  } else if (commands_[commandIndex_]->Run()) {
    commandIndex_++;
    if (commandIndex_ == (int)commands_.size()) {
      // No more commands, SequentialCommand is done
      return true;
    } else {
      // More commands to process, Initialize the next one
      commands_[commandIndex_]->Initialize();
    }
  }
  // The current command isn't finished yet
  return false;
}

void SequentialCommand::AddCommand(AutoCommand* command) {
  commands_.push_back(command);
}
