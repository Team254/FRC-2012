#include "auto/DelayCommand.h"

DelayCommand::DelayCommand(double timeout) {
  SetTimeout(timeout);
}

void DelayCommand::Initialize() {
  AutoCommand::Initialize();
}

bool DelayCommand::Run(){
  return TimeoutExpired();
}

DelayCommand::~DelayCommand() {
}
