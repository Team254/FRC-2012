#include "auto/AutoCommand.h"

AutoCommand::AutoCommand() {
  timer_ = new Timer();
  timeout_ = -1;
}

void AutoCommand::Initialize() {
  timer_->Start(); 
}

void AutoCommand::SetTimeout(double timeout)
{
  timeout_ = timeout;
}

bool AutoCommand::TimeoutExpired() {
  if (timeout_ != -1) {
    return timer_->Get() > timeout_;
  }
  return false;
}

AutoCommand::~AutoCommand() {
  delete timer_;
}

