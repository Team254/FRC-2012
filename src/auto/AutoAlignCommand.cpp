#include "auto/AutoAlignCommand.h"
#include "subsystems/Drive.h"
#include "drivers/AutoTurnDriver.cpp"

AutoAlignCommand::AutoAlignCommand(Drive* drive, AutoTurnDriver* autoTurn, double timeout) {
  SetTimeout(timeout);
  autoTurn_ = autoTurn;
  drive_ = drive;
}

void AutoAlignCommand::Initialize() {
  AutoCommand::Initialize();
  autoTurn_->Reset();
}

bool AutoAlignCommand::Run(){
  return autoTurn_->UpdateDriver() || TimeoutExpired();
}

AutoAlignCommand::~AutoAlignCommand() {
}
