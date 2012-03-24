#include "auto/AutoAlignCommand.h"
#include "subsystems/Drive.h"
#include "drivers/AutoTurnDriver.h"

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
  if (timer_->Get() < .25)
	  return false;
  bool ret = autoTurn_->UpdateDriver() || TimeoutExpired();
  if (ret){
	  drive_->SetLinearPower(0,0);
  }
  return ret;
}

AutoAlignCommand::~AutoAlignCommand() {
}
