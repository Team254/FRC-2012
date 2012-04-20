#include "auto/AutoAlignCommand.h"
#include "subsystems/Drive.h"
#include "drivers/AutoTurnDriver.h"

AutoAlignCommand::AutoAlignCommand(Drive* drive, AutoTurnDriver* autoTurn, double offset, double timeout) {
  SetTimeout(timeout);
  autoTurn_ = autoTurn;
  drive_ = drive;
  offset_ = offset;
}

void AutoAlignCommand::Initialize() {
  AutoCommand::Initialize();
  autoTurn_->Reset();
  if (offset_ != 0)
    autoTurn_->SetOffsetAngle(offset_);
}

bool AutoAlignCommand::Run(){
  if (timer_->Get() < .25)
	  return false;
  bool ret = TimeoutExpired();
  autoTurn_->UpdateDriver();
  if (ret){
	  drive_->SetLinearPower(0,0);
  }
  return ret;
}

AutoAlignCommand::~AutoAlignCommand() {
}
