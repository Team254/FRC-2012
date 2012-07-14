#include "auto/AutoAlignCommand.h"
#include "subsystems/Drive.h"
#include "drivers/AutoTurnDriver.h"

AutoAlignCommand::AutoAlignCommand(Drive* drive, AutoTurnDriver* autoTurn, BackboardFinder* target, double offset, double timeout, bool useSkew) {
  SetTimeout(timeout);
  autoTurn_ = autoTurn;
  drive_ = drive;
  offset_ = offset;
  useSkew_ = useSkew;
  target_ = target;
  resetTimer_ = new Timer();
}

void AutoAlignCommand::Initialize() {
  AutoCommand::Initialize();
  autoTurn_->Reset();
  if (offset_ != 0)
    autoTurn_->SetOffsetAngle(offset_);
  target_->SetUseSkew(useSkew_);
  resetTimer_->Start();
}

bool AutoAlignCommand::Run(){
  if (timer_->Get() < .25)
	  return false;

  if (resetTimer_->Get() > .5) {
	  autoTurn_->Reset();
	  resetTimer_->Reset();
  }

  bool ret = autoTurn_->UpdateDriver();
  if (TimeoutExpired()){
	  drive_->SetLinearPower(0,0);
  }
  return TimeoutExpired();
}

AutoAlignCommand::~AutoAlignCommand() {
}
