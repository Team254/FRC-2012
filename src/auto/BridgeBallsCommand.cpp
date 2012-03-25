#include "auto/BridgeBallsCommand.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"

BridgeBallsCommand::BridgeBallsCommand(Intake* intake, Shooter* shooter, 
                                       bool runIntake, double timeout) {
  SetTimeout(timeout);
  intake_ = intake;
  shooter_ = shooter;
  state_ = 0;
  initTime_ = 0;
  runIntake_ = runIntake;
}

void BridgeBallsCommand::Initialize() {
  state_ = 0; 
  initTime_ = timer_->Get();
  AutoCommand::Initialize();
  intake_->SetIntakePosition(Intake::INTAKE_DOWN);
}

bool BridgeBallsCommand::Run(){
  intake_->SetIntakePower(1);
  
  // Float intake eventually
  if (timer_->Get() > 2.2) {
	  intake_->SetIntakePosition(Intake::INTAKE_FLOATING);
  }
  
  // Return condition
  bool ret = TimeoutExpired();
  if (ret) {
	  intake_->SetIntakePower(0);
	  shooter_->SetTargetVelocity(48);
	  return ret;
  }
  return false;
}

BridgeBallsCommand ::~BridgeBallsCommand() {
}
