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

  if (runIntake_) {
	  intake_->SetIntakePower(1);
	  shooter_->SetLinearConveyorPower(-1);
  } else {
	  intake_->SetIntakePower(0);
	  shooter_->SetLinearConveyorPower(0);
  }


  // Float intake eventually
  if (timer_->Get() > .9) {
    intake_->SetIntakePosition(Intake::INTAKE_FLOATING);
  } else if (timer_->Get() > .5) {
    intake_->SetIntakePosition(Intake::INTAKE_DOWN);
  } else {
	intake_->SetIntakePosition(Intake::INTAKE_UP);
  }

  // Return condition
  bool ret = TimeoutExpired();
  if (ret) {
    return ret;
  }
  return false;
}

BridgeBallsCommand ::~BridgeBallsCommand() {
}
