#include "auto/BridgeBallsCommand.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"

BridgeBallsCommand::BridgeBallsCommand(Intake* intake, Shooter* shooter, double timeout) {
  SetTimeout(timeout);
  intake_ = intake;
  shooter_ = shooter;
  state_ = 0;
  initTime_ = 0;
}

void BridgeBallsCommand::Initialize() {
  state_ = 0; 
  initTime_ = timer_->Get();
  AutoCommand::Initialize();
}

bool BridgeBallsCommand::Run(){
  switch(state_)
  {
    case 0:
      intake_->SetIntakePosition(Intake::INTAKE_DOWN);
      state_ = 1;
      break;
    case 1:
      intake_->SetIntakePower(1);
      if (timer_->Get() - initTime_ > 1.5) {
        intake_->SetIntakePosition(Intake::INTAKE_FLOATING);
        state_ = 2;
      }
      break;
    case 2:
      intake_->SetIntakePower(1);
      if (timer_->Get() - initTime_ > 3.5) {
        intake_->SetIntakePosition(Intake::INTAKE_UP);
        state_ = 3;
        intake_->SetIntakePower(0);
        return true;
      }
      break;
    default:
      printf("WTF am I doing in here");
  }
  return TimeoutExpired();
}

BridgeBallsCommand ::~BridgeBallsCommand() {
}
