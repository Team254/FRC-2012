#include "auto/JumbleCommand.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"

JumbleCommand::JumbleCommand(Shooter* shooter, Intake* intake, double timeout) {
  SetTimeout(timeout);
  shooter_ = shooter;
  intake_ = intake;
}

void JumbleCommand::Initialize() {
  AutoCommand::Initialize();
}

bool JumbleCommand::Run(){
  intake_->SetIntakePower(1.0);
  shooter_->SetLinearConveyorPower(-1.0);
  bool ret = TimeoutExpired();
  if (ret){
	  //intake_->SetIntakePower(0.0);
	  //shooter_->SetLinearConveyorPower(0);
  }
  return ret;
}

JumbleCommand::~JumbleCommand() {
}
