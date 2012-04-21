#include "auto/IntakeCommand.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"

IntakeCommand::IntakeCommand(Intake* intake, Shooter* shooter) {
  SetTimeout(1.0);
  intake_ = intake;
  shooter_ = shooter;
}

void IntakeCommand::Initialize() {
  intake_->SetIntakePower(1.0);
  shooter_->SetLinearConveyorPower(-1.0);
  intake_->SetIntakePosition(Intake::INTAKE_DOWN);
}
bool IntakeCommand::Run(){
  return true;
}

IntakeCommand ::~IntakeCommand() {
}
