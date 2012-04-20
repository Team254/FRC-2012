#include "auto/SetIntakePositionCommand.h"
#include "subsystems/Intake.h"

SetIntakePositionCommand::SetIntakePositionCommand(Intake* intake, Intake::IntakePositions pos) {
  SetTimeout(1.0);
  intake_ = intake;
  pos_ = pos;
}

void SetIntakePositionCommand::Initialize() {
  intake_->SetIntakePosition(pos_);
}

bool SetIntakePositionCommand::Run(){
  return true;
}

SetIntakePositionCommand ::~SetIntakePositionCommand() {
}
