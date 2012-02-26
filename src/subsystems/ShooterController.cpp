#include "subsystems/ShooterController.h"

ShooterController::ShooterController(Shooter* shooter, Intake* intake) {
  shooter_ = shooter;
  intake_ = intake;
  intakeDownTimer_ = new Timer();
  intakeRunning_ = false;
}

void ShooterController::RunIntake() {
  if (!intakeRunning_) {
    intakeDownTimer_->Reset();
    intakeDownTimer_->Start();
  }
  if (intakeDownTimer_->Get() < .5) {
    intake_->SetIntakePosition(Intake::INTAKE_DOWN);
    intake_->SetIntakePower(0);
  }
  else {
    intake_->SetIntakePosition(Intake::INTAKE_FLOATING);
    intake_->SetIntakePower(1);
  }
}

void ShooterController::StopIntake() {
  intakeRunning_  = false;
  intake_->SetIntakePosition(Intake::INTAKE_UP);
  intake_->SetIntakePower(0);
}

void ShooterController::IntakeDown() {
  intake_->SetIntakePosition(Intake::INTAKE_DOWN);
}
