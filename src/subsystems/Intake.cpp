#include "subsystems/Intake.h"

#include <cmath>

Intake::Intake(Victor* intakeMotor, DoubleSolenoid* intakeSolenoid) {
  constants_ = Constants::GetInstance();
  intakeMotor_ =  intakeMotor;
  intakeSolenoid_ =  intakeSolenoid;
}

void Intake::SetIntakePower(double pwm) {
  intakeMotor_->Set(PwmLimit(pwm));
}

void Intake::SetIntakePosition(IntakePositions pos) {
  if(pos == INTAKE_DOWN) {
    intakeSolenoid_->Set(DoubleSolenoid::kForward);
  } else if (pos == INTAKE_UP) {
    intakeSolenoid_->Set(DoubleSolenoid::kReverse);
  } else if (pos == INTAKE_FLOATING) {
    intakeSolenoid_->Set(DoubleSolenoid::kOff);
  }
}

Intake::IntakePositions Intake::GetIntakePosition() {
  if (intakeSolenoid_->Get() == DoubleSolenoid::kForward) {
    return INTAKE_UP;
  } else if (intakeSolenoid_->Get() == DoubleSolenoid::kReverse) {
    return INTAKE_DOWN;
  } else {
    return INTAKE_FLOATING;
  }
}
