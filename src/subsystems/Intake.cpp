#include "subsystems/Intake.h"

#include <cmath>

Intake::Intake(Victor* intakeMotor1, Victor* intakeMotor2, Victor* intakeMotor3, DoubleSolenoid* intakeSolenoid) {
  constants_ = Constants::GetInstance();
  intakeMotor1_ =  intakeMotor1;
  intakeMotor2_ =  intakeMotor2;
  intakeMotor3_ =  intakeMotor3;
  intakeSolenoid_ =  intakeSolenoid;
}

void Intake::SetIntakePower(double pwm) {
  intakeMotor1_->Set(PwmLimit(pwm));
  intakeMotor2_->Set(PwmLimit(pwm));
  intakeMotor3_->Set(PwmLimit(pwm));
}

void Intake::SetIntakePosition(IntakePositions pos) {
  if (pos == INTAKE_DOWN) {
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
