#include "subsystems/Intake.h"

#include <cmath>

Intake::Intake(Victor* intakeMotor1, Victor* intakeMotor2, Victor* intakeMotor3, Solenoid* intakeUpSolenoid,
               Solenoid* intakeDownSolenoid) {
  constants_ = Constants::GetInstance();
  intakeMotor1_ =  intakeMotor1;
  intakeMotor2_ =  intakeMotor2;
  intakeMotor3_ =  intakeMotor3;
  intakeUpSolenoid_ =  intakeUpSolenoid;
  intakeDownSolenoid_ = intakeDownSolenoid;
}

void Intake::SetIntakePower(double pwm) {
  intakeMotor1_->Set(PwmLimit(pwm));
  intakeMotor2_->Set(PwmLimit(pwm));
  intakeMotor3_->Set(PwmLimit(pwm));
}

void Intake::SetIntakePosition(IntakePositions pos) {
  // The intake defaults to up when power is off, so the down-side solenoid supplies air when energized,
  // while the up-side solenoid does the opposite.
  if (pos == INTAKE_DOWN) {
    intakeUpSolenoid_->Set(true);
    intakeDownSolenoid_->Set(true);
  } else if (pos == INTAKE_UP) {
    intakeUpSolenoid_->Set(false);
    intakeDownSolenoid_->Set(false);
  } else if (pos == INTAKE_FLOATING) {
    intakeUpSolenoid_->Set(true);
    intakeDownSolenoid_->Set(false);
  }
}

Intake::IntakePositions Intake::GetIntakePosition() {
  if (!intakeUpSolenoid_->Get() && !intakeDownSolenoid_->Get()) {
    return INTAKE_UP;
  } else if (intakeUpSolenoid_->Get() && intakeDownSolenoid_->Get()) {
    return INTAKE_DOWN;
  } else {
    return INTAKE_FLOATING;
  }
}
