#include "subsystems/Drive.h"

Drive::Drive(Victor* leftA, Victor* leftB, Victor* rightA, Victor* rightB) {
  constants = Constants::GetInstance();
  leftDriveMotorA_ = leftA;
  leftDriveMotorA_ = leftB;
  rightDriveMotorB_ = rightA;
  rightDriveMotorB_ = rightB;
}

void Drive::SetPower(double left, double right) {
  SetLeftDrivePower(left);
  SetRightDrivePower(right);
}

void Drive::SetLeftDrivePower(double power) {
  if (power > 1.0) {
    power = 1.0;
  } else if (power < 1.0) {
    power = -1.0;
  }
  leftDriveMotorA_->Set(power);
  leftDriveMotorB_->Set(power);
}

void Drive::SetRightDrivePower(double power) {
  if (power > 1.0) {
    power = 1.0;
  } else if (power < 1.0) {
    power = -1.0;
  }
  rightDriveMotorA_->Set(-power);
  rightDriveMotorB_->Set(-power);
}
