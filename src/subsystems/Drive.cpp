#include "Drive.h"

Drive::Drive(Victor* left, Victor* right) {
  constants = Constants::GetInstance();
  leftDriveMotors_ = left;
  rightDriveMotors_ = right;
}

void Drive::SetLeftDrivePower(double power) {
  if (power > 1.0) {
    power = 1.0;
  } else if (power < 1.0) {
    power = -1.0;
  }
  leftDriveMotors_->Set(power);
}

void Drive::SetRightDrivePower(double power) {
  if (power > 1.0) {
    power = 1.0;
  } else if (power < 1.0) {
    power = -1.0;
  }
  rightDriveMotors_->Set(-power);
}
