#include "subsystems/Drive.h"

Drive::Drive(Victor* leftVictorA, Victor* leftVictorB, Victor* rightVictorA, Victor* rightVictorB, Encoder* leftEncoder) {
  constants = Constants::GetInstance();
  leftDriveMotorA_ = leftVictorA;
  leftDriveMotorB_ = leftVictorB;
  rightDriveMotorA_ = rightVictorA;
  rightDriveMotorB_ = rightVictorB;
  leftDriveEncoder_ = leftEncoder;
}

void Drive::SetPower(double left, double right) {
  SetLeftDrivePower(left);
  SetRightDrivePower(right);
}

void Drive::SetLeftDrivePower(double power) {
  if (power > 1.0) {
    power = 1.0;
  } else if (power < -1.0) {
    power = -1.0;
  }
  leftDriveMotorA_->Set(power);
  leftDriveMotorB_->Set(power);
}

void Drive::SetRightDrivePower(double power) {
  if (power > 1.0) {
    power = 1.0;
  } else if (power < -1.0) {
    power = -1.0;
  }
  rightDriveMotorA_->Set(-power);
  rightDriveMotorB_->Set(-power);
}

double Drive::GetLeftEncoderDistance() {
    // Number of clicks read by encoder / number of clicks per rotation *
    // gear ratio from encoder to wheel * wheel circumference

    // Don't have current specs now, just return encoder rotations
    return leftDriveEncoder_->Get() / 256.0;
}
