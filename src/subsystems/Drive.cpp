#include "subsystems/Drive.h"

#include <cmath>

Drive::Drive(Victor* leftVictorA, Victor* leftVictorB, Victor* rightVictorA, Victor* rightVictorB,
             Solenoid* shiftSolenoid, Encoder* leftEncoder, Encoder* rightEncoder, Gyro* gyro) {
  constants = Constants::GetInstance();
  leftDriveMotorA_ = leftVictorA;
  leftDriveMotorB_ = leftVictorB;
  rightDriveMotorA_ = rightVictorA;
  rightDriveMotorB_ = rightVictorB;
  shiftSolenoid_ = shiftSolenoid;
  SetHighGear(true); // Default to high gear
  leftDriveEncoder_ = leftEncoder;
  rightDriveEncoder_ = rightEncoder;
  gyro_ = gyro;
  gyro_->Reset();
}

void Drive::SetLinearPower(double left, double right) {
   SetPower(Linearize(left), Linearize(right));
}

double Drive::GetLeftEncoderDistance() {
  // Number of clicks read by encoder / number of clicks per rotation * gear ratio from encoder to wheel *
  // wheel circumference

  // Don't have current specs now, just return encoder rotations
  return -leftDriveEncoder_->Get() / 256.0;
}

double Drive::GetRightEncoderDistance() {
  // Number of clicks read by encoder / number of clicks per rotation * gear ratio from encoder to wheel *
  // wheel circumference

  // Don't have current specs now, just return encoder rotations
  return rightDriveEncoder_->Get() / 128.0;
}

void Drive::ResetEncoders() {
  leftDriveEncoder_->Reset();
  rightDriveEncoder_->Reset();
}

void Drive::SetHighGear(bool highGear) {
  shiftSolenoid_->Set(highGear);
}

double Drive::GetGyroAngle() {
  return gyro_->GetAngle();
}

void Drive::ResetGyro() {
  gyro_->Reset();
}

void Drive::SetGyroSensitivity(double sensitivity) {
  gyro_->SetSensitivity(sensitivity);
}

void Drive::SetPower(double left, double right) {
  left = SetLimit(left);
  right = SetLimit(right);
  leftDriveMotorA_->Set(left);
  leftDriveMotorB_->Set(left);
  rightDriveMotorA_->Set(-right);
  rightDriveMotorB_->Set(-right);
}

double Drive::Linearize(double x) {
  if (x > 0) {
    return constants->linearCoeffA * pow(x, 4) + constants->linearCoeffB * pow(x, 3) +
        constants->linearCoeffC * pow(x, 2) + constants->linearCoeffD * x + constants->linearCoeffE;
  } else if (x < 0) {
    // Rotate the linearization function by 180 degrees to handle negative input.
    return -Linearize(-x);
  } else {
    return 0;
  }
}

double Drive::SetLimit(double x) {
  if (x > 1.0) {
    return 1.0;
  } else if (x < -1.0) {
    return -1.0;
  } else {
    return x;
  }
}
