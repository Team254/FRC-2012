#include "main.h"

#include <math.h>
#include <stdio.h>

MainRobot::MainRobot() {
  constants_ = Constants::GetInstance();
  leftDriveMotorA_ = new Victor((int)constants_->leftMotorPortA);
  leftDriveMotorB_ = new Victor((int)constants_->leftMotorPortB);
  rightDriveMotorA_ = new Victor((int)constants_->rightMotorPortA);
  rightDriveMotorB_ = new Victor((int)constants_->rightMotorPortB);
  leftEncoder_ = new Encoder((int)constants_->leftEncoderPortA, (int)constants_->leftEncoderPortB);
  leftEncoder_->Start();
  gyro_ = new Gyro((int)constants_->gyroPort);
  drivebase_ = new Drive(leftDriveMotorA_, leftDriveMotorB_, rightDriveMotorA_, rightDriveMotorB_, leftEncoder_, gyro_);
  leftJoystick_ = new Joystick((int)constants_->leftJoystickPort);
  rightJoystick_ = new Joystick((int)constants_->rightJoystickPort);
}

MainRobot::~MainRobot() {
  delete leftDriveMotorA_;
  delete leftDriveMotorB_;
  delete rightDriveMotorA_;
  delete rightDriveMotorB_;
  delete leftEncoder_;
  delete drivebase_;
  delete leftJoystick_;
  delete rightJoystick_;
  delete gyro_;
}

void MainRobot::DisabledInit() {
}

void MainRobot::AutonomousInit() {
}

void MainRobot::TeleopInit() {
}

void MainRobot::DisabledPeriodic() {
}

void MainRobot::AutonomousPeriodic() {
}

void MainRobot::TeleopPeriodic() {
  // Drive Code
  double straightPower = HandleDeadband(-leftJoystick_->GetY(), 0.1);
  double turnPower = HandleDeadband(rightJoystick_->GetX(), 0.1);
  double leftPower = straightPower + turnPower;
  double rightPower = straightPower - turnPower;
  drivebase_->SetPower(leftPower, rightPower);
  double leftDistance = drivebase_->GetLeftEncoderDistance();
  printf("left: %f\n",leftDistance);
  
}

double MainRobot::HandleDeadband(double val, double deadband) {
  return (fabs(val) > fabs(deadband)) ? val : 0.0;
}
