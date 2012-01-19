#include "main.h"

#include <math.h>
#include <stdio.h>

MainRobot::MainRobot() {
  constants_ = Constants::GetInstance();
  leftDriveMotor_ = new Victor((int)constants_->leftMotorPortA);
  rightDriveMotor_ = new Victor((int)constants_->rightMotorPortA);
  leftEncoder_ = new Encoder((int)constants_->leftEncoderPortA, (int)constants_->leftEncoderPortB);
  leftEncoder_->Start();
  drivebase_ = new Drive(leftDriveMotor_, rightDriveMotor_, leftEncoder_);
  leftJoystick_ = new Joystick((int)constants_->leftJoystickPort);
  rightJoystick_ = new Joystick((int)constants_->rightJoystickPort);
}

MainRobot::~MainRobot() {
  delete leftDriveMotor_;
  delete rightDriveMotor_;
  delete leftEncoder_;
  delete drivebase_;
  delete leftJoystick_;
  delete rightJoystick_;
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
