#include "main.h"

#include <math.h>
#include <stdio.h>

MainRobot::MainRobot() {
  constants_ = Constants::GetInstance();
  printf("HELLO\n");
  printf("%d %d %d %d\n",(int)constants_->leftMotorPortA,(int)constants_->leftMotorPortB,(int)constants_->rightMotorPortA,(int)constants_->rightMotorPortB);
  leftDriveMotorA_ = new Victor((int)constants_->leftMotorPortA);
  leftDriveMotorB_ = new Victor((int)constants_->leftMotorPortB);
  rightDriveMotorA_ = new Victor((int)constants_->rightMotorPortA);
  rightDriveMotorB_ = new Victor((int)constants_->rightMotorPortB);
  drivebase_ = new Drive(leftDriveMotorA_,leftDriveMotorB_,rightDriveMotorA_,rightDriveMotorB_);
  leftJoystick_ = new Joystick((int)constants_->leftJoystickPort);
  rightJoystick_ = new Joystick((int)constants_->rightJoystickPort);
  printf("DONE\n");
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
}

double MainRobot::HandleDeadband(double val, double deadband) {
  return (fabs(val) > fabs(deadband)) ? val : 0.0;
}
