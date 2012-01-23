#include "main.h"

#include <math.h>
#include <stdio.h>

MainRobot::MainRobot() {
  constants_ = Constants::GetInstance();
  target_ = Target::GetInstance();
  leftDriveMotorA_ = new Victor((int)constants_->leftMotorPortA);
  leftDriveMotorB_ = new Victor((int)constants_->leftMotorPortB);
  rightDriveMotorA_ = new Victor((int)constants_->rightMotorPortA);
  rightDriveMotorB_ = new Victor((int)constants_->rightMotorPortB);
  leftEncoder_ = new Encoder((int)constants_->leftEncoderPortA, (int)constants_->leftEncoderPortB);
  leftEncoder_->Start();
  rightEncoder_ = new Encoder((int)constants_->rightEncoderPortA, (int)constants_->rightEncoderPortB);
  rightEncoder_->Start();
  gyro_ = new Gyro((int)constants_->gyroPort);
  drivebase_ = new Drive(leftDriveMotorA_, leftDriveMotorB_, rightDriveMotorA_, rightDriveMotorB_, leftEncoder_,
                         rightEncoder_, gyro_);
  leftJoystick_ = new Joystick((int)constants_->leftJoystickPort);
  rightJoystick_ = new Joystick((int)constants_->rightJoystickPort);
  //Autonomous stuff goes here
  pidTest = new DriveCommand (drivebase_, 36);
  test = new SequentialCommand(1, pidTest);
}

MainRobot::~MainRobot() {
  // Typically members should be deleted in the order they're created
  delete gyro_;
  delete rightEncoder_;
  delete drivebase_;
  delete leftJoystick_;
  delete rightJoystick_;
  delete leftEncoder_;
  delete rightDriveMotorB_;
  delete rightDriveMotorA_;
  delete leftDriveMotorB_;
  delete leftDriveMotorA_;
}

void MainRobot::DisabledInit() {
}

void MainRobot::AutonomousInit() {
  constants_->LoadFile();
  test->Initialize();
}

void MainRobot::TeleopInit() {
  constants_->LoadFile();
}

void MainRobot::DisabledPeriodic() {
}

void MainRobot::AutonomousPeriodic() {
	test->Run();
}

void MainRobot::TeleopPeriodic() {
  // Drive Code
  double straightPower = HandleDeadband(-leftJoystick_->GetY(), 0.1);
  double turnPower = HandleDeadband(rightJoystick_->GetX(), 0.1);
  double leftPower = straightPower + turnPower;
  double rightPower = straightPower - turnPower;
  drivebase_->SetLinearPower(leftPower, rightPower);
  double leftDistance = drivebase_->GetLeftEncoderDistance();
  double rightDistance = drivebase_->GetRightEncoderDistance();
}

double MainRobot::HandleDeadband(double val, double deadband) {
  return (fabs(val) > fabs(deadband)) ? val : 0.0;
}
