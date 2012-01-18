#include "main.h"

MainRobot::MainRobot() 
    : period_(kDefaultPeriod) {
  constants_ = Constants::GetInstance();
  leftDriveMotorA_ = new Victor((int)constants_->leftMotorPortA);
  leftDriveMotorB_ = new Victor((int)constants_->leftMotorPortB);
  rightDriveMotorA_ = new Victor((int)constants_->rightMotorPortA);
  rightDriveMotorB_ = new Victor((int)constants_->rightMotorPortB);
  drivebase_ = new Drive(leftDriveMotorA_,leftDriveMotorB_,rightDriveMotorA_,rightDriveMotorB_);
  leftJoystick_ = new Joystick((int)constants_->leftJoystickPort);
  rightJoystick_ = new Joystick((int)constants_->rightJoystickPort);
  controls_ = new ControlBoard(leftJoystick_, rightJoystick_);
}

void MainRobot::StartCompetition() {
}

void MainRobot::RobotInit() {
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
}

void MainRobot::DisabledContinuous() {
}

void MainRobot::AutonomousContinuous() {
}

void MainRobot::TeleopContinuous() {
}

void MainRobot::SetPeriod(double period) {
    period_=period;
}

double MainRobot::GetPeriod() {
    return period_;
}

double MainRobot::GetLoopsPerSec() {
    return 1.0/period_;
}

