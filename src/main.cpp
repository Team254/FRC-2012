#include "main.h"

#include <math.h>
#include <stdio.h>

#include "util/Functions.h"

MainRobot::MainRobot() {
  constants_ = Constants::GetInstance();
  //  target_ = new BackboardFinder();
  //target_->Start();
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
  operatorControl_ = new OperatorControl((int)constants_->operatorControlPort);

  testPid_ = new Pid(constants_->driveKP, constants_->driveKI, constants_->driveKD);
  baseLockPid_ = new Pid(constants_->baseLockKP, constants_->baseLockKI, constants_->baseLockKD);
  testTimer_ = new Timer();
  testLogger_ = new Logger("/test.log", 2);

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
  delete testPid_;
  testPid_ = new Pid(constants_->driveKP, constants_->driveKI, constants_->driveKD);
  pidTest->Initialize();
  drivebase_->ResetGyro();
  drivebase_->ResetEncoders();
  testPid_->ResetError();
  testTimer_->Reset();
  testTimer_->Start();
}

void MainRobot::TeleopInit() {
  constants_->LoadFile();
  drivebase_->ResetGyro();
  drivebase_->ResetEncoders();
  baseLockPosition_ = drivebase_->GetLeftEncoderDistance();
  baseLockPid_->ResetError();
}

void MainRobot::DisabledPeriodic() {
}

void MainRobot::AutonomousPeriodic() {
  double time = testTimer_->Get();
  double inputValue = Functions::SineWave(time, 5, 2);
  double position = drivebase_->GetLeftEncoderDistance();
  double signal = testPid_->Update(inputValue, position);
  drivebase_->SetLinearPower(signal, signal);
  testLogger_->Log("%f,%f,%f,%f\n", time, inputValue, signal, position);
  PidTuner::PushData(inputValue, position);
}

void MainRobot::TeleopPeriodic() {
  // Drive Code
  double straightPower = HandleDeadband(-leftJoystick_->GetY(), 0.1);
  double turnPower = HandleDeadband(rightJoystick_->GetX(), 0.1);
  double leftPower = straightPower + turnPower;
  double rightPower = straightPower - turnPower;
  drivebase_->SetLinearPower(leftPower, rightPower);
  double position = drivebase_->GetLeftEncoderDistance();
  if (operatorControl_->GetBaseLockSwitch()) {
    baseLockPosition_ += straightPower * .1;
    double signal = baseLockPid_->Update(baseLockPosition_, position);
    drivebase_->SetLinearPower(signal, signal);
  }
  if (!oldBaseLockSwitch_ && operatorControl_->GetBaseLockSwitch()) {
    baseLockPosition_ = position;
  }
  oldBaseLockSwitch_ = operatorControl_->GetBaseLockSwitch();
}

double MainRobot::HandleDeadband(double val, double deadband) {
  return (fabs(val) > fabs(deadband)) ? val : 0.0;
}
