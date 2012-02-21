#include "drivers/BaselockDriver.h"

#include "config/Constants.h"
#include "subsystems/Drive.h"
#include "subsystems/OperatorControl.h"
#include "subsystems/Pid.h"
#include "utils.hpp"

BaselockDriver::BaselockDriver(Drive* drive, Joystick* leftJoystick) : Driver(drive) {
  constants_ = Constants::GetInstance();
  leftJoystick_ = leftJoystick;
  leftPid_ = new Pid(constants_->baseLockKP, constants_->baseLockKI, constants_->baseLockKD);
  rightPid_ = new Pid(constants_->baseLockKP, constants_->baseLockKI, constants_->baseLockKD);
  Reset();
}

bool BaselockDriver::UpdateDriver() {
  // printf wasn't working, switched to DriverStationLCD temporarily
  DriverStationLCD* lcd = DriverStationLCD::GetInstance();

  // Baselock should be in low gear for maximum torque
  drive_->SetHighGear(false);

  double straightPower = HandleDeadband(-leftJoystick_->GetY(), 0.1);
  baseLockPosition_ += straightPower * .1;
  double leftPosition=drive_->GetLeftEncoderDistance();
  double rightPosition=drive_->GetRightEncoderDistance();
  double leftPower = leftPid_->Update(baseLockPosition_, leftPosition);
  double rightPower = rightPid_->Update(baseLockPosition_, rightPosition);
  static const double baselockErr=0.1;
  double leftError = fabs(leftPosition-baseLockPosition_);
  double rightError = fabs(rightPosition-baseLockPosition_);
  drive_->SetLinearPower(leftPower, rightPower);
  return leftError<baselockErr && rightError<baselockErr;
}

void BaselockDriver::Reset() {
  // Reset encoders and PID
  leftPid_->ResetError();
  rightPid_->ResetError();
  drive_->ResetEncoders();
  baseLockPosition_=0.0;
}

BaselockDriver::~BaselockDriver() {
}
