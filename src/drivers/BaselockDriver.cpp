#include "drivers/BaselockDriver.h"

#include "config/Constants.h"
#include "subsystems/Drive.h"
#include "subsystems/OperatorControl.h"
#include "auto/DriveCommand.h"
#include "subsystems/Pid.h"
#include "utils.hpp"

BaselockDriver::BaselockDriver(Drive* drive, Joystick* leftJoystick, Joystick* rightJoystick) : Driver(drive) {
  constants_ = Constants::GetInstance();
  leftJoystick_ = leftJoystick;
  rightJoystick_ = rightJoystick;
  cmd_ = new DriveCommand(drive, 0, 0, false, 99999999);
}

bool BaselockDriver::UpdateDriver() {
  cmd_->Run();
  distanceGoal_ += leftJoystick_->GetY() / 4.0 ;
  angleGoal_ = rightJoystick_->GetX() * 15;
  cmd_->SetGoals(distanceGoal_, angleGoal_);
  return false;
}

void BaselockDriver::Reset() {
  drive_->ResetEncoders();
  angleGoal_ = 0;
  distanceGoal_ = 0;
  delete cmd_;
  cmd_ = new DriveCommand(drive_, 0.1, 0.1, false, 99999999);
  cmd_->Initialize();
}

BaselockDriver::~BaselockDriver() {
}
