#include "drivers/AutoTurnDriver.h"

#include "WPILib.h"
#include "auto/DriveCommand.h"
#include "subsystems/Drive.h"
#include "util/PidTuner.h"
#include "vision/BackboardFinder.h"

AutoTurnDriver::AutoTurnDriver(Drive* drive, BackboardFinder* target) : Driver(drive) {
  target_ = target;
  command_ = new DriveCommand(drive_, 0.0, 0.0, false, 20.0);
  Reset();
}


void AutoTurnDriver::Reset() {
  justReset_ = true;
  foundTarget_ = false;
  delete command_;
  SetOffsetAngle(0);
  command_ = new DriveCommand(drive_, 0.0, 0.0, false, 100.0);
}

bool AutoTurnDriver::UpdateDriver() {
  if (justReset_) {
    justReset_ = false;
    foundTarget_ = false;
  }
  drive_->SetHighGear(false);
  if (!foundTarget_ && target_->SeesTarget()) {
    // Grab a camera image angle and reset the gyro
    drive_->ResetGyro();
    delete command_;
    command_ = new DriveCommand(drive_, 0.0, (-target_->GetAngle() + offsetAngle_), false, 20.0);
    command_->Initialize();
    foundTarget_ = true;
  }
  if (foundTarget_) {
    return command_->Run();
  } else {
  drive_->SetLinearPower(0,0);
    return false;
  }
}

void AutoTurnDriver::SetOffsetAngle(double angle) {
  offsetAngle_ = angle;
}
