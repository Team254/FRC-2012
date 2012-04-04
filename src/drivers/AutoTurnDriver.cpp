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
  command_ = new DriveCommand(drive_, 0.0, 0.0, false, 20.0);
}

bool AutoTurnDriver::UpdateDriver() {
  if (justReset_) {
    justReset_ = false;
    foundTarget_ = false;
  }
  if (!foundTarget_ && !target_->SeesTarget()){
      target_->DoVision();
    }else if (!foundTarget_ && target_->SeesTarget()) {
	  // Grab a camera image angle and reset the gyro
	  drive_->ResetGyro();
	  delete command_;
	  command_ = new DriveCommand(drive_, 0.0, -target_->GetAngle(), false, 20.0);
	  command_->Initialize();
	  foundTarget_ = true;
  }
  if (foundTarget_) {
	  return command_->Run();
  } else {
  	return false;
  }
}
