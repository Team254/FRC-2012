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
  return command_->Run();
  
/*
  if(foundTarget_) {
	int dir = (curAngle < angleGoal_) ? 1 : -1;
	double output = 0; 
	double diff = fabs(angleGoal_ - curAngle);
	if (diff < 4) {
      output = pid_->Update(angleGoal_, curAngle);
	} else {
      double fastGain = (diff < 10) ? 0.0 : (diff > 25) ? .7 : ((diff - 10) * (0.2/15.0)); 
	  output = (.45  + fastGain) * dir;
	}
    printf("goal: %f gyro: %f diff: %f out: %f\n", angleGoal_, curAngle, (curAngle - angleGoal_), output);
    drive_->SetLinearPower(-output, output);
  } else {
    angleGoal_ = 0;
    drive_->SetLinearPower(0, 0);
  }
  
  double turnRate = curAngle - oldAngle_;
  oldAngle_ = curAngle;


  //PidTuner::PushData(angleGoal_, curAngle, 0);
  return (fabs(angleGoal_ - curAngle) < constants_->autoAlignThreshold) &&
		  (fabs(turnRate) < .1);
  */
}
