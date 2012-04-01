#include "WPILib.h"
#include "util/PidTuner.h"
#include "drivers/AutoTurnDriver.h"
#include "subsystems/Drive.h"
#include "util/MovingAverageFilter.h"
#include "subsystems/Pid.h"
#include "config/Constants.h"
#include "vision/BackboardFinder.h"

AutoTurnDriver::AutoTurnDriver(Drive* drive, BackboardFinder* target) : Driver(drive) {
  timer_ = new Timer();
  filterL_ = new MovingAverageFilter(5);
  filterR_ = new MovingAverageFilter(5);
  constants_ = Constants::GetInstance();
  pid_ = new Pid(&constants_->autoCameraAlignKP, &constants_->autoCameraAlignKI, &constants_->autoCameraAlignKD);
  //pid_ = new Pid(&constants_->turnKP, &constants_->turnKI, &constants_->turnKD);
  pidL_ = new Pid(&constants_->driveVelKP, &constants_->driveVelKI, &constants_->driveVelKD);
  pidR_ = new Pid(&constants_->driveVelKP, &constants_->driveVelKI, &constants_->driveVelKD);
  target_ = target;
  Reset();
}


void AutoTurnDriver::Reset() {
  angle_ = 0;
  justReset_ = true;
  pidL_->ResetError();
  pidR_->ResetError();
  pid_->ResetError();
  outputValueL_ = outputValueR_ = 0;
  staticFriction_ = true;
  filterL_->Reset();
  filterR_->Reset();
  //target_->DoVision();
  foundTarget_ = false;
  printf("did reset auto turn\n");
  
}


bool AutoTurnDriver::UpdateDriver() {
  double curAngle = -drive_->GetGyroAngle();
  if (justReset_) {
    justReset_ = false;
    timer_->Reset();
    timer_->Start();
    lastTimer_ = -1; // fix this later
    lastPosL_ = drive_->GetLeftEncoderDistance();
    lastPosR_ = drive_->GetRightEncoderDistance();
    foundTarget_ = false;
  }
  drive_->SetHighGear(false); 
  drive_->SetBrakeOn(false);
  if (!foundTarget_ && !target_->SeesTarget()){
      target_->DoVision();
      
    }else if (!foundTarget_ && target_->SeesTarget()) {
    // Grab a camera image angle and reset the gyro
    drive_->ResetGyro();
    angleGoal_ = -target_->GetAngle();
    printf("**********\n**ANGLE: %f %f \n\n", angleGoal_, drive_->GetGyroAngle());
    foundTarget_ = true;
  }
  

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
}
