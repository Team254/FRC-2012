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
  pid_ = new Pid(&constants_->autoAlignKP, &constants_->autoAlignKI, &constants_->autoAlignKD);
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
  target_->DoVision();
}


bool AutoTurnDriver::UpdateDriver() {
  static int eltaco = 0;
  if(eltaco%10==0) {
	  target_->DoVision();
  }
  eltaco++;	
  if (justReset_) {
    justReset_ = false;
    timer_->Reset();
    timer_->Start();
    lastTimer_ = -1; // fix this later
    lastPosL_ = drive_->GetLeftEncoderDistance();
    lastPosR_ = drive_->GetRightEncoderDistance();
    printf("RESET FARGAGAGJADGAHGDSGDHFGASG\n");
  }
  drive_->SetHighGear(false); 

  double curX = 0;
  if (target_->SeesTarget()) {
    curX = target_->GetX();
    printf("curX: %f\n", curX);
  }

  double output = pid_->Update(0, curX);

  //  double setpointL = setpoint * constants_->autoAlignKP;
  //  double setpointR = -setpointL * constants_->autoAlignKP;

  drive_->SetLinearPower(-output, output);
  PidTuner::PushData(target_->GetX(), 0.0, output);
  return 0;
#if 0
  double posL = drive_->GetLeftEncoderDistance();
  double velL = (posL - lastPosL_) / (timer_->Get() - lastTimer_);
  velL = filterL_->Update(velL);
  outputValueL_ += pidL_->Update(setpoint, velL);
  double fixL = 0;


  double posR = drive_->GetRightEncoderDistance();
  double velR = (posR - lastPosR_) / (timer_->Get() - lastTimer_);
  velR = filterR_->Update(velR);
  outputValueR_ += pidR_->Update(setpointR, velR);
  double fixR = 0;

  if ((fabs(velR) + fabs(velL) / 2) > 3  && staticFriction_) {
    staticFriction_ = false;
    outputValueL_ = outputValueL_ + ((setpointL < 0) ? -constants_->breakStaticOffset : constants_->breakStaticOffset);
    outputValueR_ = outputValueR_ + ((setpointR < 0) ? -constants_->breakStaticOffset : constants_->breakStaticOffset);
  }

  if (staticFriction_) {
    fixR = (setpointR < 0) ? -constants_->breakStaticOffset : constants_->breakStaticOffset;
    fixL = (setpointL < 0) ? -constants_->breakStaticOffset : constants_->breakStaticOffset;
    printf("FIXXXXXXXXXXXXXXXXXXX!!!!!");
  }

  printf("%f | %f | %f | %f\n\n", (float) outputValueL_ + fixL, (float)outputValueR_ + fixR, (float)fixL, (float) fixR);
  drive_->SetLinearPower(outputValueL_ + fixL, outputValueR_ + fixR);

  lastTimer_ = timer_->Get();
  lastPosL_ = posL; 
  lastPosR_ = posR;
#endif
}
