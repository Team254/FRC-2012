#include "WPILib.h"
#include "util/PidTuner.h"
#include "drivers/AutoTurnDriver.h"
#include "subsystems/Drive.h"
#include "util/MovingAverageFilter.h"
#include "subsystems/Pid.h"
#include "config/Constants.h"

AutoTurnDriver::AutoTurnDriver(Drive* drive) : Driver(drive) {
  timer_ = new Timer();
  filterL_ = new MovingAverageFilter(5);
  filterR_ = new MovingAverageFilter(5);
  constants_ = Constants::GetInstance();
  pidL_ = new Pid(&constants_->driveVelKP, &constants_->driveVelKI, &constants_->driveVelKD);
  pidR_ = new Pid(&constants_->driveVelKP, &constants_->driveVelKI, &constants_->driveVelKD);
  Reset();
}


void AutoTurnDriver::Reset() {
  angle_ = 0;
  justReset_ = true;
  pidL_->ResetError();
  pidR_->ResetError();
}


bool AutoTurnDriver::UpdateDriver() {
  if (justReset_) {
    justReset_ = false;
    timer_->Reset();
    timer_->Start();
    lastTimer_ = -1; // fix this later
    lastPosL_ = drive_->GetLeftEncoderDistance();
    lastPosR_ = drive_->GetRightEncoderDistance();
  }
  drive_->SetHighGear(false); 
  double posL = drive_->GetLeftEncoderDistance();
  double velL = (posL - lastPosL_) / (timer_->Get() - lastTimer_);
  velL = filterL_->Update(velL);
  outputValueL_ += pidL_->Update(20, velL);

  double posR = drive_->GetRightEncoderDistance();
  double velR = (posR - lastPosR_) / (timer_->Get() - lastTimer_);
  velR = filterR_->Update(velR);
  outputValueR_ += pidR_->Update(-20, velR);

  drive_->SetLinearPower(outputValueL_, outputValueR_);

  lastTimer_ = timer_->Get();
  lastPosL_ = posL; 
  lastPosR_ = posR;
  PidTuner::PushData(velL, velR, 0.0);
}
