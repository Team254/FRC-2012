#include "WPILib.h"
#include "util/PidTuner.h"
#include "drivers/AutoTurnDriver.h"
#include "subsystems/Drive.h"

AutoTurnDriver::AutoTurnDriver(Drive* drive) : Driver(drive) {
  Reset();
  timer_ = new Timer();
}

void AutoTurnDriver::Reset() {
  angle_ = 0;
}

bool AutoTurnDriver::UpdateDriver() {
  drive_->SetLinearPower(.5,.5);
  double posL = drive_->GetLeftEncoderDistance();
  double vel = (posL - lastPosL_) / (timer_->Get() - lastTimer_);
  lastTimer_ = timer_->Get();
  lastPosL_ = posL; 
  PidTuner::PushData(vel, 0.0, 0.0);
}
