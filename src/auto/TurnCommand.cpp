#include "auto/TurnCommand.h"

#include "subsystems/Drive.h"
#include "subsystems/Pid.h"

TurnCommand::TurnCommand(Drive* drive, double angle, double timeout) {
  SetTimeout(timeout);
  angle_ = angle;
  drive_ = drive;
  oldAngle_ = 9999;
//  turnPid_ = new Pid(0,0,0); // TODO add constants
}

void TurnCommand::Initialize() {
  drive_->ResetGyro();
}

bool TurnCommand::Run() {
  double curAngle = drive_->GetGyroAngle();
  double power = turnPid_->Update(angle_, curAngle);
  drive_->SetLinearPower(-power, power);

  if (curAngle - angle_ < 1.0 && curAngle - oldAngle_ < .1) { // Make this better
    return true;
  }
  oldAngle_ = curAngle;

  return TimeoutExpired();
}
