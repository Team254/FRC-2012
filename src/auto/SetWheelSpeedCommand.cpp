#include "auto/SetWheelSpeedCommand.h"
#include "subsystems/Shooter.h"

SetWheelSpeedCommand::SetWheelSpeedCommand(Shooter* shooter, double speed) {
  shooter_ = shooter;
  speed_ = speed;
}

void SetWheelSpeedCommand::Initialize() {
  shooter_->SetTargetVelocity(speed_);
}

bool SetWheelSpeedCommand::Run(){
  return true;
}

SetWheelSpeedCommand::~SetWheelSpeedCommand() {
}
