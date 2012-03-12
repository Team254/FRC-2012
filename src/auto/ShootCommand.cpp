#include "auto/ShootCommand.h"

#include "subsystems/Shooter.h"

ShootCommand::ShootCommand(Shooter* shooter, double timeout) {
  SetTimeout(timeout);
  shooter_ = shooter;
}

void ShootCommand::Initialize() {
   shooter_->SetTargetVelocity(Constants::GetInstance()->autoShootKeyVel);
}

bool ShootCommand::Run() {
  if (timer_->Get() > 2.0) {
  shooter_->SetLinearConveyorPower(.75);
  }
  return false;//TimeoutExpired();
}
