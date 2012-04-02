#include "auto/QueueBallCommand.h"

#include "subsystems/Shooter.h"
#include "subsystems/Intake.h"

QueueBallCommand::QueueBallCommand(Shooter* shooter, Intake* intake, double timeout) {
  SetTimeout(timeout);
  shooter_ = shooter;
  intake_ = intake;
}

void QueueBallCommand::Initialize() {
  shooter_->Reset();
  AutoCommand::Initialize();
}

bool QueueBallCommand::Run() {
  shooter_->SetLinearConveyorPower(1);
  intake_->SetIntakePower(1);
  bool done = shooter_->GetBallRange() > 180 || TimeoutExpired();
  if (done) {
    shooter_->SetLinearConveyorPower(0);
    intake_->SetIntakePower(0);
  }
  return done;
}

QueueBallCommand::~QueueBallCommand() {
  
}
