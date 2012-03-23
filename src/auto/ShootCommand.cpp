#include "auto/ShootCommand.h"

#include "subsystems/Shooter.h"
#include "subsystems/Intake.h"

ShootCommand::ShootCommand(Shooter* shooter, Intake* intake, bool runIntake,
		                   double shootSpeed, double timeout) {
  SetTimeout(timeout);
  shooter_ = shooter;
  intake_ = intake;
  runIntake_ = runIntake;
  shootSpeed_ = shootSpeed;
  shooterWaitTimer_ = new Timer();
}

void ShootCommand::Initialize() {
   shooter_->SetTargetVelocity(Constants::GetInstance()->autoShootKeyVel);
   AutoCommand::Initialize();
   shooterWaitTimer_->Reset();
   shooterWaitTimer_->Start();
}

bool ShootCommand::Run() {
  shooter_->SetTargetVelocity(shootSpeed_);
  bool atSpeed = shooter_->PIDUpdate();
  printf("atspeed: %d\n", atSpeed);
  if (!atSpeed) {
	shooterWaitTimer_->Reset();
  }
  if (atSpeed && shooterWaitTimer_->Get() > .25) {
	printf("im at speeed bioootccchhhh\n");
    shooter_->SetLinearConveyorPower(1);
    if (runIntake_) {
      intake_->SetIntakePower(1);
    } else {
       intake_->SetIntakePower(0);
    }
  } else {
    shooter_->SetLinearConveyorPower(0);
    intake_->SetIntakePower(0);
  }

  bool done = TimeoutExpired();
  if (done) {
    shooter_->SetTargetVelocity(0);
    shooter_->PIDUpdate();
    shooter_->SetLinearConveyorPower(0.0);
    intake_->SetIntakePower(0);
  }
  return done;
}

ShootCommand::~ShootCommand()  {
  printf("destrcutor for shoot command\n");
}

