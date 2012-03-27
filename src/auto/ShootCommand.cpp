#include "auto/ShootCommand.h"

#include "subsystems/Shooter.h"
#include "subsystems/Intake.h"
#include "util/PidTuner.h"

ShootCommand::ShootCommand(Shooter* shooter, Intake* intake, bool runIntake,
		                   double shootSpeed, int shotsToFire, double timeout) {
  SetTimeout(timeout);
  shooter_ = shooter;
  intake_ = intake;
  runIntake_ = runIntake;
  shootSpeed_ = shootSpeed;
  shotSpotterTimer_ = new Timer();
  shotsToFire_ = shotsToFire;
  reachedSpeed_ = false;
  shotsFired_ = 0;
  downCycles_ = 0;
  atSpeedCycles_ = 0;
}

void ShootCommand::Initialize() {
   shooter_->Reset();	
   shooter_->SetTargetVelocity(Constants::GetInstance()->autoShootKeyVel);
   AutoCommand::Initialize();
}

bool ShootCommand::Run() {
  shooter_->SetTargetVelocity(shootSpeed_);
  bool atSpeed = shooter_->AtTargetVelocity();

  if (atSpeed) {
    if (atSpeedCycles_++ > 5)
      reachedSpeed_ = true;
    downCycles_ = 0;
  } else {
    atSpeedCycles_ = 0;
    if(shooter_->GetVelocity() < shootSpeed_ - 7  && reachedSpeed_) {
      if (downCycles_++ > 10) {
        downCycles_ = 0;
        shotsFired_++;
        reachedSpeed_ = false; // Stop running the conveyor
        if(shotsFired_ >= shotsToFire_) {
          shotSpotterTimer_->Start();
        }
      }
    }
  }

  if (reachedSpeed_) {
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

  bool done = TimeoutExpired() || (shotsFired_ >= shotsToFire_ && 
		  shotSpotterTimer_->Get() > .25);
  if (done) {
    shooter_->SetTargetVelocity(0);
    shooter_->SetLinearConveyorPower(0.0);
    intake_->SetIntakePower(0);
  }
  return done;
}

ShootCommand::~ShootCommand()  {
  printf("destrcutor for shoot command\n");
}

