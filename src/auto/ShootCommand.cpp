#include "auto/ShootCommand.h"

#include "subsystems/Shooter.h"
#include "subsystems/Intake.h"
#include "util/PidTuner.h"

ShootCommand::ShootCommand(Shooter* shooter, Intake* intake, bool runIntake,
                       double shootSpeed, int shotsToFire, double timeout, bool doJumble) {
  SetTimeout(timeout);
  shooter_ = shooter;
  intake_ = intake;
  runIntake_ = runIntake;
  shootSpeed_ = shootSpeed;
  shotSpotterTimer_ = new Timer();
  lastShotTimer_ = new Timer();
  shotsToFire_ = shotsToFire;
  reachedSpeed_ = false;
  shotsFired_ = 0;
  downCycles_ = 0;
  atSpeedCycles_ = 0;
  doJumble_ = doJumble;
  jumbleTimer_ = new Timer();
}

void ShootCommand::Initialize() {
   shooter_->Reset();
   shooter_->SetTargetVelocity(shootSpeed_);
   AutoCommand::Initialize();
   lastShotTimer_->Reset();
   lastShotTimer_->Start();
   jumbleTimer_->Start();
   goBack_ = 0;
}

bool ShootCommand::Run() {
  shooter_->SetTargetVelocity(shootSpeed_);

  // Hacked this at SVR to get the seconds balls to stop before shooting
  bool atSpeed = shooter_->AtTargetVelocity() || lastShotTimer_->Get() > 2.0;
  goBack_--;
  if (goBack_ < 0){
	  goBack_ = 0;
  }
  if (atSpeed) {
    if (atSpeedCycles_++ > 5) {
      reachedSpeed_ = true;
      lastShotTimer_->Reset();
    }
    downCycles_ = 0;
    goBack_ = 0;
  } else {
    atSpeedCycles_ = 0;
    if (shooter_->GetVelocity() < shootSpeed_ - 4  && reachedSpeed_) {
      if (downCycles_++ > 4) {
        downCycles_ = 0;
        shotsFired_++;
        goBack_ = 7;
        reachedSpeed_ = false; // Stop running the conveyor
        lastShotTimer_->Reset();
        lastShotTimer_->Start();
        if (shotsFired_ >= shotsToFire_) {
          shotSpotterTimer_->Start();
        }
      }
    }
  }
  jumbleTimer_->Start();
  if (reachedSpeed_) {
    shooter_->SetLinearConveyorPower(1);
    if (runIntake_) {
      if (doJumble_ && jumbleTimer_->Get() > .75) {
        jumbleTimer_->Reset();
        intake_->SetIntakePower(-1);
      }
      else if (doJumble_ && jumbleTimer_->Get() > .6)
        intake_->SetIntakePower(-1);
      else
    	intake_->SetIntakePower(1);
    } else {
       intake_->SetIntakePower(0);
    }
  }
  else if (goBack_ > 0) {
    shooter_->SetLinearConveyorPower(-1);
    intake_->SetIntakePower(0);
  } else {
    shooter_->SetLinearConveyorPower(0);
    intake_->SetIntakePower(0);
  }

  bool done = TimeoutExpired() || (shotsFired_ >= shotsToFire_ &&
      shotSpotterTimer_->Get() > .15);
  if (done) {
    shooter_->SetLinearConveyorPower(0.0);
    intake_->SetIntakePower(0);
  }
  return done;
}

ShootCommand::~ShootCommand()  {
}

