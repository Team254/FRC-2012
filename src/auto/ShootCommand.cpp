#include "auto/ShootCommand.h"

#include "subsystems/Shooter.h"
#include "subsystems/Intake.h"
#include "util/PidTuner.h"

ShootCommand::ShootCommand(Shooter* shooter, Intake* intake, bool runIntake,
		                   double shootSpeed, int shotsToFire, double timeout, bool intakeDown) {
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
  intakeDown_ = intakeDown;
}

void ShootCommand::Initialize() {
   shooter_->Reset();	
   shooter_->SetTargetVelocity(shootSpeed_);
   AutoCommand::Initialize();
   if (intakeDown_) {
   	  intake_->SetIntakePosition(Intake::INTAKE_DOWN);
   	  ///reachedSpeed_ = true;
   }
   lastShotTimer_->Reset();
}

bool ShootCommand::Run() {
  //intake_->SetIntakePosition(Intake::INTAKE_DOWN);
  shooter_->SetTargetVelocity(shootSpeed_);
  lastShotTimer_->Start();
  
  if (timer_->Get() > 2.2 && intakeDown_) {
	  intake_->SetIntakePosition(Intake::INTAKE_FLOATING);
  }
  if (timer_->Get() > 3.5 && intakeDown_)
	  intakeDown_ = false;
  // Hacked this at SVR to get the seconds balls to stop before shooting
  bool atSpeed = shooter_->AtTargetVelocity() || (lastShotTimer_->Get() > 2.0 && lastShotTimer_->Get() < 2.1);
  bool goBack = false;

  if (atSpeed) {
    if (atSpeedCycles_++ > 5)
      reachedSpeed_ = true;
    downCycles_ = 0;
  } else {
    atSpeedCycles_ = 0;
    if(shooter_->GetVelocity() < shootSpeed_ - 4  && reachedSpeed_) {
      if (downCycles_++ > 5) {
        downCycles_ = 0;
        shotsFired_++;
        goBack = true;
        reachedSpeed_ = false; // Stop running the conveyor
        intakeDown_ = false;
        lastShotTimer_->Reset();
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
  } 
  else if (goBack) {
	  shooter_->SetLinearConveyorPower(-1);
	  intake_->SetIntakePower(0);
  } else if (intakeDown_) {
	  shooter_->SetLinearConveyorPower(1);
	  intake_->SetIntakePower(1);
  } else {
    shooter_->SetLinearConveyorPower(0);
    intake_->SetIntakePower(0);
  }

  bool done = TimeoutExpired() || (shotsFired_ >= shotsToFire_ && 
		  shotSpotterTimer_->Get() > .25);
  if (done) {
    //shooter_->SetTargetVelocity(0);
    shooter_->SetLinearConveyorPower(0.0);
    intake_->SetIntakePower(0);
  }
  return done;
}

ShootCommand::~ShootCommand()  {
  //printf("destrcutor for shoot command\n");
}

