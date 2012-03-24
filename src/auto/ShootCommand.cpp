#include "auto/ShootCommand.h"

#include "subsystems/Shooter.h"
#include "subsystems/Intake.h"

ShootCommand::ShootCommand(Shooter* shooter, Intake* intake, bool runIntake,
		                   double shootSpeed, int shotsToFire, double timeout) {
  SetTimeout(timeout);
  shooter_ = shooter;
  intake_ = intake;
  runIntake_ = runIntake;
  shootSpeed_ = shootSpeed;
  shooterWaitTimer_ = new Timer();
  shotSpotterTimer_ = new Timer();
  shotsToFire_ = shotsToFire;
  countLatch_ = false;
  reachedSpeed_ = false;
  shotsFired_ = 0;
}

void ShootCommand::Initialize() {
   shooter_->SetTargetVelocity(Constants::GetInstance()->autoShootKeyVel);
   AutoCommand::Initialize();
   shooterWaitTimer_->Reset();
   shooterWaitTimer_->Start();
   shotSpotterTimer_->Reset();
}

bool ShootCommand::Run() {
  shooter_->SetTargetVelocity(shootSpeed_);
  bool atSpeed = shooter_->AtTargetVelocity();
  if (!atSpeed) {
	shooterWaitTimer_->Reset();
  }
  if (atSpeed && shooterWaitTimer_->Get() > .25) {
	countLatch_ = false;
    shooter_->SetLinearConveyorPower(1);
    if (runIntake_) {
      intake_->SetIntakePower(1);
    } else {
       intake_->SetIntakePower(0);
    }
    reachedSpeed_ = true;
  } else {
    shooter_->SetLinearConveyorPower(0);
    intake_->SetIntakePower(0);
    if(shooter_->GetVelocity() < shootSpeed_ - 7 && !countLatch_ && reachedSpeed_) {
      shotsFired_++;
      countLatch_ = true;
      if(shotsFired_ >= shotsToFire_) {
    	  shotSpotterTimer_->Start();
      }
    } 
  }
  bool done = TimeoutExpired() || (shotsFired_ >= shotsToFire_ && 
		  shotSpotterTimer_->Get() > .01);
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

