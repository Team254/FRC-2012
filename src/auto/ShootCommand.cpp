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
  shooterWaitTimer_ = new Timer();
  shotSpotterTimer_ = new Timer();
  shotsToFire_ = shotsToFire;
  countLatch_ = false;
  reachedSpeed_ = false;
  shotsFired_ = 0;
  downCycles_ = 0;
}

void ShootCommand::Initialize() {
   shooter_->SetTargetVelocity(Constants::GetInstance()->autoShootKeyVel);
   AutoCommand::Initialize();
   shooterWaitTimer_->Reset();
   shooterWaitTimer_->Start();
   shotSpotterTimer_->Reset();
}

bool ShootCommand::Run() {
  intake_->SetIntakePosition(Intake::INTAKE_DOWN);
  shooter_->SetTargetVelocity(shootSpeed_);
  static int counter= 0;
  static int dropCounts = 0;
  // printf("count: %d\n", counter++);
  bool atSpeed = shooter_->AtTargetVelocity();
  counter++;
  if (!atSpeed) {
	shooterWaitTimer_->Reset();
  }
  if (atSpeed ) {
	countLatch_ = false;
    shooter_->SetLinearConveyorPower(1);
    if (runIntake_) {
      intake_->SetIntakePower(1);
    } else {
       intake_->SetIntakePower(0);
    }
    reachedSpeed_ = true;
    dropCounts = 0;
  } else {
    shooter_->SetLinearConveyorPower(0);
    intake_->SetIntakePower(0);
    if(shooter_->GetVelocity() < shootSpeed_ - 8 && !countLatch_ && reachedSpeed_) {
      if (dropCounts++ > 10) {
    	dropCounts = 0;
        shotsFired_++;
        countLatch_ = true;
        printf("I shot a ball %d %d %f\n", shotsFired_, counter, shooter_->GetVelocity());
        //countLatch_ = true;
        if(shotsFired_ >= shotsToFire_) {
    	    shotSpotterTimer_->Start();
        }
      }
    } 
  }
  bool done = TimeoutExpired() || (shotsFired_ >= shotsToFire_ && 
		  shotSpotterTimer_->Get() > .25);
  if (done) {
    shooter_->SetTargetVelocity(0);
    //shooter_->PIDUpdate();
    shooter_->SetLinearConveyorPower(0.0);
    intake_->SetIntakePower(0);
  }
  return done;
}

ShootCommand::~ShootCommand()  {
  printf("destrcutor for shoot command\n");
}

