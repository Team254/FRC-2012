#include "auto/AutoShootCommand.h"
#include "auto/ShootCommand.h"
#include "subsystems/Shooter.h"
#include "subsystems/Intake.h"
#include "vision/BackboardFinder.h"
#include "config/Constants.h"

AutoShootCommand::AutoShootCommand(Shooter* shooter, Intake* intake, 
		           BackboardFinder* target, bool runIntake, int shotsToFire, 
		           double timeout, bool intakeDown) {
	shooter_ = shooter;
	intake_ = intake;
	target_ = target;
	runIntake_ = runIntake;
	shotsToFire_ = shotsToFire;
	timeout_ = timeout;
	intakeDown_ = intakeDown;
	constants_ = Constants::GetInstance();
	//Creates default shootcommand to be replaced during initialize
	cmd_ = new ShootCommand(shooter_, intake_, runIntake_, 0, shotsToFire_, timeout_, intakeDown_);
}

void AutoShootCommand::Initialize() {
	target_->DoVision();
	Shooter::hoodPref pref = Shooter::NO;
	double dist = target_->GetDistance();
	double newVel = (((constants_->shooterKeyFarSpeed - constants_->shooterKeyCloseSpeed ) / (190 - 122)) *
	   			      (dist - 122)) + constants_->shooterKeyCloseSpeed;
	if(dist < 111) {
	  pref = Shooter::DOWN;
	  newVel = (((constants_->shooterFarFenderSpeed - constants_->shooterFenderSpeed ) / (95 - 60)) *
	   		  		  (dist - 60)) + constants_->shooterFenderSpeed;
	}
	delete cmd_;
	cmd_ = new ShootCommand(shooter_, intake_, runIntake_, newVel, shotsToFire_, timeout_, intakeDown_);
	cmd_->Initialize();
}

bool AutoShootCommand::Run() {
  return cmd_->Run();	
}

AutoShootCommand::~AutoShootCommand() {
	delete cmd_;
}
