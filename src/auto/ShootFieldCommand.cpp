#include "auto/ShootFieldCommand.h"
#include "subsystems/Intake.h"
#include "auto/ShootCommand.h"

ShootFieldCommand::ShootFieldCommand(Shooter* shooter, Intake* intake, bool runIntake,double shootSpeed, int shotsToFire, double timeout) {
  cmd_ = new ShootCommand(shooter, intake, runIntake, shootSpeed, shotsToFire, timeout);
  SetTimeout(timeout);
  intake_ = intake;
  jumbleTimer_ = new Timer();
}

void ShootFieldCommand::Initialize() {
  AutoCommand::Initialize();
  jumbleTimer_->Start();
}

bool ShootFieldCommand::Run(){
  if (jumbleTimer_->Get() > .6) {
	  jumbleTimer_->Reset();
  } else if (jumbleTimer_->Get() > .4) {
	  intake_->SetIntakePower(-1);
  }
  bool done = cmd_->Run();
  return done || TimeoutExpired();
}

ShootFieldCommand::~ShootFieldCommand() {
}
