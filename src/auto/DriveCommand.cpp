#include "auto/DriveCommand.h"

#include "subsystems/Drive.h"
#include "subsystems/Pid.h"
#include "util/PidTuner.h"
#include "config/Constants.h"

DriveCommand::DriveCommand(Drive* drive, double distance, bool usePizza, double timeout) {
  SetTimeout(timeout);
  drive_ = drive;
  distanceGoal_ = distance;
  usePizza_ = usePizza;
  resetPizza_ =  usePizza; //(usePizza && drive->GetPizzaUp());

  Constants* constants = Constants::GetInstance();
  leftPid_ = new Pid(&constants->driveKP, &constants->driveKI, &constants->driveKD);
  rightPid_ = new Pid(&constants->driveKP, &constants->driveKI, &constants->driveKD);
  driveTimer_ = new Timer();
  brakeTimer_ = new Timer();
  prevTime_ = 0;
  prevLeftDist_ = 0;
  prevRightDist_ = 0;
}

void DriveCommand::Initialize() {
  AutoCommand::Initialize();
  drive_->ResetEncoders();
  drive_->SetPizzaWheelDown(usePizza_);
  startingAngle_ = drive_->GetGyroAngle();
  driveTimer_->Reset();
  driveTimer_->Start();
  brakeTimer_->Reset();
  prevTime_ = driveTimer_->Get();
  prevLeftDist_ = drive_->GetLeftEncoderDistance();
  prevRightDist_ = drive_->GetRightEncoderDistance();
}

bool DriveCommand::Run() {
  if (TimeoutExpired()) {
	drive_->SetLinearPower(0, 0);
	return true;
  }
  double currLeftDist = drive_->GetLeftEncoderDistance();
  double currRightDist = drive_->GetRightEncoderDistance();
  double currTime = driveTimer_->Get();
  double lVel = (currLeftDist - prevLeftDist_)/(currTime - prevTime_);
  double rVel = (currRightDist - prevRightDist_)/(currTime - prevTime_);
  prevTime_ = currTime;
  prevLeftDist_ = currLeftDist;
  prevRightDist_ = currRightDist;
  // If the goal has been reached, this command is done.
  if(brakeTimer_->Get() > .2) {
  	  drive_->SetLinearPower(0, 0);
  	  printf("returnd from brake timer\n");
  	  return true;
    }
  
  

  // Get PID feedback and send back to the motors.
  double leftPIDOutput = PwmLimit(leftPid_->Update(distanceGoal_, currLeftDist));
  double rightPIDOutput = PwmLimit(rightPid_->Update(distanceGoal_, currRightDist));
  double angleDiff = drive_->GetGyroAngle() - startingAngle_;
  double straightGain = angleDiff * Constants::GetInstance()->straightDriveGain;
  double leftPwr = leftPIDOutput - straightGain;
  double rightPwr = rightPIDOutput + straightGain;
  if (fabs(currLeftDist - distanceGoal_ ) < 2 || fabs(currRightDist- distanceGoal_) < 2) {
      drive_->SetPizzaWheelDown(resetPizza_);
      brakeTimer_->Start();
    
      //return false;
  }
  
  if (brakeTimer_->Get() != 0.0 ){
	  int dir = (distanceGoal_ > 0) ? 1 : -1;
	  drive_->SetLinearPower((-.3 * dir) - straightGain, (-.3 * dir) + straightGain);
      return false;
  }
    
  drive_->SetLinearPower(leftPwr, rightPwr);
  PidTuner::PushData(leftPwr, straightGain, 0);

  // Indicate that the goal has not yet been reached.
  return false;
}

DriveCommand::~DriveCommand() {
  delete leftPid_;
  delete rightPid_;
}
