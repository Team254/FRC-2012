#include "auto/DriveCommand.h"

#include "subsystems/Drive.h"
#include "subsystems/Pid.h"
#include "util/PidTuner.h"
#include "config/Constants.h"

DriveCommand::DriveCommand(Drive* drive, double distance, bool coast, bool usePizza, double timeout, double maxSpeed) {
  SetTimeout(timeout);
  drive_ = drive;
  distanceGoal_ = distance;
  usePizza_ = usePizza;
  resetPizza_ =  usePizza; //(usePizza && drive->GetPizzaUp());
  coast_ = coast;
  maxSpeed_ = maxSpeed;

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
  drive_->SetHighGear(true);
  double currLeftDist = drive_->GetLeftEncoderDistance();
  double currRightDist = drive_->GetRightEncoderDistance();
  double currTime = driveTimer_->Get();
  double lVel = (currLeftDist - prevLeftDist_)/(currTime - prevTime_);
  double rVel = (currRightDist - prevRightDist_)/(currTime - prevTime_);
  prevTime_ = currTime;
  prevLeftDist_ = currLeftDist;
  prevRightDist_ = currRightDist;
  

  // Get PID feedback and send back to the motors.
  double leftPIDOutput = PwmLimit(leftPid_->Update(distanceGoal_, currLeftDist));
  double rightPIDOutput = PwmLimit(rightPid_->Update(distanceGoal_, currRightDist));
  double angleDiff = drive_->GetGyroAngle() - startingAngle_;
  double straightGain = angleDiff * Constants::GetInstance()->straightDriveGain;
  double leftPwr = leftPIDOutput; - straightGain;
  double rightPwr = rightPIDOutput; + straightGain;
  
  leftPwr = (leftPwr < -maxSpeed_) ?  -maxSpeed_: (leftPwr > maxSpeed_) ? maxSpeed_ : leftPwr;
  rightPwr = (rightPwr < -maxSpeed_) ?  -maxSpeed_ : (rightPwr > maxSpeed_) ? maxSpeed_ : rightPwr;
  
  leftPwr -= straightGain;
  rightPwr += straightGain;
  //PidTuner::PushData(currLeftDist, distanceGoal_, 0.0);    
  drive_->SetLinearPower(leftPwr, rightPwr);
  
  if (fabs(currLeftDist - distanceGoal_ ) < 2 || fabs(currRightDist- distanceGoal_) < 2) {
	  if (coast_) {
	    drive_->SetLinearPower(0,0);
	    return true;
	   }
	  if (fabs(lVel) < 6 && fabs(rVel) < 6) {
	    brakeTimer_->Start();
	  }

    }
  
  if (brakeTimer_->Get() > .2) {
	  drive_->SetPizzaWheelDown(resetPizza_);
	  drive_->SetLinearPower(0,0);
	  return true;
  }
  // Indicate that the goal has not yet been reached.
  return false;
}

DriveCommand::~DriveCommand() {
  delete leftPid_;
  delete rightPid_;
}
