#include "auto/DriveCommand.h"

#include "subsystems/Drive.h"
#include "subsystems/Pid.h"
#include "util/PidTuner.h"
#include "config/Constants.h"

DriveCommand::DriveCommand(Drive* drive, double distance, bool usePizza) : leftFilter(4), rightFilter(4) {
  drive_ = drive;
  distanceGoal_ = distance;
  usePizza_ = usePizza;
  resetPizza_ =  usePizza; //(usePizza && drive->GetPizzaUp());
  oldLeftDist_ = 0;
  oldRightDist_ = 0;

  Constants* constants = Constants::GetInstance();
  leftPid_ = new Pid(&constants->driveKP, &constants->driveKI, &constants->driveKD);
  rightPid_ = new Pid(&constants->driveKP, &constants->driveKI, &constants->driveKD);
}

void DriveCommand::Initialize() {
  drive_->ResetEncoders();
  drive_->SetPizzaWheelDown(usePizza_);
}

bool DriveCommand::Run() {

  double currLeftDist = drive_->GetLeftEncoderDistance();
  double currRightDist = drive_->GetRightEncoderDistance();
  double velL = leftFilter.Update(currLeftDist - oldLeftDist_ );
  double velR = rightFilter.Update(currRightDist - oldRightDist_);
  oldLeftDist_ = currLeftDist;
  oldRightDist_ = currRightDist;
  double offset = currLeftDist - currRightDist;
  
  // If the goal has been reached, this command is done.
  if (fabs(currLeftDist - distanceGoal_ ) < 3 && fabs(currRightDist- distanceGoal_) < 3 && fabs(velL) < .1) {
    drive_->SetPizzaWheelDown(resetPizza_);
    drive_->SetLinearPower(0, 0);
    return true;
  }
  // Get PID feedback and send back to the motors.
  /*
  static const double gainCap = 0.3;
  if(straightGain>gainCap) {
	  straightGain=gainCap;
  } else if(straightGain<-gainCap) {
	  straightGain=-gainCap;
  }
  */
  double leftPIDOutput = PwmLimit(leftPid_->Update(distanceGoal_, currLeftDist));
  double rightPIDOutput = PwmLimit(rightPid_->Update(distanceGoal_, currRightDist));
  double straightGain = Constants::GetInstance()->straightDriveGain * offset * fabs(leftPIDOutput);
  double leftPwr = leftPIDOutput - straightGain;
  double rightPwr = rightPIDOutput + straightGain;
  drive_->SetLinearPower(leftPwr, rightPwr);
  printf("l: %f, r: %f, delta: %f, power: %f\n", velL, velR, offset, straightGain);
  PidTuner::PushData(leftPwr, straightGain, offset );

  // Indicate that the goal has not yet been reached.
  return false;
}

DriveCommand::~DriveCommand() {
  delete leftPid_;
  delete rightPid_;
}
