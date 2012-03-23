#include "auto/DriveCommand.h"

#include "subsystems/Drive.h"
#include "subsystems/Pid.h"
#include "util/PidTuner.h"
#include "config/Constants.h"

DriveCommand::DriveCommand(Drive* drive, double distance, bool usePizza) {
  drive_ = drive;
  distanceGoal_ = distance;
  usePizza_ = usePizza;
  resetPizza_ =  usePizza; //(usePizza && drive->GetPizzaUp());

  Constants* constants = Constants::GetInstance();
  leftPid_ = new Pid(&constants->driveKP, &constants->driveKI, &constants->driveKD);
  rightPid_ = new Pid(&constants->driveKP, &constants->driveKI, &constants->driveKD);
}

void DriveCommand::Initialize() {
  drive_->ResetEncoders();
  drive_->SetPizzaWheelDown(usePizza_);
  startingAngle_ = drive_->GetGyroAngle();
}

bool DriveCommand::Run() {
  double currLeftDist = drive_->GetLeftEncoderDistance();
  double currRightDist = drive_->GetRightEncoderDistance();
  
  // If the goal has been reached, this command is done.
  if (fabs(currLeftDist - distanceGoal_ ) < 3 || fabs(currRightDist- distanceGoal_) < 3) {
    drive_->SetPizzaWheelDown(resetPizza_);
    drive_->SetLinearPower(0, 0);
    return true;
  }

  // Get PID feedback and send back to the motors.
  double leftPIDOutput = PwmLimit(leftPid_->Update(distanceGoal_, currLeftDist));
  double rightPIDOutput = PwmLimit(rightPid_->Update(distanceGoal_, currRightDist));
  double angleDiff = drive_->GetGyroAngle() - startingAngle_;
  double straightGain = angleDiff * Constants::GetInstance()->straightDriveGain;
  double leftPwr = leftPIDOutput - straightGain;
  double rightPwr = rightPIDOutput + straightGain;
  drive_->SetLinearPower(leftPwr, rightPwr);
  PidTuner::PushData(leftPwr, straightGain, 0);

  // Indicate that the goal has not yet been reached.
  return false;
}

DriveCommand::~DriveCommand() {
  delete leftPid_;
  delete rightPid_;
}
