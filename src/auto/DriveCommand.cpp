#include "auto/DriveCommand.h"

DriveCommand::DriveCommand(Drive* drive, double distance) {
  drive_ = drive;
  distanceGoal_ = distance;
  // set constants for tuning here
  leftPid_ = new Pid(.1, 0, 0);
  //leftPid_->SetGoal(distanceGoal_);
  rightPid_ = new Pid(.1, 0, 0);
  //rightPid_->SetGoal(distanceGoal_);
  distanceGoal_ = distance;
}

void DriveCommand::Initialize() {
  drive_->ResetEncoders();
}

bool DriveCommand::Run() {
  double currLeftDist = drive_->GetLeftEncoderDistance();
  double currRightDist = drive_->GetRightEncoderDistance();
  // if goal reached, return true so current command can be popped off queue
  if(currLeftDist == distanceGoal_ && currRightDist == distanceGoal_) {
    return true;
  }
  // get PID feedback and send back to motors
  double leftPwr = leftPid_->Update(distanceGoal_, currLeftDist);
  double rightPwr = rightPid_->Update(distanceGoal_, currRightDist);
  drive_->SetLinearPower(leftPwr, rightPwr);
  // indicates that goal is not reached
  return false;
}

DriveCommand::~DriveCommand() {
  delete leftPid_;
  delete rightPid_;
  delete drive_;
}
