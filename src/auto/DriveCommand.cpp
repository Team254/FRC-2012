#include "auto/DriveCommand.h"

#include "subsystems/Drive.h"
#include "subsystems/Pid.h"
#include "util/PidTuner.h"

DriveCommand::DriveCommand(Drive* drive, double distance, bool usePizza) {
  drive_ = drive;
  distanceGoal_ = distance;
  usePizza_ = usePizza;
  resetPizza_ =  usePizza; //(usePizza && drive->GetPizzaUp());
  oldLeftDist_ = 0;

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
  double vel = currLeftDist - oldLeftDist_ ;
  oldLeftDist_ = currLeftDist;
  // If the goal has been reached, this command is done.
  if (fabs(currLeftDist - distanceGoal_ ) < 3 && fabs(currRightDist- distanceGoal_) < 3 && fabs(vel) < .1) {
    drive_->SetPizzaWheelDown(resetPizza_);
    drive_->SetLinearPower(0, 0);
    return true;
  }
  printf("cur: %f, r: %f, goal: %f\n", currLeftDist, currRightDist, distanceGoal_);
  PidTuner::PushData(currLeftDist, distanceGoal_, currRightDist);
  // Get PID feedback and send back to the motors.
  double leftPwr = leftPid_->Update(distanceGoal_, currLeftDist);
  double rightPwr = rightPid_->Update(distanceGoal_, currRightDist);
  drive_->SetLinearPower(leftPwr, rightPwr);

  // Indicate that the goal has not yet been reached.
  return false;
}

DriveCommand::~DriveCommand() {
  delete leftPid_;
  delete rightPid_;
}
