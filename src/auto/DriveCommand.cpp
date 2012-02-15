#include "auto/DriveCommand.h"

#include "subsystems/Drive.h"
#include "subsystems/Pid.h"

DriveCommand::DriveCommand(Drive* drive, double distance, bool usePizza) {
  drive_ = drive;
  distanceGoal_ = distance;
  usePizza_ = usePizza;
  resetPizza_ =  (usePizza && drive->GetPizzaUp());

  Constants* constants = Constants::GetInstance();
  leftPid_ = new Pid(constants->driveKP, constants->driveKI, constants->driveKD);
  rightPid_ = new Pid(constants->driveKP, constants->driveKI, constants->driveKD);
}

void DriveCommand::Initialize() {
  drive_->ResetEncoders();
  drive_->SetPizzaWheelDown(usePizza_);
}

bool DriveCommand::Run() {
  double currLeftDist = drive_->GetLeftEncoderDistance();
  double currRightDist = drive_->GetRightEncoderDistance();

  // If the goal has been reached, this command is done.
  if (currLeftDist == distanceGoal_ && currRightDist == distanceGoal_) {
    drive_->SetPizzaWheelDown(!resetPizza_);
    return true;
  }

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
