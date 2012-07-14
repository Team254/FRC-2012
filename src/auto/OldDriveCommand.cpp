#include "auto/OldDriveCommand.h"

#include "subsystems/Drive.h"
#include "subsystems/Pid.h"
#include "util/PidTuner.h"
#include "config/Constants.h"
#include "matlab/mat.h"
#include "util/ContinuousAccelFilter.h"

OldDriveCommand::OldDriveCommand(Drive* drive, double distance, double angle, bool usePizza, double timeout, double maxSpeed, double maxAcceleration, double maxAlpha, double maxOmega) {
  SetTimeout(timeout);
  drive_ = drive;
  angleGoal_ = -angle;
  distanceGoal_ = distance;
  usePizza_ = usePizza;
  resetPizza_ =  usePizza;
  maxSpeed_ = maxSpeed;
  maxAcceleration_ = maxAcceleration;
  maxOmega_ = maxOmega;
  maxAlpha_ = maxAlpha;

  Constants* constants = Constants::GetInstance();
  leftPid_ = new Pid(&constants->driveKP, &constants->driveKI, &constants->driveKD);
  rightPid_ = new Pid(&constants->driveKP, &constants->driveKI, &constants->driveKD);
  brakeTimer_ = new Timer();
  ssc_ = new ss_controller(2, 2, 4, ss_controller::DRIVE);
  y_ = init_matrix(2, 1);
  r_ = init_matrix(4, 1);
  straightFilter_ = new ContinuousAccelFilter();
  turnFilter_ = new ContinuousAccelFilter();

  // Old
  driveTimer_ = new Timer();
  prevTime_ = 0;
  prevLeftDist_ = 0;
  prevRightDist_ = 0;
  startingAngle_ = 0;
}

void OldDriveCommand::Initialize() {
  AutoCommand::Initialize();
  drive_->ResetEncoders();
  drive_->SetPizzaWheelDown(usePizza_);
  brakeTimer_->Reset();
  curA_ = 0.0;
  curV_ = 0.0;
  curX_ = 0.0;
  curJeez_ = 0.0;
  curWubl_ = 0.0;
  curThet_ = 0.0;
  turnOffset_ = 0.0;
  sumStoppedError_ = 0.0;
  flash_matrix(y_, 0.0, 0.0);
  flash_matrix(r_, 0.0, 0.0, 0.0, 0.0);
  ssc_->reset();
  delete straightFilter_;
  delete turnFilter_;
  straightFilter_ = new ContinuousAccelFilter();
  turnFilter_ = new ContinuousAccelFilter();

  // Old
  prevTime_ = 0;
  prevLeftDist_ = drive_->GetLeftEncoderDistance();;
  prevRightDist_ = drive_->GetRightEncoderDistance();;
  driveTimer_->Start();
  driveTimer_->Reset();
  startingAngle_ = drive_->GetGyroAngle();
}

bool OldDriveCommand::Run() {
  drive_->SetHighGear(true);
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
  double angleDiff = drive_->GetGyroAngle() - (startingAngle_ + angleGoal_);
  double straightGain = angleDiff * Constants::GetInstance()->straightDriveGain;
  double leftPwr = leftPIDOutput - straightGain;
  double rightPwr = rightPIDOutput + straightGain;

  leftPwr = (leftPwr < -maxSpeed_) ?  -maxSpeed_: (leftPwr > maxSpeed_) ? maxSpeed_ : leftPwr;
  rightPwr = (rightPwr < -maxSpeed_) ?  -maxSpeed_ : (rightPwr > maxSpeed_) ? maxSpeed_ : rightPwr;

  leftPwr -= straightGain;
  rightPwr += straightGain;
  //PidTuner::PushData(currLeftDist, distanceGoal_, 0.0);
  drive_->SetLinearPower(leftPwr, rightPwr);

  if (fabs(currLeftDist - distanceGoal_ ) < 2 || fabs(currRightDist- distanceGoal_) < 2) {
    if (false) {
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

OldDriveCommand::~OldDriveCommand() {
}
