#include "auto/DriveCommand.h"

#include "subsystems/Drive.h"
#include "subsystems/Pid.h"
#include "util/PidTuner.h"
#include "config/Constants.h"
#include "matlab/mat.h"
#include "util/ContinuousAccelFilter.h"

DriveCommand::DriveCommand(Drive* drive, double distance, double angle, bool usePizza, double timeout, double maxSpeed, double maxAcceleration, double maxAlpha, double maxOmega) {
  SetTimeout(timeout);
  drive_ = drive;
  angleGoal_ = angle;
  distanceGoal_ = distance;
  usePizza_ = usePizza;
  resetPizza_ =  usePizza; //(usePizza && drive->GetPizzaUp());
  maxSpeed_ = maxSpeed;
  maxAcceleration_ = maxAcceleration;
  maxOmega_ = maxOmega;
  maxAlpha_ = maxAlpha;

  Constants* constants = Constants::GetInstance();
  brakeTimer_ = new Timer();
  ssc_ = new ss_controller(2, 2, 4, ss_controller::DRIVE);
  y_ = init_matrix(2, 1);
  r_ = init_matrix(4, 1);
  straightFilter_ = new ContinuousAccelFilter();
  turnFilter_ = new ContinuousAccelFilter();
}

void DriveCommand::Initialize() {
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
  flash_matrix(y_, 0.0, 0.0);
  flash_matrix(r_, 0.0, 0.0, 0.0, 0.0);
  ssc_->reset();
  delete straightFilter_;
  delete turnFilter_;
  straightFilter_ = new ContinuousAccelFilter();
  turnFilter_ = new ContinuousAccelFilter();
}

bool DriveCommand::Run() {
  if (TimeoutExpired()) {
	drive_->SetLinearPower(0, 0);
	return true;
  }
  drive_->SetHighGear(true);
  double currLeftDist = drive_->GetLeftEncoderDistance() / (39.3700787);
  double currRightDist = drive_->GetRightEncoderDistance() / (39.3700787);
  
  // Convert from inches to meters and degrees to radians
  double distGoal = distanceGoal_ * 0.0254;
  double maxAcc = maxAcceleration_ * 0.0254;
  double maxVel = maxSpeed_ * 0.0254;
  double angGoal = angleGoal_ * 1.714532925;
  double maxAlph = maxAlpha_ * 1.714532925;
  double maxOmeg = maxOmega_ * 1.714532925;
  
  
  straightFilter_->CalcSystem(distGoal - curX_, curV_, 0.0, maxAcc, maxVel, 0.02);
  curA_ = straightFilter_->GetCurrAcc();
  curV_ = straightFilter_->GetCurrVel();
  curX_ = straightFilter_->GetCurrPos();
  
  turnFilter_->CalcSystem(angGoal - curThet_, curWubl_, 0.0, maxAlph, maxOmeg, 0.02);
  curJeez_ = turnFilter_->GetCurrAcc();
  curWubl_ = turnFilter_->GetCurrVel();
  curThet_ = turnFilter_->GetCurrPos();
  
  static const double robotWidth = 0.6477;
  double wubbleuFactor = curWubl_ * robotWidth / 2;
  double thetaFactor = curThet_ * robotWidth / 2;
  
  flash_matrix(r_, curX_ - thetaFactor, curV_ - wubbleuFactor, curX_ + thetaFactor, curV_ + wubbleuFactor);
  flash_matrix(y_, currLeftDist, currRightDist);
  ssc_->update(r_, y_);
  
  drive_->SetLinearPower(ssc_->U->data[0] / 12.0, ssc_->U->data[1] / 12.0);
  
  bool angleDone = fabs(drive_->GetGyroAngle() * 1.714532925 - angleGoal_) < 1.0;
  bool leftDone = fabs((distGoal - angGoal * robotWidth / 2.0) - currLeftDist) < 1.0;
  bool rightDone = fabs((distGoal + angGoal * robotWidth / 2.0) - currRightDist) < 1.0;
  
  return TimeoutExpired() || (angleDone && leftDone && rightDone);
  
/*
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
  */
}

DriveCommand::~DriveCommand() {
}
