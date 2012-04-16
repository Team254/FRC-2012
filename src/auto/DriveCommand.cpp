#include "DriverStationLCD.h"
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

bool DriveCommand::Run() {

  if (distanceGoal_ == 0.0  && angleGoal_ == 0.0)
    return true;
  
  if (TimeoutExpired()) {
	drive_->SetLinearPower(0, 0);
	return true;
  }
  drive_->SetHighGear(false);
  double currLeftDist = drive_->GetLeftEncoderDistance() * 0.0254;
  double currRightDist = drive_->GetRightEncoderDistance() * 0.0254;
  
  // Convert from inches to meters and degrees to radians
  double distGoal = distanceGoal_ * 0.0254;
  double maxAcc = maxAcceleration_ * 0.0254;
  double maxVel = maxSpeed_ * 0.0254;
  double angGoal = angleGoal_ * 0.0174532925;
  double maxAlph = maxAlpha_ * 0.0174532925;
  double maxOmeg = maxOmega_ * 0.0174532925;
  
  
  straightFilter_->CalcSystem(distGoal - curX_, curV_, 0.0, maxAcc, maxVel, 0.02);
  curA_ = straightFilter_->GetCurrAcc();
  curV_ = straightFilter_->GetCurrVel();
  curX_ = straightFilter_->GetCurrPos();
  
  turnFilter_->CalcSystem(angGoal - curThet_, curWubl_, 0.0, maxAlph, maxOmeg, 0.02);
  curJeez_ = turnFilter_->GetCurrAcc();
  curWubl_ = turnFilter_->GetCurrVel();
  curThet_ = turnFilter_->GetCurrPos();
  
  static double robotWidth = .827096;
  double wubbleuFactor = curWubl_ * robotWidth / 2;
  double thetaFactor = curThet_ * robotWidth / 2;
  double theta_measured = (currRightDist - currLeftDist) / robotWidth;
  double theta_gyro = -(drive_->GetGyroAngle() - startingAngle_) * 0.0174532925 ;
  double kI = 0.5;
  double KpTurn = 0.3;
  double gyroError = (angGoal + turnOffset_ - theta_measured);
  if (fabs(angGoal - curThet_) < 0.0001 && fabs(gyroError) < 18.0 * 0.0174532925) {
	  double KiTurn = 0.15;
	  sumStoppedError_ += gyroError * KiTurn;
  } else {
	  if (!(fabs(angGoal - curThet_) < 0.0001)) {
		  sumStoppedError_ *= 0.97;
	  }
  }
  
  double doffset = ((theta_measured - turnOffset_) - theta_gyro) * kI;
  if (doffset > 0.05) {
	  doffset = 0.05;
  } else if (doffset < -0.05) {
	  doffset = -0.05;
  }
  turnOffset_ += doffset;
  double turnCompensationOffset = (turnOffset_ + sumStoppedError_ + gyroError * KpTurn) * robotWidth;
  //printf("offset: %f error: %f total: %f\n", turnOffset_, sumStoppedError_, (turnOffset_ + sumStoppedError_) / 2.0);
  flash_matrix(r_, curX_ - turnCompensationOffset / 2.0 - thetaFactor, curV_ - wubbleuFactor, curX_ + turnCompensationOffset / 2.0 + thetaFactor, curV_ + wubbleuFactor);
  flash_matrix(y_, currLeftDist, currRightDist);
  ssc_->update(r_, y_);
  
  //printf("l: %f r: %f g: %f\n", ssc_->U->data[0] / 12.0, ssc_->U->data[1] / 12.0, drive_->GetGyroAngle());
  //PidTuner::PushData((ssc_->X_hat->data[0]-ssc_->X_hat->data[2])/robotWidth, (currLeftDist-currRightDist)/robotWidth, -curThet_);//angGoal*(robotWidth));
  PidTuner::PushData((-(drive_->GetGyroAngle() - startingAngle_ ) - angleGoal_) * 0.0174532925,
		  ((curX_ - turnCompensationOffset / 2.0 - thetaFactor)-(curX_ + turnCompensationOffset / 2.0 + thetaFactor))/robotWidth,
		  (currLeftDist-currRightDist)/robotWidth);//angGoal*(robotWidth));
  //printf("%f %f %f\n",ssc_->X_hat->data[0], currLeftDist, distGoal);
  drive_->SetLinearPower(ssc_->U->data[0] / 12.0, ssc_->U->data[1] / 12.0);
  DriverStationLCD::GetInstance()->PrintfLine(DriverStationLCD::kUser_Line1, "curX:%f", curX_);
  DriverStationLCD::GetInstance()->PrintfLine(DriverStationLCD::kUser_Line2, "stoppedError:%f", sumStoppedError_);
  DriverStationLCD::GetInstance()->PrintfLine(DriverStationLCD::kUser_Line3, "thetaFactor:%f", thetaFactor);
  DriverStationLCD::GetInstance()->PrintfLine(DriverStationLCD::kUser_Line4, "turnOffset:%f", turnOffset_);
  DriverStationLCD::GetInstance()->PrintfLine(DriverStationLCD::kUser_Line5, "compensation:", turnCompensationOffset);
  DriverStationLCD::GetInstance()->PrintfLine(DriverStationLCD::kUser_Line6, "%f", 0);
  DriverStationLCD::GetInstance()->UpdateLCD();
  bool angleDone = fabs(drive_->GetGyroAngle() * 0.01714532925 - angGoal) < 0.5;
  bool leftDone = fabs((distGoal - angGoal * robotWidth / 2.0) - currLeftDist) < 0.03;
  bool rightDone = fabs((distGoal + angGoal * robotWidth / 2.0) - currRightDist) < 0.03;
  
  return TimeoutExpired() || (angleDone && leftDone && rightDone);
  //return false;
 
  
}

DriveCommand::~DriveCommand() {
}
