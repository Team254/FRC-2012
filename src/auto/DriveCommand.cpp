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
    double leftPwr = leftPIDOutput - straightGain;
    double rightPwr = rightPIDOutput + straightGain;
    
    leftPwr = (leftPwr < -maxSpeed_) ?  -maxSpeed_: (leftPwr > maxSpeed_) ? maxSpeed_ : leftPwr;
    rightPwr = (rightPwr < -maxSpeed_) ?  -maxSpeed_ : (rightPwr > maxSpeed_) ? maxSpeed_ : rightPwr;
    
    leftPwr -= straightGain;
    rightPwr += straightGain;
    //PidTuner::PushData(currLeftDist, distanceGoal_, 0.0);    
    drive_->SetLinearPower(leftPwr, rightPwr);
    
    if (fabs(currLeftDist - distanceGoal_ ) < 2 || fabs(currRightDist- distanceGoal_) < 2) {
      //if (coast_) {
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
  /*
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
  double angGoal = angleGoal_ * 0.01714532925;
  double maxAlph = maxAlpha_ * 0.01714532925;
  double maxOmeg = maxOmega_ * 0.01714532925;
  
  
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
  double theta_measured = (currRightDist - currLeftDist) / robotWidth;
  double theta_gyro = -drive_->GetGyroAngle() * 0.01714532925 ;
  double kI = 0.017;
  if (fabs(angGoal - curThet_) < 0.0001 && fabs(angGoal + turnOffset_ - theta_measured) < 18.0 * 0.01714532925) {
    double KiTurn = 0.0254;
    sumStoppedError_ += (angGoal + turnOffset_ - theta_measured) * KiTurn;
  } else {
    if (!(fabs(angGoal - curThet_) < 0.0001)) {
      sumStoppedError_ *= 0.97;
    }
  }
  
  double doffset = ((theta_measured - turnOffset_) - theta_gyro) * kI;
  if (doffset > 0.01) {
    doffset = 0.01;
  } else if (doffset < -0.01) {
    doffset = -0.01;
  }
  turnOffset_ += doffset;
  //printf("offset: %f error: %f total: %f\n", turnOffset_, sumStoppedError_, (turnOffset_ + sumStoppedError_) / 2.0);
  flash_matrix(r_, curX_ - (turnOffset_ + sumStoppedError_) / 2.0 - thetaFactor, curV_ - wubbleuFactor, curX_ + (turnOffset_ + sumStoppedError_) / 2.0 + thetaFactor, curV_ + wubbleuFactor);
  flash_matrix(y_, currLeftDist, currRightDist);
  ssc_->update(r_, y_);
  
  //printf("l: %f r: %f g: %f\n", ssc_->U->data[0] / 12.0, ssc_->U->data[1] / 12.0, drive_->GetGyroAngle());
  //PidTuner::PushData(ssc_->X_hat->data[0], currLeftDist, distGoal);
  //printf("%f %f %f\n",ssc_->X_hat->data[0], currLeftDist, distGoal);
  drive_->SetLinearPower(ssc_->U->data[0] / 12.0, ssc_->U->data[1] / 12.0);
  return false;
  bool angleDone = fabs(drive_->GetGyroAngle() * 0.01714532925 - angleGoal_) < 1.0;
  bool leftDone = fabs((distGoal - angGoal * robotWidth / 2.0) - currLeftDist) < 1.0;
  bool rightDone = fabs((distGoal + angGoal * robotWidth / 2.0) - currRightDist) < 1.0;
  
  return TimeoutExpired() || (angleDone && leftDone && rightDone);
  */
  
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
