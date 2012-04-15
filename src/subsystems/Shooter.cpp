#include "subsystems/Shooter.h"

#include <cmath>
#include "utils.hpp"

#include "util/PidTuner.h"
#include "util/Logger.h"

Shooter::Shooter(Victor* conveyorMotor, Victor* leftShooterMotor, Victor* rightShooterMotor,
                Encoder* shooterEncoder, Solenoid* hoodSolenoid, AnalogChannel* ballSensor,
                AnalogChannel* poofMeter, AnalogChannel* ballRanger) {
  constants_ = Constants::GetInstance();
  conveyorMotor_ = conveyorMotor;
  leftShooterMotor_ = leftShooterMotor;
  rightShooterMotor_ = rightShooterMotor;
  shooterEncoder_ = shooterEncoder;
  hoodSolenoid_ = hoodSolenoid;
  prevPos_ = 0;
  ballSensor_ = ballSensor;
  targetVelocity_ = 0.0;
  velocity_ = 0.0;
  timer_ = new Timer();
  timer_->Reset();
  timer_->Start();
  pid_ = new Pid(&constants_->shooterKP, &constants_->shooterKI, &constants_->shooterKD);
  for (int i = 0; i < FILTER_SIZE; i++) {
    velocityFilter_[i] = 0;
  }
  filterIndex_ = 0;
  outputValue_ = 0;
  for (int i = 0; i < OUTPUT_FILTER_SIZE; i++) {
    outputFilter_[i] = 0;
  }
  outputFilterIndex_ = 0;
  poofMeter_ = poofMeter;
  poofCorrectionFactor_ = 1.0;
  prevBallSensor_ = false;
  ballRanger_ = ballRanger;
  filter_ = DaisyFilter::SinglePoleIIRFilter(0.5f);
  pidGoal_ = 0.0;
  atTarget_ = false;
  
  y_ = init_matrix(1,1);
    r_ = init_matrix(2,1);
    flash_matrix(y_, 0.0);
    flash_matrix(r_, 0.0, 0.0);
    ssc_ = new ss_controller(1, 1, 2, ss_controller::SHOOTER);
    ssc_->reset();
    
  hardnessOffset_ = 0;  
}

void Shooter::SetLinearPower(double pwm) {
  SetPower(Linearize(pwm));
}

void Shooter::SetTargetVelocity(double velocity, hoodPref pref) {
  if (velocity > 0.0) {
		targetVelocity_ = velocity + hardnessOffset_;
  } else {
	  targetVelocity_ = 0.0;
  }
  pid_->ResetError();
  outputValue_ = 0;
  
  if(pref==UP) {
    SetHoodUp(true);
  } else if (pref==DOWN) {
    SetHoodUp(false);
  } else {
	  if (velocity > 40) {
		SetHoodUp(true);
	  } else if (velocity > 0) {
		SetHoodUp(false);
	  }
  }
}

bool Shooter::PIDUpdate() {
  double dt = timer_->Get();
  timer_->Reset();

  double currEncoderPos = shooterEncoder_->GetRaw() / 128.0 * 2 * 3.1415926;
  
  double velocity_goal = 2 * 3.1415926 * targetVelocity_;
  double instantVelocity = ((currEncoderPos - prevPos_) /  (1.0/50.0));
  
  flash_matrix(y_, (double)currEncoderPos);
  const double velocity_weight_scalar = 0.35;

  double u_min = ssc_->U_min->data[0];
  double u_max = ssc_->U_max->data[0];
  double x_hat1 = ssc_->X_hat->data[1];
  double k1 = ssc_->K->data[1];
  double k0 = ssc_->K->data[0];
  double x_hat0 = ssc_->X_hat->data[0];
  const double max_reference = (u_max - velocity_weight_scalar * (velocity_goal - x_hat1) * k1) / k0 + x_hat0;
  const double min_reference = (u_min - velocity_weight_scalar * (velocity_goal - x_hat1) * k1) / k0 + x_hat0;
  double minimum = (pidGoal_ < max_reference) ? pidGoal_ : max_reference;
  pidGoal_ = (minimum > min_reference) ? minimum : min_reference;

  flash_matrix(r_, pidGoal_, velocity_goal);
  pidGoal_ += ((1.0/50.0) * velocity_goal);
  ssc_->update(r_, y_);
  
  if (velocity_goal < 1.0) {
    SetLinearPower(0.0);
    pidGoal_ = currEncoderPos;
  } else {
    SetLinearPower(ssc_->U->data[0] / 12.0);
  }

  instantVelocity =  instantVelocity / (2 * 3.1415926);
  velocity_ = UpdateFilter(instantVelocity);

  prevPos_ = currEncoderPos;  
  atTarget_ = fabs(velocity_ - targetVelocity_) < VELOCITY_THRESHOLD;
  DriverStationLCD::GetInstance()->PrintfLine(DriverStationLCD::kUser_Line2, "%.1f | %.1f rps", targetVelocity_,
                     velocity_);
  DriverStationLCD::GetInstance()->UpdateLCD();
  return atTarget_;
}

void Shooter::SetLinearConveyorPower(double pwm) {
  conveyorMotor_->Set(pwm);
}

bool Shooter::AtTargetVelocity() {
  return atTarget_;
}

void Shooter::SetHoodUp(bool up) {
  hoodSolenoid_->Set(up);
}

double Shooter::UpdateFilter(double value) {
  //return value;
  velocityFilter_[filterIndex_] = value;
  filterIndex_++;
  if (filterIndex_ == FILTER_SIZE) {
    filterIndex_ = 0;
  }
  double sum = 0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    sum += velocityFilter_[i];
  }
  return sum / (double)FILTER_SIZE;
}

double Shooter::UpdateOutputFilter(double value) {
  outputFilter_[outputFilterIndex_] = value;
  outputFilterIndex_++;
  if (outputFilterIndex_ == OUTPUT_FILTER_SIZE) {
    outputFilterIndex_ = 0;
  }
  double sum = 0;
  for (int i = 0; i < OUTPUT_FILTER_SIZE; i++) {
    sum += outputFilter_[i];
  }
  return sum / (double)OUTPUT_FILTER_SIZE;
}

double Shooter::GetVelocity() {
  return velocity_;
}

void Shooter::SetPower(double power) {
  // The shooter should only ever spin in one direction.
  if (power < 0 || targetVelocity_ == 0) {
    power = 0;
  }
  leftShooterMotor_->Set(PwmLimit(-power));
  rightShooterMotor_->Set(PwmLimit(-power));
}

double Shooter::Linearize(double x) {
  if (x >= 0.0) {
    return constants_->shooterCoeffA * pow(x, 4) + constants_->shooterCoeffB * pow(x, 3) +
        constants_->shooterCoeffC * pow(x, 2) + constants_->shooterCoeffD * x;
  } else {
    // Rotate the linearization function by 180.0 degrees to handle negative input.
    return -Linearize(-x);
  }
}

void Shooter::Reset() {
  atTarget_ = false;
  for (int i = 0; i < FILTER_SIZE; i++) {
  velocityFilter_[i] = 0;
  }
  flash_matrix(y_, 0.0);
  flash_matrix(r_, 0.0, 0.0);
  ssc_->reset();
  timer_->Reset();
  shooterEncoder_->Reset();
  prevPos_ = shooterEncoder_->Get();
}

double Shooter::ConveyorLinearize(double x) {
  if (fabs(x) < 0.01 ) {
    return 0;
  } else if (x > 0) {
    return constants_->conveyorCoeffA * pow(x, 3) + constants_->conveyorCoeffB * pow(x, 2) +
        constants_->conveyorCoeffC * x + constants_->conveyorCoeffD;
  } else {
    // Rotate the linearization function by 180.0 degrees to handle negative input.
    return -Linearize(-x);
  }
}

double Shooter::GetBallRange() {
  return ballRanger_->GetValue();
}

double Shooter::GetTargetVelocity() {
  return targetVelocity_;
}

void Shooter::CallUpdate(void* shooter){
  Shooter* s = (Shooter*) shooter;
  s->PIDUpdate();
}

double Shooter::SetHardnessOffset(double offset) {
  hardnessOffset_ = offset;	
}

