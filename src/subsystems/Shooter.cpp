#include "subsystems/Shooter.h"

#include <cmath>

#include "util/PidTuner.h"

Shooter::Shooter(Victor* conveyorMotor, Victor* leftShooterMotor, Victor* rightShooterMotor,
                Encoder* shooterEncoder, Solenoid* hoodSolenoid, AnalogChannel* ballSensor,
                AnalogChannel* poofMeter, AnalogChannel* ballRanger) {
  constants_ = Constants::GetInstance();
  conveyorMotor_ = conveyorMotor;
  leftShooterMotor_ = leftShooterMotor;
  rightShooterMotor_ = rightShooterMotor;
  shooterEncoder_ = shooterEncoder;
  hoodSolenoid_ = hoodSolenoid;
  prevEncoderPos_ = shooterEncoder->Get();
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
}

void Shooter::SetLinearPower(double pwm) {
  SetPower(Linearize(pwm));
}

void Shooter::SetTargetVelocity(double velocity) {
  targetVelocity_ = velocity;
  pid_->ResetError();
  outputValue_ = 0;
  if (velocity > 44) {
    SetHoodUp(true);
  } else if (velocity > 0) {
    SetHoodUp(false);
  }

}

bool Shooter::PIDUpdate() {
  int currEncoderPos = shooterEncoder_->Get();
  double instantVelocity = (float)(currEncoderPos - prevEncoderPos_) / TICKS_PER_REV / timer_->Get();
  velocity_ = UpdateFilter(instantVelocity);
  prevEncoderPos_ = currEncoderPos;

  double correctedTargetVelocity_ = targetVelocity_ * poofCorrectionFactor_;
  outputValue_ += pid_->Update(correctedTargetVelocity_, velocity_);
  timer_->Reset();

  double filteredOutput = UpdateOutputFilter(outputValue_);
  SetLinearPower(filteredOutput);
  //double t = GetTime();

  atTarget_ = fabs(correctedTargetVelocity_ - velocity_) < VELOCITY_THRESHOLD;
  //printf("target: %f vel: %f ret: %d\n",correctedTargetVelocity_,velocity_, ret);
  PidTuner::PushData(correctedTargetVelocity_, velocity_, 0.0);
  return atTarget_ == true;
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
