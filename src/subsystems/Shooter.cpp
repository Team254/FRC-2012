#include "subsystems/Shooter.h"

#include <cmath>

#include "util/PidTuner.h"

Shooter::Shooter(Victor* conveyorMotor, Victor* leftShooterMotor, Victor* rightShooterMotor,
                 Encoder* shooterEncoder, Solenoid* hoodSolenoid) {
  constants_ = Constants::GetInstance();
  conveyorMotor_ = conveyorMotor;
  leftShooterMotor_ = leftShooterMotor;
  rightShooterMotor_ = rightShooterMotor;
  shooterEncoder_ = shooterEncoder;
  hoodSolenoid_ = hoodSolenoid;
  prevEncoderPos_ = shooterEncoder->Get();
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
}

void Shooter::SetLinearPower(double pwm) {
  SetPower(Linearize(pwm));
}

void Shooter::SetTargetVelocity(double velocity) {
  targetVelocity_ = velocity;
  pid_->ResetError();
  outputValue_ = 0;
}

bool Shooter::PIDUpdate() {
  int currEncoderPos = shooterEncoder_->Get();
  double instantVelocity = (float)(currEncoderPos - prevEncoderPos_) / TICKS_PER_REV / timer_->Get();
  velocity_ = UpdateFilter(instantVelocity);
  prevEncoderPos_ = currEncoderPos;

  outputValue_ += pid_->Update(targetVelocity_, velocity_);
  timer_->Reset();

  double filteredOutput = UpdateOutputFilter(outputValue_);
  SetLinearPower(filteredOutput);
  return (fabs(targetVelocity_ - velocity_) < VELOCITY_THRESHOLD);
}

void Shooter::SetConveyorPower(double pwm) {
  conveyorMotor_->Set(PwmLimit(pwm));
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
  if (power < 0) {
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
