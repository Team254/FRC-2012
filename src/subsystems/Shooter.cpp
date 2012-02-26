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
}

void Shooter::SetLinearPower(double pwm) {
  SetPower(Linearize(pwm));
}

void Shooter::SetTargetVelocity(double velocity) {
  targetVelocity_ = velocity;
  pid_->ResetError();
}

bool Shooter::PIDUpdate() {
  int currEncoderPos = shooterEncoder_->Get();
  velocity_ = (float)(currEncoderPos - prevEncoderPos_) / TICKS_PER_REV / timer_->Get();
  prevEncoderPos_ = currEncoderPos;
  timer_->Reset();

  double signal = pid_->Update(targetVelocity_, velocity_);
  //SetLinearPower(signal);
  SetLinearPower(targetVelocity_ / 100.0);
  PidTuner::PushData(targetVelocity_, velocity_, signal);
  return (fabs(targetVelocity_ - velocity_) < VELOCITY_THRESHOLD);
}

void Shooter::SetConveyorPower(double pwm) {
  conveyorMotor_->Set(PwmLimit(pwm));
}

void Shooter::SetHoodUp(bool up) {
  hoodSolenoid_->Set(up);
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
  if(fabs(x) < 0.01 ) {
      x = 0.0;
  }
  if (x > 0.0) {
    return constants_->linearCoeffA * pow(x, 4) + constants_->linearCoeffB * pow(x, 3) +
        constants_->linearCoeffC * pow(x, 2) + constants_->linearCoeffD * x + constants_->linearCoeffE;
  } else if (x < 0.0) {
    // Rotate the linearization function by 180.0 degrees to handle negative input.
    return -Linearize(-x);
  } else {
    return 0.0;
  }
}
