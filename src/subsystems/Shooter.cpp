#include "subsystems/Shooter.h"

#include <cmath>

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
  timer_ = new Timer();
  timer_->Reset();
  timer_->Start();
  pid_ = new Pid(constants_->shooterKP, constants_->shooterKI, constants_->shooterKD);
}

void Shooter::SetLinearPower(double pwm) {
  SetPower(Linearize(pwm));
}

void Shooter::SetTargetVelocity(double velocity) {
  targetVelocity_ = velocity;
  pid_->ResetError();
}

bool Shooter::PIDUpdate() {
  double velocity = GetVelocity();
  SetLinearPower(pid_->Update(targetVelocity_, GetVelocity()));
  return (fabs(targetVelocity_ - velocity) < VELOCITY_THRESHOLD);
}

void Shooter::SetConveyorPower(double pwm) {
  conveyorMotor_->Set(PwmLimit(pwm));
}

void Shooter::SetHoodUp(bool up) {
  hoodSolenoid_->Set(up);
}

double Shooter::GetVelocity() {
  int currEncoderPos = shooterEncoder_->Get();
  double velocity = (float)(currEncoderPos - prevEncoderPos_) / TICKS_PER_REV / timer_->Get();
  prevEncoderPos_ = currEncoderPos;
  timer_->Reset();
  return velocity;
}

void Shooter::SetPower(double power) {
  leftShooterMotor_->Set(PwmLimit(-power));
  rightShooterMotor_->Set(PwmLimit(-power));
}

double Shooter::Linearize(double x) {
  return x; // Do this later
}
