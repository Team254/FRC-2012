#include "subsystems/Shooter.h"

#include <cmath>

Shooter::Shooter(Victor* intakeMotor, Victor* conveyorMotor, Victor* leftShooterMotor,
                 Victor* rightShooterMotor, Encoder* shooterEncoder, Solenoid* hoodSolenoid,
                 Solenoid* intakeSolenoid) {
  constants_ = Constants::GetInstance();
  intakeMotor_ =  intakeMotor;
  conveyorMotor_ =  conveyorMotor;
  leftShooterMotor_ =  leftShooterMotor;
  rightShooterMotor_ =  rightShooterMotor;
  shooterEncoder_ =  shooterEncoder;
  hoodSolenoid_ =  hoodSolenoid;
  intakeSolenoid_ =  intakeSolenoid;
  ResetEncoder();
  prevShooterVal_=0.0;
  targetWubbleu_=0.0;
  timer_.Reset();
  timer_.Start();
  pid_=Pid(constants_->shooterKP,constants_->shooterKI,constants_->shooterKD);
}

void Shooter::SetLinearPower(double pwm) {
  SetPower(Linearize(pwm));
}

void Shooter::SetTargetWubbleu(double wubbleu) {
  targetWubbleu_ = wubbleu;
  pid_.ResetError();
}

bool Shooter::PIDUpdate() {
  static const double threshold=1.0;
  double wubbleu=GetShooterWubbleu();
  if(fabs(wubbleu-targetWubbleu_)<threshold) {
      return true;
  }
  pid_.Update(targetWubbleu_,wubbleu);
  return false;
}

void Shooter::SetConveyorPower(double pwm) {
  conveyorMotor_->Set(PwmLimit(pwm));
}

void Shooter::SetIntakePower(double pwm) {
  intakeMotor_->Set(PwmLimit(pwm));
}

void Shooter::SetIntakeUp(bool up) {
  if(up) {
    intakeSolenoid_->Set(DoubleSolenoid::kForward);
  } else {
    intakeSolenoid_->Set(DoubleSolenoid::kReverse);
  }
}

void Shooter::SetHoodUp(bool up) {
  hoodSolenoid_->Set(up);
}

double Shooter::GetShooterWubbleu() {
  double currShooterVal = shooterEncoder_->Get() / 256.0 * 360;
  double returnVal = (currShooterVal - prevShooterVal_) / timer_.Get();
  prevShooterVal_ = currShooterVal;
  timer_.Reset();
  return returnVal;
}

void Shooter::ResetEncoder() {
  shooterEncoder_->Reset();
}

void Shooter::SetPower(double power) {
  leftShooterMotor_->Set(PwmLimit(power));
  rightShooterMotor_->Set(PwmLimit(-power));
}

double Shooter::Linearize(double x) {
  return x; // Do this later
}
