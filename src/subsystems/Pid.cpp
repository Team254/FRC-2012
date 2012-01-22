#include "subsystems/Pid.h"

Pid::Pid(double kP, double kI, double kD) {
  kP_ = kP;
  kI_ = kI;
  kD_ = kD;
  errorSum_ = 0;
  lastError_ = 0;
}

void Pid::SetGoal(double goal) {
  goal_ = goal;
  errorSum_ = 0.0;
  lastError_ = 0.0;
}

double Pid::Update(double currentValue) {
  double error = goal_ - currentValue;
  double p = kP_ * error;
  errorSum_ += error;
  double i = kI_ * errorSum_;
  double dError = (error - lastError_) / .01;
  double d = kD_ * dError;
  lastError_ = error;

  return (p + i + d);
}

