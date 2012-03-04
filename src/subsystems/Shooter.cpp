#include "subsystems/Shooter.h"

#include <cmath>

#include "util/PidTuner.h"

Shooter::Shooter(Victor* conveyorMotor, Victor* leftShooterMotor, Victor* rightShooterMotor, Encoder* shooterEncoder,
                 Solenoid* hoodSolenoid, Encoder* conveyorEncoder, DigitalInput* ballSensor) {
  constants_ = Constants::GetInstance();
  conveyorMotor_ = conveyorMotor;
  leftShooterMotor_ = leftShooterMotor;
  rightShooterMotor_ = rightShooterMotor;
  shooterEncoder_ = shooterEncoder;
  hoodSolenoid_ = hoodSolenoid;
  prevEncoderPos_ = shooterEncoder->Get();
  conveyorEncoder_ = conveyorEncoder;
  ballSensor_ = ballSensor;
  targetVelocity_ = 0.0;
  velocity_ = 0.0;
  timer_ = new Timer();
  timer_->Reset();
  timer_->Start();
  pid_ = new Pid(&constants_->shooterKP, &constants_->shooterKI, &constants_->shooterKD);
  conveyorPid_ = new Pid(&constants_->conveyorKP, &constants_->conveyorKI, &constants_->conveyorKD);
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

void Shooter::SetLinearConveyorPower(double pwm) {
  conveyorMotor_->Set(ConveyorLinearize(pwm));
}

void Shooter::SetHoodUp(bool up) {
  hoodSolenoid_->Set(up);
}

void Shooter::SetConveyorTarget(double deltaTicks) {
  conveyorTarget_ += deltaTicks;
}

void Shooter::ResetConveyorTarget() {
  conveyorTarget_ = conveyorEncoder_->Get();
}

bool Shooter::ConveyorPIDUpdate() {
  double currVal = conveyorEncoder_->Get();
  if(fabs(currVal-conveyorTarget_)<constants_->conveyorPIDThreshold) {
    return true;
  }
  SetLinearConveyorPower(conveyorPid_->Update(conveyorTarget_, currVal));
  return false;
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

bool Shooter::QueueBall() {
  // If we have a ball, check that the conveyor has moved a certain distance so we don't double count a ball
  if (ballSensor_->Get() && (ballQ_.empty() || fabs(conveyorEncoder_->Get() - ballQ_.back().pos) >
      constants_->minConveyorBallDist)) {
    // Just got a ball
    ballStats ball = {conveyorEncoder_->Get(), 0.0};
    ballQ_.push_back(ball);
  } else if (ballQ_.empty()) {
    // No ball, go full speed
    SetLinearConveyorPower(1.0);
  }

  // If we have a ball in the queue, increment the PID goal to the top
  if (!ballQ_.empty()) {
    // Set the target to put the top ball at the top minus the buffer
    double distToTop = fabs(constants_->conveyorHeight - (conveyorEncoder_->Get() - ballQ_.front().pos) -
        constants_->conveyorIntakeBuffer);
    if (distToTop > constants_->conveyorPIDThreshold) {
      SetConveyorTarget(constants_->conveyorPIDIncrement);
      return false;
    } else {
      return true;
    }
  }
}

void Shooter::ShootBall() {
  if (!ballQ_.empty()) {
    SetBallShooterTarget(ballQ_.front());
    ballQ_.pop_front();
  }
}

void Shooter::ResetQueue() {
  ballQ_.clear();
}

void Shooter::SetBallShooterTarget(ballStats ball) {
  // do something here...
}
