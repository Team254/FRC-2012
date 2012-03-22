#include "subsystems/Shooter.h"

#include <cmath>

#include "util/PidTuner.h"

Shooter::Shooter(Victor* conveyorMotor, Victor* leftShooterMotor, Victor* rightShooterMotor, Encoder* shooterEncoder,
                 Solenoid* hoodSolenoid, Encoder* conveyorEncoder, AnalogChannel* ballSensor, AnalogChannel* poofMeter,
                 AnalogChannel* ballRanger) {
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
  } else {
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

  bool ret = fabs(correctedTargetVelocity_ - velocity_) < VELOCITY_THRESHOLD;
  printf("target: %f vel: %f ret: %d\n",correctedTargetVelocity_,velocity_, ret);
  PidTuner::PushData(correctedTargetVelocity_, velocity_, (ret) ? 50.0 : 0.0);
  return ret == true;
}

void Shooter::SetLinearConveyorPower(double pwm) {
  conveyorMotor_->Set(pwm);
}

void Shooter::SetHoodUp(bool up) {
  hoodSolenoid_->Set(up);
}

void Shooter::SetConveyorTarget(int target) {
  conveyorTarget_ = target;
}

void Shooter::ConveyorPIDUpdate() {
  SetLinearConveyorPower(conveyorPid_->Update(conveyorTarget_, conveyorEncoder_->Get()));
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

bool Shooter::QueueBall() {
  // If we have a ball, check that the conveyor has moved a certain distance so we don't double count a ball
  if (ballSensor_->GetValue() > 100 && !prevBallSensor_ &&
      (ballQ_.empty() || fabs(conveyorEncoder_->Get() - ballQ_.back().position) >
          constants_->minConveyorBallDist)) {
    // Just got a ball
    ballStats ball = {conveyorEncoder_->Get(), 0};
    ballQ_.push_back(ball);
  }

  bool returnValue = false;
#if 0 // Pat's old logic, maybe pull this back in later
  if (ballQ_.empty()) {
    // No ball, go full speed
    SetLinearConveyorPower(1.0);
  } else {
    // Set the target to put the top ball at the top minus the buffer
    int distToTop = ballQ_.front().position + (int)constants_->conveyorHeight - conveyorEncoder_->Get();
    if (distToTop < (int)constants_->conveyorPIDThreshold) {
      SetLinearConveyorPower(0.0);
      returnValue= true;
    } else {
      conveyorTarget_ = ballQ_.front().position + (int)constants_->conveyorHeight;
      ConveyorPIDUpdate(); //+= (int)constants_->conveyorPIDIncrement;
    }
  }
#endif

  double ballRange = (double) ballRanger_->GetValue();
  PidTuner::PushData(ballRange, conveyorEncoder_->Get(), 0.0);
  double val = conveyorPid_->Update(200.0, ballRange);
  //Conveyor was jamming low down
  if (ballRange < 100) {
    val = 1;
  }
  SetLinearConveyorPower(val);
  prevBallSensor_ = ballSensor_->GetValue() > 100;

  // Update the poofometer readings if a ball is within the sensor's window.
  for (std::deque<ballStats>::iterator iter = ballQ_.begin(); iter != ballQ_.end(); ++iter) {
    int ballPosition = conveyorEncoder_->Get() - iter->position;
    if (ballPosition > (int)constants_->conveyorPoofWindowLow &&
        ballPosition < (int)constants_->conveyorPoofWindowHigh) {
      // Record the peak sensor value for that ball.
      int poofiness = poofMeter_->GetValue();
      if (poofiness > iter->poofiness) {
        iter->poofiness = poofiness;
      }

      // Only one ball should ever be within the window, so we can exit early.
      break;
    }
  }

  return returnValue;
}

void Shooter::ShootBall() {
  if (!ballQ_.empty()) {
// TODO(pat): Uncomment once we want to do linear interpolation for correction.
//    SetBallShooterTarget(ballQ_.front());
    ballQ_.pop_front();
  }
}

void Shooter::ResetQueueState() {
  ballQ_.clear();
}

void Shooter::SetBallShooterTarget(ballStats ball) {
  // Do linear interpolation with the poofiness to correct for ball variance.
  poofCorrectionFactor_ = (ball.poofiness - constants_->poofometerLowPoofiness) /
      (constants_->poofometerHighPoofiness - constants_->poofometerLowPoofiness) *
      (constants_->poofometerHighCorrection - constants_->poofometerLowCorrection) +
      constants_->poofometerLowCorrection;
}

void Shooter::DebugBallQueue() {
  int i = 0;
  for (std::deque<ballStats>::const_iterator iter = ballQ_.begin(); iter != ballQ_.end(); ++iter) {
    printf("Ball %d pos: %d poof: %d\n", i, iter->position, iter->poofiness);
    i++;
  }
}
