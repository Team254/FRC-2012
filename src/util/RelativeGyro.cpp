#include "util/RelativeGyro.h"

RelativeGyro::RelativeGyro(int port) : Gyro(port) {
}

float RelativeGyro::GetAbsoluteAngle() {
  return this->Gyro::GetAngle();
}

float RelativeGyro::GetAngle() {
  return GetAbsoluteAngle() - offset_;
}

void RelativeGyro::Reset() {
  offset_ = GetAbsoluteAngle();
}

void RelativeGyro::ResetAbsolute() {
  this->Gyro::Reset();
}
