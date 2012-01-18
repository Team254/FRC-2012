#ifndef SUBSYSTEMS_DRIVE_H_
#define SUBSYSTEMS_DRIVE_H_

#include "WPILib.h"

#include "config/Constants.h"

class Drive {
 public:
  Drive(Victor* leftA, Victor* leftB, Victor* rightA, Victor* rightB);
  void SetLeftDrivePower(double power);
  void SetRightDrivePower(double power);
 private:
  Victor* leftDriveMotorA_;
  Victor* leftDriveMotorB_;
  Victor* rightDriveMotorA_;
  Victor* rightDriveMotorB_;

  Constants* constants;
};


#endif  // SUBSYSTEMS_DRIVE_H_
