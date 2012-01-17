#ifndef DRIVE_H_INCLUDED
#define DRIVE_H_INCLUDED

#include "Constants.h"
#include "WPILib.h"

class Drive {
 public:
  Drive(Victor* left, Victor* right);
  void SetLeftDrivePower(double power);
  void SetRightDrivePower(double power);
 private:
  Victor* leftDriveMotors_;
  Victor* rightDriveMotors_;

  Constants* constants;
};


#endif // DRIVE_H_INCLUDED
