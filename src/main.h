#ifndef MAIN_H_
#define MAIN_H_

#include "WPILib.h"

#include "subsystems/ControlBoard.h"
#include "subsystems/Drive.h"

class MainRobot : public IterativeRobot {
 public:
  MainRobot();	
	
  virtual void DisabledInit();
  virtual void AutonomousInit();
  virtual void TeleopInit();

  virtual void DisabledPeriodic();
  virtual void AutonomousPeriodic();
  virtual void TeleopPeriodic();

 private:

  Constants* constants_;

  Drive* drivebase_;

  Joystick* leftJoystick_;
  Joystick* rightJoystick_;

  Victor* leftDriveMotorA_;
  Victor* leftDriveMotorB_;
  Victor* rightDriveMotorA_;
  Victor* rightDriveMotorB_;
};

#endif  // MAIN_H_
