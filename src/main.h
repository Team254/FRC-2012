#ifndef MAIN_H_
#define MAIN_H_

#include "WPILib.h"

#include "subsystems/ControlBoard.h"
#include "subsystems/Drive.h"

class MainRobot : public IterativeRobot {
 public:
  MainRobot();	
	
  static const double kDefaultPeriod = 5e-3;	/** default period for periodic functions **/
  virtual void StartCompetition();

  virtual void RobotInit();
  virtual void DisabledInit();
  virtual void AutonomousInit();
  virtual void TeleopInit();

  virtual void DisabledPeriodic();
  virtual void AutonomousPeriodic();
  virtual void TeleopPeriodic();

  virtual void DisabledContinuous();
  virtual void AutonomousContinuous();
  virtual void TeleopContinuous();

  void SetPeriod(double period);
  double GetLoopsPerSec();
 private:
  Constants* constants_;
  ControlBoard* controls_;
  Drive* drivebase_;
  Joystick* leftJoystick_;
  Joystick* rightJoystick_;
  Victor* leftDriveMotors_;
  Victor* rightDriveMotors_;
};

#endif  // MAIN_H_
