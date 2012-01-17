#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED

#include "ControlBoard.h"
#include "Drive.h"
#include "WPILib.h"

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

#endif // MAIN_H_INCLUDED
