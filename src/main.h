#ifndef MAIN_H_
#define MAIN_H_

#include "WPILib.h"

class BackboardFinder;
class Constants;
class Drive;
class Logger;
class OperatorControl;
class Pid;

/**
 * @author Eric Caldwell
 *
 * The main robot class. Almost all of our logic outside of individual subsystems will be placed in here.
 */
class MainRobot : public IterativeRobot {
 public:
  /**
   * Constructor
   * Initializes all motors, sensors, and control board inputs the program will use
   * Also constructs support subsystem classes
   */
  MainRobot();

  /**
   * Runs once when the robot enters Disabled mode
   */
  virtual void DisabledInit();

  /**
   * Runs once when the robot enters Autonomous mode
   */
  virtual void AutonomousInit();

  /**
   * Runs once when the robot enters Teleop mode
   */
  virtual void TeleopInit();

  /**
   * Is called periodically as long as the robot is in Disabled mode
   */
  virtual void DisabledPeriodic();

  /**
   * Is called periodically as long as the robot is in Autonomous mode
   */
  virtual void AutonomousPeriodic();

  /**
   * Is called periodically as long as the robot is in Teleop mode
   */
  virtual void TeleopPeriodic();

  // Helper functions

  /**
   * Implements a deadband on a joystick
   * @param val the joystick value
   * @param deadband the maximum value the deadband will return 0 for
   * @return 0.0 if the absolute value of the joystick value is less than the deadband, else the joystick value
   */
  double HandleDeadband(double val, double deadband);

 private:

  // Constants
  Constants* constants_;

  // Vision
  BackboardFinder* target_;

  // Subsystems
  Drive* drivebase_;

  // Joysticks
  Joystick* leftJoystick_;
  Joystick* rightJoystick_;
  OperatorControl* operatorControl_;

  // Motors
  Victor* leftDriveMotorA_;
  Victor* leftDriveMotorB_;
  Victor* rightDriveMotorA_;
  Victor* rightDriveMotorB_;

  // Sensors
  Encoder* leftEncoder_;
  Encoder* rightEncoder_;
  Gyro* gyro_;
  
  // Pneumatics
  Compressor* compressor_;
  Solenoid* shiftSolenoid_;

  Pid* testPid_;
  Pid* baseLockPid_;
  double baseLockPosition_;
  bool oldBaseLockSwitch_;
  Timer* testTimer_;
  Logger* testLogger_;
};

// Start the actual program
START_ROBOT_CLASS(MainRobot);

#endif  // MAIN_H_
