#ifndef MAIN_H_
#define MAIN_H_

#include "WPILib.h"

class BackboardFinder;
class Constants;
class Drive;
class Logger;
class OperatorControl;
class Pid;
class Intake;
class Shooter;
class Driver;
class TeleopDriver;
class BaselockDriver;
class ShooterController;

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

 private:

  // Constants
  Constants* constants_;

  // Vision
  BackboardFinder* target_;

  // Subsystems
  Drive* drivebase_;
  Intake* intake_;
  Shooter* shooter_;
  ShooterController* sc_;

  // Drivers
  Driver* currDriver_;
  TeleopDriver* teleopDriver_;
  BaselockDriver* baselockDriver_;

  // Joysticks
  Joystick* leftJoystick_;
  Joystick* rightJoystick_;
  OperatorControl* operatorControl_;

  // Motors
  Victor* leftDriveMotorA_;
  Victor* leftDriveMotorB_;
  Victor* rightDriveMotorA_;
  Victor* rightDriveMotorB_;
  Victor* intakeMotor_;
  Victor* conveyorMotor_;
  Victor* leftShooterMotor_;
  Victor* rightShooterMotor_;
  Victor* jumbleMotor_;

  // Sensors
  Encoder* leftEncoder_;
  Encoder* rightEncoder_;
  Encoder* shooterEncoder_;
  Encoder* conveyorEncoder_;
  Gyro* gyro_;
  Accelerometer* accelerometerX_;
  Accelerometer* accelerometerY_;
  Accelerometer* accelerometerZ_;
  AnalogChannel* poofMeter_;
  DigitalInput* bumpSensor_;
  DigitalInput* conveyorBallSensor_;

  // Pneumatics
  Compressor* compressor_;
  Solenoid* shiftSolenoid_;
  Solenoid* hoodSolenoid_;
  DoubleSolenoid* pizzaWheelSolenoid_;
  DoubleSolenoid* intakeSolenoid_;
  DoubleSolenoid* brakeSolenoid_;

  Pid* testPid_;
  Timer* testTimer_;
  Logger* testLogger_;
  DriverStationLCD* lcd_;
  bool oldBaseLockSwitch_;
  double shooterTargetVelocity_;
  double power_;

  // Incremental shooter speed stuff
  bool oldShooterUpSwitch_;
  bool oldShooterDownSwitch_;

  bool oldBallQueueSwitch_;
};

// Start the actual program
START_ROBOT_CLASS(MainRobot);

#endif  // MAIN_H_
