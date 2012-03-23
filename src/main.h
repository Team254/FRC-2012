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
class AutoTurnDriver;
class SequentialCommand;

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
  
  /**
   * Resets motor powers.
   */
  void ResetMotorPower();

 private:

  // Constants
  Constants* constants_;

  // Vision
  BackboardFinder* target_;
  DigitalOutput* ledRingSwitch_;

  // Subsystems
  Drive* drivebase_;
  Intake* intake_;
  Shooter* shooter_;
  ShooterController* sc_;

  // Drivers
  Driver* currDriver_;
  TeleopDriver* teleopDriver_;
  BaselockDriver* baselockDriver_;
  AutoTurnDriver* autoAlignDriver_;

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
  AnalogChannel* conveyorBallSensor_;
  AnalogChannel* ballRanger_;
 
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
  bool oldAutoAlignButton_;
  double shooterTargetVelocity_;
  bool oldShooterSwitch_;
  bool oldIncreaseButton_;
  bool oldDecreaseButton_;
  bool oldAutonSelectButton_;

  bool oldBallQueueSwitch_;

  // Autonomous
  double autonDelay_;
  Timer* autonTimer_;
  enum AutonMode {
    AUTON_NONE = 0,
    AUTON_FENDER,
    AUTON_BRIDGE_SLOW,
    AUTON_BRIDGE_FAST,
    AUTON_ALLIANCE_BRIDGE,
    NUM_AUTON_MODES
  };
  AutonMode autonMode_;
  SequentialCommand* autoBaseCmd_;
};

// Start the actual program
START_ROBOT_CLASS(MainRobot);

#endif  // MAIN_H_
