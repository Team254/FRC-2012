#ifndef SKYFIRE_H_
#define SKYFIRE_H_

#include "WPILib.h"
#include "CheesyRobot.h"

class AutoTurnDriver;
class BackboardFinder;
class BaselockDriver;
class Constants;
class Drive;
class Driver;
class Intake;
class Logger;
class OperatorControl;
class SequentialCommand;
class Shooter;
class TeleopDriver;

/**
 * @author Eric Caldwell
 *
 * The main robot class. Almost all of our logic outside of individual subsystems will be placed in here.
 */
class Skyfire : public CheesyRobot {
 public:
  /**
   * Constructor
   * Initializes all motors, sensors, and control board inputs the program will use
   * Also constructs support subsystem classes
   */
  Skyfire();

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
   * Resets motor powers to zero.
   */
  void ResetMotors();

 private:

  // Constants
  Constants* constants_;

  // Vision
  BackboardFinder* target_;

  // Subsystems
  Drive* drivebase_;
  Intake* intake_;
  Shooter* shooter_;

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
  Victor* intakeMotor1_;
  Victor* intakeMotor2_;
  Victor* intakeMotor3_;
  Victor* conveyorMotor_;
  Victor* leftShooterMotor_;
  Victor* rightShooterMotor_;

  // Sensors
  Encoder* leftEncoder_;
  Encoder* rightEncoder_;
  Encoder* shooterEncoder_;
  Gyro* gyro_;
  AnalogChannel* poofMeter_;
  DigitalInput* bumpSensor_;
  AnalogChannel* conveyorBallSensor_;
  AnalogChannel* ballRanger_;
 
  // Pneumatics
  Compressor* compressor_;
  Solenoid* shiftSolenoid_;
  Solenoid* hoodSolenoid_;
  Solenoid* pizzaWheelSolenoid_;
  Solenoid* intakeUpSolenoid_;
  Solenoid* intakeDownSolenoid_;
  DoubleSolenoid* brakeSolenoid_;

  DriverStationLCD* lcd_;
  bool oldAutoAlignButton_;
  bool oldShooterSwitch_;
  bool oldIncreaseButton_;
  bool oldDecreaseButton_;
  bool oldAutonSelectButton_;
  bool oldHardUpButton_;
  bool oldHardDownButton_;
  double shooterTargetVelocity_;
  double shooterIncr_;

  // Autonomous
  double autonDelay_;
  Timer* autonTimer_;
  enum AutonMode {
    AUTON_NONE = 0,
    AUTON_START_ANYWHERE,
    AUTON_FENDER,
    AUTON_FAR_BRIDGE_SLOW,
    AUTON_SHORT_SIMPLE,
    AUTON_FAR_SIMPLE,
    AUTON_TEST,
    NUM_AUTON_MODES,
    AUTON_CLOSE_BRIDGE_SLOW,
    AUTON_BRIDGE_FAST,
    AUTON_ALLIANCE_BRIDGE
    
  };
  
  enum BallHardness {
	BALL_WTF_SOFT = 0, 
	BALL_SOFT,
	BALL_DEFAULT,
	BALL_HARD,
	BALL_WTF_HARD,
	NUM_BALLS
  };
  
  AutonMode autonMode_;
  BallHardness ballHardness_;
  SequentialCommand* autoBaseCmd_;
  double prevLeftDist_;
  double prevRightDist_;
  double prevTime;
  Timer* timer_;
  Notifier* shooterControl_;
};

// Start the actual program
START_ROBOT_CLASS(Skyfire);

#endif  // SKYFIRE_H_
