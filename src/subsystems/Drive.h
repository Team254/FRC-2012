#ifndef SUBSYSTEMS_DRIVE_H_
#define SUBSYSTEMS_DRIVE_H_

#include "WPILib.h"

#include "config/Constants.h"

/**
 * @author Eric Caldwell
 * 
 * Easy-access functions for drive functions: setting power, getting encoder values, etc.
 */
class Drive {
 public:
  /**
   * Constructor
   * Accepts the Victors and Encoders to get and set values
   */
  Drive(Victor* leftVictorA, Victor* leftVictorB, Victor* rightVictorA, Victor* rightVictorB, Encoder* leftEncoder);

  /**
   * Sets power to the left and right sides of the drivetrain
   * @param left left power
   * @param right right power
   */
  void SetPower(double left, double right);

  /**
   * Sets the power to the left wheel
   * Automatically caps the power to +1.0 or -1.0
   * @param power the power to set
   */
  void SetLeftDrivePower(double power);

  /**
   * Sets the power to the left wheel
   * Automatically caps the power to +1.0 or -1.0
   * @param power the power to set
   */
  void SetRightDrivePower(double power);

  /**
   * Gets the distance travelled by the left side of the robot in meters
   * Calculated via the wheel circumference, gear ratio, and encoder return value
   * @param power the power to set
   */
  double GetLeftEncoderDistance();

 private:

  // Victors

  Victor* leftDriveMotorA_;
  Victor* leftDriveMotorB_;
  Victor* rightDriveMotorA_;
  Victor* rightDriveMotorB_;

  // Sensors
  Encoder* leftDriveEncoder_;

  Constants* constants;
};


#endif  // SUBSYSTEMS_DRIVE_H_
