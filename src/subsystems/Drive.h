#ifndef SUBSYSTEMS_DRIVE_H_
#define SUBSYSTEMS_DRIVE_H_

#include "WPILib.h"

#include "config/Constants.h"

/**
 * @author Eric Caldwell
 * @author Bhargava Manja
 * @author Art Kalb
 * Easy-access functions for drive functions: setting power, getting encoder values, etc.
 */
class Drive {
 public:
  /**
   * Constructor
   * Accepts the Victors and Encoders to get and set values
   */
  Drive(Victor* leftVictorA, Victor* leftVictorB, Victor* rightVictorA, Victor* rightVictorB,
        Encoder* leftEncoder, Encoder* rightEncoder_, Gyro* gyro);

  /**
   * Sets power to the left and right sides of the drivetrain
   * @param left left power
   * @param right right power
   */
  void SetPower(double left, double right);

  /**
   * Makes drive power linear to input, then sets power to the respective side of the drivetrain
   * @param left left power
   * @param right right power
   */
  void SetLinearPower(double left, double right);


  /**
   * Gets the distance travelled by the left side of the robot in meters
   * Calculated via the wheel circumference, gear ratio, and encoder return value
   * @param power the power to set
   */
  double GetLeftEncoderDistance();

  /**
   * Gets the distance travelled by the right side of the robot in meters
   * Calculated via the wheel circumference, gear ratio, and encoder return value
   */
  double GetRightEncoderDistance();

  /**
   * Returns current gyro angle. Gyro is reset after instance is created
   */
  double GetGyroAngle();

  /**
   * Sets gyro's sensitivity. Input is in V/sec, so remember to divide argument
   * by 1000 if spec sheet says mV, which is most common
   */
  void SetGyroSensitivity(double sensitivity);

  /**
   * Resets gyro so that current angle becomes new 0 degrees. Makes sequential turns
   * easier. Also required to make sensor values accurate after noise is encountered
   */
  void ResetGyro();

 private:
  /**
   * Gets the value which will linearize input
   * @param x the value which will be modified
   */
  double Linearize(double x);

  /**
   * If the input is out of bounds, returns the max or min
   */
  double SetLimit(double x);

  // Victors

  Victor* leftDriveMotorA_;
  Victor* leftDriveMotorB_;
  Victor* rightDriveMotorA_;
  Victor* rightDriveMotorB_;

  // Sensors
  Encoder* leftDriveEncoder_;
  Encoder* rightDriveEncoder_;
  Gyro* gyro_;
  Constants* constants;
};

#endif  // SUBSYSTEMS_DRIVE_H_
