#ifndef SUBSYSTEMS_DRIVE_H_
#define SUBSYSTEMS_DRIVE_H_

#include "WPILib.h"

#include "config/Constants.h"

/**
 * @author Eric Caldwell
 * @author Bhargava Manja
 * @author Art Kalb
 *
 * Easy-access functions for drive functions: setting power, getting encoder values, etc.
 */
class Drive {
 public:
  /**
   * Constructor
   * Accepts the Victors and Encoders to get and set values
   */
  Drive(Victor* leftVictorA, Victor* leftVictorB, Victor* rightVictorA, Victor* rightVictorB,
        Solenoid* shiftSolenoid, Encoder* leftEncoder, Encoder* rightEncoder, Gyro* gyro);

  /**
   * Makes drive power linear to input, then sets power to the respective side of the drivetrain
   * @param left left power
   * @param right right power
   */
  void SetLinearPower(double left, double right);

  /**
   * Gets the distance travelled by the left side of the robot in inches
   * Calculated via the wheel circumference, gear ratio, and encoder return value
   * @return inches travelled by the left encoder
   */
  double GetLeftEncoderDistance();

  /**
   * Gets the distance travelled by the right side of the robot in meters
   * Calculated via the wheel circumference, gear ratio, and encoder return value
   * @return inches travelled by the right encoder
   */
  double GetRightEncoderDistance();

  void ResetEncoders();

  /**
   * Selects the drive gearing
   * @param highGear true for high gear, false for low gear
   */
  void SetHighGear(bool highGear);

  /**
   * Returns current gyro angle.
   * @return the gyro angle in degrees
   */
  double GetGyroAngle();

  /**
   * Sets gyro's sensitivity. Input is in V/sec, so remember to divide argument
   * by 1000 if spec sheet says mV, which is most common
   * @param sensitivity the sensitivity to set
   */
  void SetGyroSensitivity(double sensitivity);

  /**
   * Resets gyro so that current angle becomes new 0 degrees. Makes sequential turns
   * easier. Also required to make sensor values accurate after noise is encountered
   */
  void ResetGyro();

 private:
  /**
   * Sets unlinearized power to the left and right sides of the drivetrain
   * @param left left power
   * @param right right power
   */
  void SetPower(double left, double right);

  /**
   * Linearizes an Victor input pwm based on the pre-calculated linearization polynomial
   * @param pwm the desired pwm
   * @return the de-linearized victor pwm output
   */
  double Linearize(double x);

  /**
   * Caps a value to plus or minus 1.0
   * @param val the value to be limited
   * @return the capped value
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

  // Pneumatics
  Solenoid* shiftSolenoid_;

  Constants* constants;
};

#endif  // SUBSYSTEMS_DRIVE_H_
