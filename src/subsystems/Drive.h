#ifndef SUBSYSTEMS_DRIVE_H_
#define SUBSYSTEMS_DRIVE_H_

#include "WPILib.h"

#include "config/Constants.h"

/**
 * @author Eric Caldwell
 * @author Bhargava Manja
 * @author Art Kalb
 * @author Tom Bottiglieri
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
        Solenoid* shiftSolenoid, DoubleSolenoid* pizzaWheelSolenoid, DoubleSolenoid* brakeSolenoid, Encoder* leftEncoder,
        Encoder* rightEncoder, Gyro* gyro, Accelerometer* accelerometerX, Accelerometer* accelerometerY,
        Accelerometer* accelerometerZ);

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

  /**
   * Resets the drive encoders
   */
  void ResetEncoders();

  /**
   * Selects the drive gearing
   * @param highGear true for high gear, false for low gear
   */
  void SetHighGear(bool highGear);

  /**
   * Sets the pizza wheel position
   * @param up true for down, false for up
   */
  void SetPizzaWheelDown(bool down);

  /**
   * Gets the current upness of the pizza cutter wheels
   * @return true if wheels up, false if wheels down
   */
  bool GetPizzaUp();

  /**
   * Returns current gyro angle.
   * @return the gyro angle in degrees
   */
  double GetGyroAngle();

  /**
   * Returns current robot acceleration in the X direction
   * @return the robot X acceleration in G's
   */
  double GetXAcceleration();

  /**
   * Returns current robot acceleration in the Y direction
   * @return the robot Y acceleration in G's
   */
  double GetYAcceleration();

  /**
   * Returns current robot acceleration in the Z direction
   * @return the robot Z acceleration in G's
   */
  double GetZAcceleration();

  /**
   * Resets gyro so that current angle becomes new 0 degrees. Makes sequential turns
   * easier. Also required to make sensor values accurate after noise is encountered
   */
  void ResetGyro();

  /**
   * Nice operator control for a fast robot
   */
  void CheesyDrive(double throttle, double wheel, bool quickturn);

  /**
   * Turn brakes on
   */
  void SetBrakeOn(bool on);

  /**
   * Is brake on?
   */
  bool GetBrakeOn();

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

  // Victors
  Victor* leftDriveMotorA_;
  Victor* leftDriveMotorB_;
  Victor* rightDriveMotorA_;
  Victor* rightDriveMotorB_;

  // Sensors
  Encoder* leftDriveEncoder_;
  Encoder* rightDriveEncoder_;
  Gyro* gyro_;
  Accelerometer* accelerometerX_;
  Accelerometer* accelerometerY_;
  Accelerometer* accelerometerZ_;

  // Pneumatics
  Solenoid* shiftSolenoid_;
  DoubleSolenoid* pizzaWheelSolenoid_;

  // Brake
  DoubleSolenoid* brakeSolenoid_;

  DriverStationLCD* lcd_;
  Constants* constants_;
};

#endif  // SUBSYSTEMS_DRIVE_H_
