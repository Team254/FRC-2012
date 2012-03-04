#ifndef SUBSYSTEMS_SHOOTER_H_
#define SUBSYSTEMS_SHOOTER_H_

#include "WPILib.h"
#include <deque>

#include "config/Constants.h"
#include "subsystems/Pid.h"

#define TICKS_PER_REV 32
#define VELOCITY_THRESHOLD 1.0
#define FILTER_SIZE 5
#define OUTPUT_FILTER_SIZE 3

/**
 * Represents a collection of stats about a given ball in the queue
 */
struct ballStats {
  double pos; // Position on the conveyor relative to the starting position in ticks
  double poofs; // Arbitrary unit of squishiness
};

/**
 * @author Eric Bakaan
 *
 * Easy-access functions for shooter functions: shooting, intaking, conveying, etc.
 */
class Shooter {
 public:
  /**
   * Constructor
   * Accepts the Victors, Encoders, pneumatics, etc. to be used
   */
  Shooter(Victor* conveyorMotor, Victor* leftShooterMotor, Victor* rightShooterMotor, Encoder* shooterEncoder,
          Solenoid* hoodSolenoid, Encoder* conveyorEncoder, DigitalInput* ballSensor);

  /**
   * Sets the linearized power of the shooter motors
   * @param pwm the power to st
   */
  void SetLinearPower(double pwm);

  /**
   * Sets the target rotational velocity of the wheel in rotations/second
   * @param velocity the rotational velocity to set
   */
  void SetTargetVelocity(double velocity);

  /**
   * Updates the shooter wheel velocity PID
   * @return true if done, else false
   */
  bool PIDUpdate();

  /**
   * Sets the linearized power of the conveyor motor
   * @param pwm the power to st
   */
  void SetLinearConveyorPower(double pwm);

  /**
   * Sets the RELATIVE target conveyor position in ticks
   * @param deltaTicks the change in ticks to add to the current target
   */
  void SetConveyorTarget(double deltaTicks);

  /**
   * Sets the conveyor target to the current position
   */
  void ResetConveyorTarget();

  /**
   * Updates the conveyor position PID
   * @return true if done, else false
   */
  bool ConveyorPIDUpdate();

  /**
   * Sets the solenoid position for the hood
   * @param up true to set up, else false
   */
  void SetHoodUp(bool up);

  double UpdateFilter(double value);

  double UpdateOutputFilter(double value);

  /**
   * Get the shooter wheel's current measured rotational velocity
   * @return the shooter wheel's rotational velocity in rotations/second
   */
  double GetVelocity();

  /**
   * Sets the unlinearized power of the shooter motors
   * @param pwm the power to st
   */
  void SetPower(double power);

  /**
   * Runs the conveyor belt until a ball has been loaded from the conveyor
   * @return true if a ball has been detected and added to the queue, else false
   */
  bool QueueBall();

  /**
   * Sets the shooter wheel velocity and pops the ball off the queue
   */
  void ShootBall();

  /**
   * Empties the queue
   */
  void ResetQueue();

  /**
   * Sets the shooter target velocity based on the current ball, distance, etc.
   * @param ball the stats of the ball we're shooting
   */
  void SetBallShooterTarget(ballStats ball);

 private:
  /**
   * Linearizes the shooter motors
   * @param x the unlinearized input
   * @return the linearized output
   */
  double Linearize(double x);

  /**
   * Linearizes the conveyor motor
   * @param x the unlinearized input
   * @return the linearized output
   */
  double ConveyorLinearize(double x);

  // Motors
  Victor* conveyorMotor_;
  Victor* leftShooterMotor_;
  Victor* rightShooterMotor_;

  // Sensors
  Encoder* shooterEncoder_;
  Encoder* conveyorEncoder_;
  DigitalInput* ballSensor_;

  // Solenoids
  Solenoid* hoodSolenoid_;

  // Other
  Constants* constants_;
  Timer* timer_;
  Pid* pid_;
  Pid* conveyorPid_;
  std::deque<ballStats> ballQ_;
  int prevEncoderPos_;
  double conveyorTarget_;
  double targetVelocity_;
  double velocity_;
  double velocityFilter_[FILTER_SIZE];
  int filterIndex_;
  double outputValue_;
  double outputFilter_[OUTPUT_FILTER_SIZE];
  int outputFilterIndex_;
};

#endif  // SUBSYSTEMS_SHOOTER_H_
