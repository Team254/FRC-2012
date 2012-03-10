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
  int position; // Position on the conveyor relative to the starting position in ticks
  int poofiness; // Arbitrary unit of squishiness
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
          Solenoid* hoodSolenoid, Encoder* conveyorEncoder, AnalogChannel* ballSensor, AnalogChannel* poofMeter,
          AnalogChannel* ballRanger);

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
   * Sets the target conveyor position in ticks
   */
  void SetConveyorTarget(int target);

  /**
   * Updates the conveyor position PID
   */
  void ConveyorPIDUpdate();

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
   * @return true if a ball has been queued and is ready to shoot
   */
  bool QueueBall();

  /**
   * Sets the shooter wheel velocity and pops the ball off the queue
   */
  void ShootBall();

  /**
   * Empties the queue
   */
  void ResetQueueState();

  /**
   * Sets the shooter target velocity based on the current ball, distance, etc.
   * @param ball the stats of the ball we're shooting
   */
  void SetBallShooterTarget(ballStats ball);

  void DebugBallQueue();

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
  AnalogChannel* ballSensor_;
  AnalogChannel* poofMeter_;
  AnalogChannel* ballRanger_;

  // Solenoids
  Solenoid* hoodSolenoid_;

  // Other
  Constants* constants_;
  Timer* timer_;
  Pid* pid_;
  Pid* conveyorPid_;
  std::deque<ballStats> ballQ_;
  int prevEncoderPos_;
  int conveyorTarget_;
  double targetVelocity_;
  double velocity_;
  double velocityFilter_[FILTER_SIZE];
  int filterIndex_;
  double outputValue_;
  double outputFilter_[OUTPUT_FILTER_SIZE];
  int outputFilterIndex_;
  double poofCorrectionFactor_;
  bool prevBallSensor_;
};

#endif  // SUBSYSTEMS_SHOOTER_H_
