#ifndef SUBSYSTEMS_SHOOTER_H_
#define SUBSYSTEMS_SHOOTER_H_

#include "WPILib.h"

#include "config/Constants.h"
#include "subsystems/Pid.h"
#include "util/DaisyFilter.h"
#include "matlab/mat.h"


#define TICKS_PER_REV 32
#define VELOCITY_THRESHOLD 0.3
#define FILTER_SIZE 5
#define OUTPUT_FILTER_SIZE 3

/**
 * @author Eric Bakaan
 *
 * Easy-access functions for shooter functions: shooting, conveying, etc.
 */
class Shooter {
 public:

	enum hoodPref {
		UP,
		DOWN,
		NO,
		NUM_PREFS
	};
  /**
   * Constructor
   * Accepts the Victors, Encoders, pneumatics, etc. to be used
   */
  Shooter(Victor* conveyorMotor, Victor* leftShooterMotor, Victor* rightShooterMotor, Encoder* shooterEncoder,
          Solenoid* hoodSolenoid, AnalogChannel* ballSensor, AnalogChannel* poofMeter,
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
  void SetTargetVelocity(double velocity, hoodPref pref=NO);

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

  bool AtTargetVelocity();

  void Reset();

  double GetBallRange();

  /**
   * returns target velocity
   */
  double GetTargetVelocity();

  static void CallUpdate(void* shooter);

  double SetHardnessOffset(double offset);

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
  AnalogChannel* ballSensor_;
  AnalogChannel* poofMeter_;
  AnalogChannel* ballRanger_;

  // Solenoids
  Solenoid* hoodSolenoid_;

  // Other
  Constants* constants_;
  Timer* timer_;
  Pid* pid_;
  double pidGoal_;
  double prevPos_;
  double targetVelocity_;
  double velocity_;
  double velocityFilter_[FILTER_SIZE];
  int filterIndex_;
  double outputValue_;
  double outputFilter_[OUTPUT_FILTER_SIZE];
  int outputFilterIndex_;
  double poofCorrectionFactor_;
  bool prevBallSensor_;
  bool atTarget_;
  DaisyFilter *filter_;


  // Matrix Stuff
  struct matrix *y_;
  struct matrix *r_;
  ss_controller *ssc_;

  // Offset
  double hardnessOffset_;

};

#endif  // SUBSYSTEMS_SHOOTER_H_
