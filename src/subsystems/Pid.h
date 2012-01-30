#ifndef SUBSYSTEMS_PID_H_
#define SUBSYSTEMS_PID_H_

#include "WPILib.h"

/**
 * @author Bhargava Manja
 * @author Francisco Sanchez
 *
 * Pid class for generic error correction: scales error to create next motor input and achieve desired value
 */
class Pid {
 public:
  /**
   * Constructor
   * accepts the kP, kI, and kD to determine the correction value
   */
   Pid(double kP, double kI, double kD);

  /**
   * Resets the error counts. Call when the PID loop is not active.
   */
  void ResetError();

  /**
   * Returns the output of the PID controller for the current state of the system.
   * @param goal The current goal to reach
   * @param currentValue The current value of the quantity to be corrected
   */
  double Update(double goal, double currentValue);

 private:
  //PID constants
  double kP_;
  double kI_;
  double kD_;

  //sum of past errors used to create integral term
  double errorSum_;

  //last error, used to find error difference to create derivative term
  double lastError_;
};

#endif // SUBSYSTEMS_PID_H_
