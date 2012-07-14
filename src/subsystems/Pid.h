#ifndef SUBSYSTEMS_PID_H_
#define SUBSYSTEMS_PID_H_

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
   * Accepts the kP, kI, and kD to determine the correction value
   * Defaults to (0,0,0) - useful when a class has member Pid's
   */
   Pid(double* kP = 0, double* kI = 0, double* kD = 0);
  /**
   * Resets the error counts. Call when the PID loop is not active to prevent integral windup.
   */
  void ResetError();

  /**
   * Returns the output of the PID controller for the current state of the system.
   * @param goal The current goal to reach
   * @param currentValue The current value of the quantity to be corrected
   */
  double Update(double goal, double currentValue);

 private:
  // PID constants
  double* kP_;
  double* kI_;
  double* kD_;

  // Cumulative error used in integral term
  double errorSum_;

  // Last error value used to find error difference for derivative term
  double lastError_;
};

#endif  // SUBSYSTEMS_PID_H_
