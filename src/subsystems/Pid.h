#ifndef SUBSYSTEMS_PID_H_
#define SUBSYSTEMS_PID_H_

#include "WPILib.h"

#include "config/Constants.h"

/**
 * @author Bhargava Manja
 * @author Francisco Sanchez
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
	  * sets the new goal for the Pid to correct
	  * @param goal the new goal
	  */
	 void SetGoal(double goal);
	 
	 /**
	  * updates the correction value to get to the desired goal
	  * @param currentValue the current value of the quantity to be corrected
	  */
	 double Update(double currentValue);
	 
 private:
	 //PID constants
	 double kP_;
	 double kI_;
	 double kD_;
	 //goal to be achieved
	 double goal_;
	 //sum of past errors used to create integral term
	 double errorSum_;
	 //last error, used to find error difference to create derivative term
	 double lastError_;
};

#endif