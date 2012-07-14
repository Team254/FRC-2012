#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <math.h>

/**
 * @author Eric Bakan
 * Useful functions which are repeated across classes
 */

/**
 * Limits a PWM to plus or minus 1
 * @param pwm the input PWM
 * @return the capped PWM value
 */
inline double PwmLimit(double pwm) {
  return pwm >= 1.0 ? 1.0 : pwm <= -1.0 ? -1.0 : pwm;
}

/**
 * Implements a deadband on a joystick
 * @param val the joystick value
 * @param deadband the maximum value the deadband will return 0 for
 * @return 0.0 if the absolute value of the joystick value is less than the deadband, else the joystick value
 */
inline double HandleDeadband(double val, double deadband) {
  return (fabs(val) > fabs(deadband)) ? val : 0.0;
}

#endif // UTILS_HPP_
