#ifndef UTILS_HPP_
#define UTILS_HPP_
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
  return pwm>=1.0?1.0:pwm<=-1.0?-1.0:pwm;
}

#endif // UTILS_HPP_
