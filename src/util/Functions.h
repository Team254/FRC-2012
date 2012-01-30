#ifndef UTIL_FUNCTIONS_H_
#define UTIL_FUNCTIONS_H_

/**
 * @author Patrick Fairbank
 * @author Avery Strand
 *
 * Static class containing methods for generating various functions of time.
 */
class Functions {
 public:
  /**
   * Generates a square wave with the given period and amplitude at the given point in time.
   */
  static double SquareWave(double timeSec, double periodSec, double amplitude);

  /**
   * Generates a sine wave with the given period and amplitude at the given point in time.
   */
  static double SineWave(double timeSec, double periodSec, double amplitude);

 private:
  // Prevent instantiation of this class since it contains only static methods.
  Functions() {}
};

#endif  // UTIL_FUNCTIONS_H_
