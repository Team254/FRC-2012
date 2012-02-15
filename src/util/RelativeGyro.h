#ifndef UTIL_RELATIVE_GYRO_H_
#define UTIL_RELATIVE_GYRO_H_

#include "WPILib.h"

/**
 * @author Tom Bottiglieri
 *
 * Subclass on a WPILib gyro that can keep track of both absolute and relative
 * angle after a Reset()
 */
class RelativeGyro : public Gyro {
 public:

  /**
   * Returns the absolute angle since boot or last absolute reset
   */
  float GetAbsoluteAngle();

  /**
   * Returns the relative angle since reset
   */
  float GetAngle();

  /**
   * Resets the relative angle
   */
  void Reset();

  /**
   * Resets the absolute angle
   */
  void ResetAbsolute();

  /**
   * Constrcutor. Takes an analog port number.
   */
  RelativeGyro(int port);

 private:
  float offset_;
};

#endif //UTIL_RELATIVE_GYRO_H_
