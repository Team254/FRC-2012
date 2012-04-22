#ifndef DRIVERS_BASELOCK_DRIVER_H
#define DRIVERS_BASELOCK_DRIVER_H

#include "drivers/Driver.h"

class Constants;
class Joystick;
class OperatorControl;
class Pid;
class DriveCommand;
/**
 * @author Eric Bakan
 *
 * A baselock drive controller
 * Holds current position, but can adjust based on joystick input
 */
class BaselockDriver : public Driver {
 public:

  /**
   * Only needs a drive and control board inputs
   */
  BaselockDriver(Drive* drive, Joystick* leftJoystick, Joystick* rightJoystick);

  /**
   * Updates the driver
   * Controls ALL the logic
   */
  virtual bool UpdateDriver();

  /**
   * Resets the pid, the baselock switch, etc.
   */
  virtual void Reset();

  /**
   * Clean up clean up, everybody everywhere!
   * Clean up clean up, everybody do your share!
   */
  virtual ~BaselockDriver();

 private:
  Constants* constants_;
  Joystick* leftJoystick_;
  Joystick* rightJoystick_;
  DriveCommand* cmd_;
  double baseLockPosition_;
  double angleGoal_;
  double distanceGoal_;
};

#endif // DRIVERS_BASELOCK_DRIVER_H
