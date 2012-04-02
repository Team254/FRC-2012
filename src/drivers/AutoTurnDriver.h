#ifndef AUTO_TURN_DRIVER_H
#define AUTO_TURN_DRIVER_H

#include "drivers/Driver.h"

class BackboardFinder;
class DriveCommand;
class Drive;

/**
 * @author Eric Bakan
 *
 * A baselock drive controller
 * Holds current position, but can adjust based on joystick input
 */
class AutoTurnDriver : public Driver {
 public:
  AutoTurnDriver(Drive* drive, BackboardFinder* target);
  virtual void Reset();
  void SetAngle(double angle);
  virtual bool UpdateDriver();
 private:
  BackboardFinder* target_;
  bool justReset_;
  bool foundTarget_;
  DriveCommand* command_;
};

#endif
