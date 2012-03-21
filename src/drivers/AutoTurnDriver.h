#ifndef AUTO_TURN_DRIVER_H
#define AUTO_TURN_DRIVER_H

#include "drivers/Driver.h"

class Timer;
class Encoder;
class Gyro;
class MovingAverageFilter;
class Pid;
class Constants;
class BackboardFinder;

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
  double angle_;
  Encoder* leftEncoder_;
  Encoder* rightEncoder_;
  Timer* timer_;
  double lastPosL_;
  double lastPosR_;
  double lastTimer_;
  bool justReset_;
  MovingAverageFilter* filterL_;
  MovingAverageFilter* filterR_;
  Pid* pidL_;
  Pid* pidR_;
  Pid* pid_;
  double outputValueL_;
  double outputValueR_;
  Constants* constants_;
  bool staticFriction_;
  BackboardFinder* target_;
  double angleGoal_;
};

#endif
