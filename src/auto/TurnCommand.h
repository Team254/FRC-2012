#ifndef AUTO_TURN_CMD_H_
#define AUTO_TURN_CMD_H_

#include "auto/AutoCommand.h"

class Drive;
class Pid;

class TurnCommand : public AutoCommand {
 public:
  /**
   * Constructor. Takes a drive object and the angle to turn.
   */
  TurnCommand(Drive* drive, double angle, double timeout);

  /**
   * Initializes 
   */
  void Initialize();

  /**
   * Uses PID to turn to the wanted angle. Returns true when complete.
   */
  bool Run();

 private:
  Drive* drive_;
  Pid* turnPid_;
  double angle_, oldAngle_;
};

#endif
