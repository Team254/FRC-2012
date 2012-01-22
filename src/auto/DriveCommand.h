#ifndef AUTO_DRIVE_COMMAND_H_
#define AUTO_DRIVE_COMMAND_H_

#include "auto/AutoCommand.h"
#include "subsystems/Drive.h"
#include "subsystems/Pid.h"

class DriveCommand : public AutoCommand {
 public:
  DriveCommand(Drive* drive, double distance);
  void Initialize();
  bool Run();
 ~DriveCommand();
 private:
  Drive* drive_;
  Pid* leftPid_;
  Pid* rightPid_;
  double distanceGoal_;	 
};

#endif //AUTO_DRIVE_COMMAND_H_
