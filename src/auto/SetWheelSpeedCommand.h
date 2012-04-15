#ifndef SetWheelSpeed_COMMAND_H_
#define SetWheelSpeed_COMMAND_H_

#include "auto/AutoCommand.h"
class Shooter;

class SetWheelSpeedCommand : public AutoCommand {

 public:  
  SetWheelSpeedCommand(Shooter* shooter, double speed);
  void Initialize();
  bool Run();
  ~SetWheelSpeedCommand();
 private:
  Shooter* shooter_;
  double speed_;
  
};

#endif 

