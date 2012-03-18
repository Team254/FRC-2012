#ifndef BRIDGE_BALLS_COMMAND_H_
#define BRIDGE_BALLS_COMMAND_H_

#include "auto/AutoCommand.h"

class Intake;
class Shooter;

class BridgeBallsCommand : public AutoCommand {

 public:
  
  BridgeBallsCommand(Intake* intake, Shooter* shooter, bool runIntake, double timeout);
  
  void Initialize();
  
  bool Run();
  
  ~BridgeBallsCommand();
  
 private:
  
  Intake* intake_;
  Shooter* shooter_;
  double timeout_;
  int state_;
  double initTime_;
  bool runIntake_;
  
};

#endif 

