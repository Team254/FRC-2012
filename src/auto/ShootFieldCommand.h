#ifndef ShootField_COMMAND_H_
#define ShootField_COMMAND_H_

#include "auto/AutoCommand.h"

class ShootCommand;
class Intake;
class Timer;
class Shooter;

class ShootFieldCommand : public AutoCommand {

 public:  
  ShootFieldCommand(Shooter* shooter, Intake* intake, bool runIntake,double shootSpeed, int shotsToFire, double timeout);
  void Initialize();
  bool Run();
  ~ShootFieldCommand();
  
 private:
  ShootCommand* cmd_;
  Intake* intake_;
  Timer* jumbleTimer_;
  
};

#endif 

