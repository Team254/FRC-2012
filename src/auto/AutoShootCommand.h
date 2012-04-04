#ifndef AUTO_AUTO_SHOOT_CMD_H_
#define AUTO_AUTO_SHOOT_CMD_H_

#include "auto/AutoCommand.h"

class Shooter;
class Intake;
class BackboardFinder;
class ShootCommand;
class Constants;

class AutoShootCommand : public AutoCommand {
 public:
  
  AutoShootCommand(Shooter* shooter, Intake* intake, BackboardFinder* target, 
		           bool runIntake, int shotsToFire, double timeout, 
		           bool intakeDown = false);	
	
  void Initialize();
  
  bool Run();
  
  ~AutoShootCommand();
  
 private:
  
  Shooter* shooter_;
  Intake* intake_;
  BackboardFinder* target_;
  ShootCommand* cmd_;
  Constants* constants_;
  bool runIntake_;
  int shotsToFire_;
  bool reachedSpeed_;
  bool intakeDown_;
  	
};

#endif //AutoShootCommand
