#ifndef AUTO_SHOOT_CMD_H_
#define AUTO_SHOOT_CMD_H_

#include "auto/AutoCommand.h"

class Drive;
class Pid;
class Shooter;
class Intake;

class ShootCommand : public AutoCommand {
 public:
  /**
   * Constructor. Takes shooter and intake objects and whether or not to run the intake
   */
  ShootCommand(Shooter* shooter, Intake* intake, bool runIntake,double shootSpeed, double timeout);

  /**
   * Initializes 
   */
  void Initialize();

  /**
   * Runs conveyor, and runs intake if specified
   */
  bool Run();
  
  /**
   * Destructor
   */
  ~ShootCommand();

 private:
  Shooter* shooter_;
  Intake* intake_;
  bool runIntake_;
  double shootSpeed_;
};

#endif
