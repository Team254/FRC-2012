#ifndef AUTO_SHOOT_BRIDGE_CMD_H_
#define AUTO_SHOOT_BRIDGE_CMD_H_

#include "auto/AutoCommand.h"

class Drive;
class Pid;
class Shooter;
class Intake;

class ShootFromBridgeCommand : public AutoCommand {
 public:
  /**
   * Constructor. Takes shooter and intake objects and whether or not to run the intake
   */
  ShootFromBridgeCommand(Shooter* shooter, Intake* intake, bool runIntake,double shootSpeed, int shotsToFire, double timeout, bool intakeDown = false);

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
  ~ShootFromBridgeCommand();

 private:
  Shooter* shooter_;
  Intake* intake_;
  Timer* shotSpotterTimer_;
  bool runIntake_;
  double shootSpeed_;
  int shotsToFire_;
  bool reachedSpeed_;
  int shotsFired_;
  int downCycles_;
  int atSpeedCycles_;
  bool intakeDown_;
  bool initialIntakeDown_;
  Timer* lastShotTimer_;
};

#endif
