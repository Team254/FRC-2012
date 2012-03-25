#ifndef AUTO_JUMBLE_CMD_H_
#define AUTO_JUMBLE_CMD_H_

#include "auto/AutoCommand.h"

class Shooter;
class Intake;

class JumbleCommand : public AutoCommand {
 public:
  /**
   * Constructor. Takes shooter and intake objects and whether or not to run the intake
   */
  JumbleCommand(Shooter* shooter, Intake* intake, double timeout);

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
  ~JumbleCommand();

 private:
  Shooter* shooter_;
  Intake* intake_;
};

#endif
