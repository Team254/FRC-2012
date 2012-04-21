#ifndef AUTO_INTAKE_COMMAND_H_
#define AUTO_INTAKE_COMMAND_H_

#include "auto/AutoCommand.h"
#include "subsystems/Intake.h"

class Shooter;

class IntakeCommand : public AutoCommand {

 public:
  
  IntakeCommand(Intake* intake, Shooter* shooter);
  
  void Initialize();
  
  bool Run();
  
  ~IntakeCommand();
  
 private:
  
  Intake* intake_;
  Shooter* shooter_;
  
};

#endif // AUTO_INTAKE_POS_COMMAND_H_

