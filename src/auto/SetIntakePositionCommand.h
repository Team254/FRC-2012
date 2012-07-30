#ifndef AUTO_INTAKE_POS_COMMAND_H_
#define AUTO_INTAKE_POS_COMMAND_H_

#include "auto/AutoCommand.h"
#include "subsystems/Intake.h"

class SetIntakePositionCommand : public AutoCommand {

 public:

  SetIntakePositionCommand(Intake* intake, Intake::IntakePositions pos);

  void Initialize();

  bool Run();

  ~SetIntakePositionCommand();

 private:

  Intake* intake_;
  Intake::IntakePositions pos_;

};

#endif // AUTO_INTAKE_POS_COMMAND_H_

