#include "WPILib.h"
#include "subsystems/Shooter.h"
#include "subsystems/Intake.h"

class ShooterController {
 public:
  void RunIntake();
  void StopIntake();
  void IntakeDown();
  ShooterController(Shooter* shooter, Intake* intake);

 private:
  Shooter* shooter_;
  Intake* intake_;
  bool intakeRunning_;
  Timer* intakeDownTimer_;
};
