#ifndef SUBSYSTEMS_INTAKE_H_
#define SUBSYSTEMS_INTAKE_H_

#include "WPILib.h"

#include "config/Constants.h"
#include "subsystems/Pid.h"

/**
 * @author Patrick Fairbank
 *
 * Class encapsulating the functionality of the intake.
 */
class Intake {
 public:
  enum IntakePositions {
    INTAKE_UP,
    INTAKE_DOWN,
    INTAKE_FLOATING
  };

  /**
   * Constructor
   * Accepts the Victors and pneumatics, etc. to be used
   */
  Intake(Victor* intakeMotor1, Victor* intakeMotor2, Victor* intakeMotor3, DoubleSolenoid* intakeSolenoid);

  /**
   * Sets the power of the intake motor
   * @param pwm the power to st
   */
  void SetIntakePower(double pwm);

  /**
   * Sets the solenoid position for the intake
   * @param up true to set up, else false
   */
  void SetIntakePosition(IntakePositions pos);

  IntakePositions GetIntakePosition();

 private:
  // Motors
  Victor* intakeMotor1_;
  Victor* intakeMotor2_;
  Victor* intakeMotor3_;

  // Solenoids
  DoubleSolenoid* intakeSolenoid_;

  // Other
  Constants* constants_;
  Timer timer_;
};

#endif  // SUBSYSTEMS_INTAKE_H_
