#ifndef SUBSYSTEMS_SHOOTER_H_
#define SUBSYSTEMS_SHOOTER_H_

#include "WPILib.h"

#include "config/Constants.h"
#include "subsystems/Pid.h"

/**
 * @author Eric Bakaan
 *
 * Easy-access functions for shooter functions: shooting, intaking, conveying, etc.
 * If you haven't taken Janke's class yet, wubbleu=lowercase omega=rotational velocity
 */
class Shooter {
 public:
  enum IntakePositions {
    INTAKE_UP,
    INTAKE_DOWN,
    INTAKE_FLOATING
  };

  /**
   * Constructor
   * Accepts the Victors, Encoders, pneumatics, etc. to be used
   */
  Shooter(Victor* intakeMotor, Victor* conveyorMotor, Victor* leftShooterMotor,
          Victor* rightShooterMotor, Encoder* shooterEncoder, Solenoid* hoodSolenoid,
          DoubleSolenoid* intakeSolenoid);

  /**
   * Sets the linearized power of the shooter motors
   * @param pwm the power to st
   */
  void SetLinearPower(double pwm);
  
  /**
   * Sets the target rotational velocity of the wheel in degrees/second
   * @param wubbleu the rotational velocity to set
   */
  void SetTargetWubbleu(double wubbleu);
  
  /**
   * Updates the shooter wheel velocity PID
   * @return true if done, else false
   */
  bool PIDUpdate();
  
  /**
   * Sets the power of the conveyor motor
   * @param pwm the power to st
   */
  void SetConveyorPower(double pwm);
  
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
  /**
   * Sets the solenoid position for the hood
   * @param up true to set up, else false
   */
  void SetHoodUp(bool up);
  
  /**
   * Get the shooter wheel's current rotational velocity
   * @return the shooter wheel's rotational velocity in degrees/second
   */
  double GetShooterWubbleu();
  
  /**
   * Resets the shooter wheel encoder
   */
  void ResetEncoder();
  
 private:
  /**
   * Sets the unlinearized power of the shooter motors
   * @param pwm the power to st
   */
  void SetPower(double power);
  
  /**
   * Linearizes the shooter motors
   * @param x the unlinearized input
   * @return the linearzied output
   */
  double Linearize(double x);

  // Motors
  Victor* intakeMotor_;
  Victor* conveyorMotor_;
  Victor* leftShooterMotor_;
  Victor* rightShooterMotor_;

  // Sensors
  Encoder* shooterEncoder_;

  // Solenoids
  Solenoid*  hoodSolenoid_;
  DoubleSolenoid* intakeSolenoid_;

  // Other
  Constants* constants_;
  Timer timer_;
  Pid pid_;
  double prevShooterVal_;
  double targetWubbleu_;
};

#endif  // SUBSYSTEMS_SHOOTER_H_
