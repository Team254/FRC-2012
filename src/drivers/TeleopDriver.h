#ifndef DRIVERS_TELEOP_DRIVER_H
#define DRIVERS_TELEOP_DRIVER_H

#include "drivers/Driver.h"

class Constants;
class Joystick;
class OperatorControl;

/**
 * @author Eric Bakan
 *
 * A teleop drive controller
 * Primarily translate control board inputs into drive outputs
 */
class TeleopDriver : public Driver {
 public:

  /**
   * Only needs a drive and control board inputs
   */
  TeleopDriver(Drive* drive, Joystick* leftJoystick, Joystick* rightJoystick,
               OperatorControl* operatorControl_);

  /**
   * Updates the driver
   * Controls ALL the logic
   */
  virtual bool UpdateDriver();

  /**
   * Resets some toggles and such like pizza wheels, etc.
   */
  virtual void Reset();

  /**
   * Clean up clean up, everybody everywhere!
   * Clean up clean up, everybody do your share!
   */
  virtual ~TeleopDriver();
  
  void AskForBrake(bool on);
  
 private:
  Constants* constants_;
  Joystick* leftJoystick_;
  Joystick* rightJoystick_;
  OperatorControl* operatorControl_;

  // Pizza cutter wheels!
  bool oldPizzaWheelsButton_;
  bool pizzaWheelsDown_;
  bool askForBrake_;
  
  //initial joystick vals
  double startLeftJoystick_;
  double startRightJoystick_;
};

#endif // DRIVERS_TELEOP_DRIVER_H
