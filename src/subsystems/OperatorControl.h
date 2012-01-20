#ifndef SUBSYSTEMS_OPERATOR_CONTROL_H_
#define SUBSYSTEMS_OPERATOR_CONTROL_H_

#include "WPILib.h"

#include "config/Constants.h"

/**
 * @author Art Kalb
 * 
 * OperatorControl is a wrapper class that provides quick and easy access to the buttons on the ControlBoard.
 */
class OperatorControl {
 public:
  /**
   * @param stick Takes a joystick and stores it allowing the class to use it later.
   */	
  OperatorControl(Joystick* stick);

  /**
   * Destroys the Joystick and the Constants (created in constructor) variables
   */
  ~OperatorControl();

  /**
   * The Get* functions return the value of the button following Get
   */
  bool GetConveyorUpButton();
  bool GetConveyorIndexButton();
  bool GetConveyorDownButton();
  bool GetFineControlLeftButton();
  bool GetFineControlRightButton();
  bool GetIntakeDeploySwitch();
  bool GetPresetFenderButton();
  bool GetPresetKeyButton();
  bool GetPresetHalfCourtButton();
  bool GetPresetFullCourtButton();
  bool GetShooterSwitch();
  bool GetHoodIncrementButton();
  bool GetHoodDecrementButton();
  bool GetBaseLockSwitch();
  bool GetManualOverrideSwitch();
  bool GetBridgeLowererButton();

 private:
  Joystick* operatorJoystick_;
  Constants* constants_;
};

#endif  // SUBSYSTEMS_OPERATOR_CONTROL_H_
