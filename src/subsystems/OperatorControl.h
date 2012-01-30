#ifndef SUBSYSTEMS_OPERATOR_CONTROL_H_
#define SUBSYSTEMS_OPERATOR_CONTROL_H_

#include "WPILib.h"

#include "config/Constants.h"

/**
 * @author Art Kalb
 *
 * Convenience class that wraps a Joystick to provide access to the buttons on the control board.
 */
class OperatorControl {
 public:
  /**
   * Constructor
   * @param port The port number of the Joystick
   */
  OperatorControl(int port);

  /**
   * Destroys the Joystick and the Constants (created in constructor) variables
   */
  ~OperatorControl();

  /**
   * Return the value of the specified button
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
