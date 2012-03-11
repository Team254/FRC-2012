#ifndef SUBSYSTEMS_OPERATOR_CONTROL_H_
#define SUBSYSTEMS_OPERATOR_CONTROL_H_

#include "WPILib.h"

#include "config/Constants.h"
#include "subsystems/Intake.h"

/**
 * @author Art Kalb
 * @author Eric Bakan
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
  bool GetControlLoopsSwitch();
  bool GetBrakeSwitch();
  Intake::IntakePositions GetIntakePositionSwitch();
  bool GetAutonSelectButton();
  bool GetUnjamButton();
  bool GetShootButton();
  bool GetAutoShootButton();
  bool GetIntakeButton();
  bool GetIncreaseButton();
  bool GetDecreaseButton();
  bool GetKeyFarButton();
  bool GetKeyCloseButton();
  bool GetFarFenderButton();
  bool GetFenderButton();
  bool GetShooterSwitch();


 private:
  Joystick* operatorJoystick_;
  Constants* constants_;
};

#endif  // SUBSYSTEMS_OPERATOR_CONTROL_H_
