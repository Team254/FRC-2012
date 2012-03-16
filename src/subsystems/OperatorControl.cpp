#include "subsystems/OperatorControl.h"

OperatorControl::OperatorControl(int port) {
  operatorJoystick_ = new Joystick(port);
  constants_ = Constants::GetInstance();
}

OperatorControl::~OperatorControl() {
  delete operatorJoystick_;
}

bool OperatorControl::GetControlLoopsSwitch() {
  return operatorJoystick_->GetX()>0.0;
}

bool OperatorControl::GetBrakeSwitch() {
  return operatorJoystick_->GetY()>0.0;
}

Intake::IntakePositions OperatorControl::GetIntakePositionSwitch() {
  bool axis = operatorJoystick_->GetRawAxis(4)<0.0;
  bool button = operatorJoystick_->GetRawButton(12);
  printf("axis: %d button: %d\n",axis,button);
  if (axis) {
    return Intake::INTAKE_UP;
  }
  if (button) {
    return Intake::INTAKE_DOWN;
  }
  return Intake::INTAKE_FLOATING;
}

bool OperatorControl::GetAutonSelectButton() {
  return (bool)operatorJoystick_->GetRawButton((int)constants_->autonSelectPort);
}

bool OperatorControl::GetUnjamButton() {
  return (bool)operatorJoystick_->GetRawButton((int)constants_->unjamPort);
}

bool OperatorControl::GetShootButton() {
  return (bool)operatorJoystick_->GetRawButton((int)constants_->shootPort);
}

bool OperatorControl::GetAutoShootButton() {
  return (bool)operatorJoystick_->GetRawButton((int)constants_->autoShootPort);
}

bool OperatorControl::GetIntakeButton() {
  return (bool)operatorJoystick_->GetRawButton((int)constants_->intakePort);
}

bool OperatorControl::GetIncreaseButton() {
  return (bool)operatorJoystick_->GetRawButton((int)constants_->increasePort);
}

bool OperatorControl::GetDecreaseButton() {
  return (bool)operatorJoystick_->GetRawButton((int)constants_->decreasePort);
}

bool OperatorControl::GetKeyFarButton() {
  return (bool)operatorJoystick_->GetRawButton((int)constants_->keyFarPort);
}

bool OperatorControl::GetKeyCloseButton() {
  return (bool)operatorJoystick_->GetRawButton((int)constants_->keyClosePort);
}

bool OperatorControl::GetFarFenderButton() {
  return (bool)operatorJoystick_->GetRawButton((int)constants_->farFenderPort);
}

bool OperatorControl::GetFenderButton() {
  return (bool)operatorJoystick_->GetRawButton((int)constants_->fenderPort);
}

bool OperatorControl::GetShooterSwitch() {
  return (bool)(operatorJoystick_->GetRawAxis(3)<0.0);
}

