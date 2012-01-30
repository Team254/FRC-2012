#include "subsystems/OperatorControl.h"

OperatorControl::OperatorControl(int port) {
  operatorJoystick_ = new Joystick(port);
  constants_ = Constants::GetInstance();
}

OperatorControl::~OperatorControl() {
  delete operatorJoystick_;
}

bool OperatorControl::GetConveyorUpButton() {
  return operatorJoystick_->GetRawButton((int)constants_->conveyorUpPort);
}

bool OperatorControl::GetConveyorIndexButton() {
  return operatorJoystick_->GetRawButton((int)constants_->conveyorIndexPort);
}

bool OperatorControl::GetConveyorDownButton() {
  return operatorJoystick_->GetRawButton((int)constants_->conveyorDownPort);
}

bool OperatorControl::GetFineControlLeftButton() {
  return operatorJoystick_->GetRawButton((int)constants_->fineControlLeftPort);
}

bool OperatorControl::GetFineControlRightButton() {
  return operatorJoystick_->GetRawButton((int)constants_->fineControlRightPort);
}

bool OperatorControl::GetIntakeDeploySwitch() {
  return operatorJoystick_->GetRawButton((int)constants_->intakeDeployPort);
}

bool OperatorControl::GetPresetFenderButton() {
  return operatorJoystick_->GetRawButton((int)constants_->presetFenderPort);
}

bool OperatorControl::GetPresetKeyButton() {
  return operatorJoystick_->GetRawButton((int)constants_->presetKeyPort);
}

bool OperatorControl::GetPresetHalfCourtButton() {
  return operatorJoystick_->GetRawButton((int)constants_->presetHalfCourtPort);
}

bool OperatorControl::GetPresetFullCourtButton() {
  return operatorJoystick_->GetRawButton((int)constants_->presetMaxPort);
}

bool OperatorControl::GetShooterSwitch() {
  return operatorJoystick_->GetRawButton((int)constants_->shooterPort);
}

bool OperatorControl::GetHoodIncrementButton() {
  return operatorJoystick_->GetRawButton((int)constants_->hoodIncrementPort);
}

bool OperatorControl::GetHoodDecrementButton() {
  return operatorJoystick_->GetRawButton((int)constants_->hoodDecrementPort);
}

bool OperatorControl::GetBaseLockSwitch() {
  return operatorJoystick_->GetRawButton((int)constants_->baseLockPort);
}

bool OperatorControl::GetManualOverrideSwitch() {
  return operatorJoystick_->GetRawButton((int)constants_->manualOverridePort);
}

bool OperatorControl::GetBridgeLowererButton() {
  return operatorJoystick_->GetRawButton((int)constants_->bridgeLowererPort);
}
