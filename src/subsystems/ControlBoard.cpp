#include "ControlBoard.h"

ControlBoard::ControlBoard(Joystick* left, Joystick* right) {
  leftJoystick_ = left;
  rightJoystick_ = right;
}

double ControlBoard::GetJoystickValue(BoardJoystick stick) {
  switch (stick) {
    case kLeftForward:
      break;
    case kLeftLateral:
      break;
    case kRightForward:
      break;
    case kRightLateral:
      break;
    default:
      return 0;
      break;
  }
}

double ControlBoard::GetButtonValue(BoardButton button) {
	
}
