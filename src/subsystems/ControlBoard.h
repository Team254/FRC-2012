#ifndef CONTROLBOARD_H_
#define CONTROLBOARD_H_

#include "WPILib.h"

class ControlBoard {
 public:
  enum BoardJoystick {
    kLeftForward,
    kLeftLateral,
    kRightForward,
    kRightLateral
  };
  enum BoardButton {
  
  };	
  ControlBoard(Joystick* left, Joystick* right);
  double GetJoystickValue(BoardJoystick stick);
  double GetButtonValue(BoardButton input);
  
 private:
  
  
  Joystick* leftJoystick_;
  Joystick* rightJoystick_;
};

#endif //CONTROLBOARD_H_
