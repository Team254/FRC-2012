#include "drivers/TeleopDriver.h"

#include "config/Constants.h"
#include "subsystems/Drive.h"
#include "subsystems/OperatorControl.h"

TeleopDriver::TeleopDriver(Drive* drive, Joystick* leftJoystick, Joystick* rightJoystick,
                           OperatorControl* operatorControl) : Driver(drive) {
  constants_ = Constants::GetInstance();
  leftJoystick_ = leftJoystick;
  rightJoystick_ = rightJoystick;
  operatorControl_ = operatorControl;
  Reset();
}

bool TeleopDriver::UpdateDriver() {
  // Operator drive control
  bool wantHighGear = !leftJoystick_->GetRawButton((int)constants_->highGearPort);
  bool quickTurning = rightJoystick_->GetRawButton((int)constants_->quickTurnPort);
  drive_->SetHighGear(wantHighGear);
  double straightPower = HandleDeadband(-leftJoystick_->GetY(), 0.1);
  double turnPower = HandleDeadband(rightJoystick_->GetX(), 0.1);

  // If the switch has toggled, flip the pizza wheels.
  bool currPizzaWheelsButton = rightJoystick_->GetRawButton((int)constants_->pizzaSwitchPort);
  if (currPizzaWheelsButton != oldPizzaWheelsButton_) {
    pizzaWheelsDown_ = !pizzaWheelsDown_;
  }
  oldPizzaWheelsButton_ = currPizzaWheelsButton;

  // Retract the pizza wheels if the sensor has detected the bump.
  if (pizzaWheelsDown_ && drive_->GetBumpSensorValue()) {
    pizzaWheelsDown_ = false;
  }
  drive_->SetPizzaWheelDown(pizzaWheelsDown_);

  // Brake
  drive_->SetBrakeOn(operatorControl_->GetBrakeSwitch());

  // Drive
  DriverStationLCD::GetInstance()->PrintfLine(DriverStationLCD::kUser_Line1, "l:%0.3f r:%0.3f", straightPower, turnPower);
  drive_->CheesyDrive(straightPower, turnPower, quickTurning);
  
  return true;
}

void TeleopDriver::Reset() {
  // Reset pizza wheels.
  oldPizzaWheelsButton_ = rightJoystick_->GetRawButton((int)constants_->pizzaSwitchPort);
  pizzaWheelsDown_= false;
  drive_->SetPizzaWheelDown(pizzaWheelsDown_);
}

TeleopDriver::~TeleopDriver() {
}
