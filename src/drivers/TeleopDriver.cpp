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
  // printf wasn't working, switched to DriverStationLCD temporarily
  DriverStationLCD* lcd = DriverStationLCD::GetInstance();

  // Operator drive control
  bool wantHighGear = !leftJoystick_->GetRawButton((int)constants_->highGearPort);
  bool quickTurning = rightJoystick_->GetRawButton((int)constants_->quickTurnPort);
  drive_->SetHighGear(wantHighGear);
  double straightPower = HandleDeadband(-leftJoystick_->GetY(), 0.1);
  double turnPower = HandleDeadband(rightJoystick_->GetX(), 0.1);

  // Debugging
  lcd->PrintfLine(DriverStationLCD::kUser_Line1, "lj:%.4f rj:%.4f", HandleDeadband(-leftJoystick_->GetY(), 0.1), HandleDeadband(rightJoystick_->GetX(), 0.1));
  lcd->PrintfLine(DriverStationLCD::kUser_Line2, "sp:%.4f tp:%.4f", straightPower, turnPower);

  // Pizza wheel control
  bool currPizzaWheelsButton = rightJoystick_->GetRawButton((int)constants_->pizzaSwitchPort);

  // If the switch has toggled, flip the pizza wheels
  if (currPizzaWheelsButton != oldPizzaWheelsButton_) {
    pizzaWheelsDown_ = !pizzaWheelsDown_;
  }
  // Update the button
  oldPizzaWheelsButton_ = rightJoystick_->GetRawButton((int)constants_->pizzaSwitchPort);

  // If we've run over the bump
  if (pizzaWheelsDown_ && drive_->GetBumpSensorValue()) {
    pizzaWheelsDown_ = false;
  }
  drive_->SetPizzaWheelDown(pizzaWheelsDown_);

  // Brake
  if (rightJoystick_->GetTrigger()) {
    drive_->SetBrakeOn(true);
  } else if (leftJoystick_->GetTrigger()) {
    drive_->SetBrakeOn(false);
  }

  // Drive
  if (pizzaWheelsDown_) {
    turnPower = 0;
  }
  drive_->CheesyDrive(straightPower, turnPower, quickTurning);

  // Teleop is always ready to be broken out of
  return true;
}

void TeleopDriver::Reset() {
  // Reset Pizza Wheels
  oldPizzaWheelsButton_ = rightJoystick_->GetRawButton((int)constants_->pizzaSwitchPort);
  pizzaWheelsDown_= false;
  drive_->SetPizzaWheelDown(pizzaWheelsDown_);
}

TeleopDriver::~TeleopDriver() {
}
