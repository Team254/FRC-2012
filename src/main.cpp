#include "main.h"

#include <math.h>
#include <stdio.h>

#include "drivers/Driver.h"
#include "drivers/TeleopDriver.h"
#include "drivers/BaselockDriver.h"
#include "subsystems/Drive.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"
#include "subsystems/ShooterController.h"
#include "subsystems/OperatorControl.h"
#include "subsystems/Pid.h"
#include "subsystems/PidCommander.h"
#include "util/Functions.h"
#include "util/Logger.h"
#include "util/PidTuner.h"
#include "vision/BackboardFinder.h"

Joystick* xbox = new Joystick(3);

MainRobot::MainRobot() {
  // Constants
  constants_ = Constants::GetInstance();

  // Motors
  leftDriveMotorA_ = new Victor((int)constants_->leftDrivePwmA);
  leftDriveMotorB_ = new Victor((int)constants_->leftDrivePwmB);
  rightDriveMotorA_ = new Victor((int)constants_->rightDrivePwmA);
  rightDriveMotorB_ = new Victor((int)constants_->rightDrivePwmB);
  intakeMotor_ = new Victor((int)constants_->intakePwm);
  conveyorMotor_ = new Victor((int)constants_->conveyorPwm);
  leftShooterMotor_ = new Victor((int)constants_->leftShooterPwm);
  rightShooterMotor_ = new Victor((int)constants_->rightShooterPwm);

  // Sensors
  leftEncoder_ = new Encoder((int)constants_->leftEncoderPortA, (int)constants_->leftEncoderPortB);
  leftEncoder_->Start();
  rightEncoder_ = new Encoder((int)constants_->rightEncoderPortA, (int)constants_->rightEncoderPortB);
  rightEncoder_->Start();
  shooterEncoder_ = new Encoder((int)constants_->shooterEncoderPortA, (int)constants_->shooterEncoderPortB,
                                false, CounterBase::k1X);
  shooterEncoder_->Start();
  gyro_ = new Gyro((int)constants_->gyroPort);
  gyro_->SetSensitivity(1.0);

  double accelerometerSensitivity = 1.0;
  //accelerometerX_ = new Accelerometer((int)constants_->accelerometerXPort);
  //accelerometerY_ = new Accelerometer((int)constants_->accelerometerYPort);
  //accelerometerZ_ = new Accelerometer((int)constants_->accelerometerZPort);
  //accelerometerX_->SetSensitivity(accelerometerSensitivity);
  //accelerometerY_->SetSensitivity(accelerometerSensitivity);
  //accelerometerZ_->SetSensitivity(accelerometerSensitivity);
  bumpSensor_ = new DigitalInput((int)constants_->bumpSensorPort);

  // Pneumatics
  compressor_ = new Compressor((int)constants_->compressorPressureSwitchPort,(int)constants_->compressorRelayPort);
  compressor_->Start();
  shiftSolenoid_ = new Solenoid((int)constants_->shiftSolenoidPort);
  hoodSolenoid_ = new Solenoid((int)constants_->hoodSolenoidPort);
  pizzaWheelSolenoid_ = new DoubleSolenoid((int)constants_->pizzaWheelSolenoidDownPort, (int)constants_->pizzaWheelSolenoidUpPort);
  intakeSolenoid_ = new DoubleSolenoid((int)constants_->intakeSolenoidUpPort, (int)constants_->intakeSolenoidDownPort);
  brakeSolenoid_ = new DoubleSolenoid((int)constants_->brakeSolenoidOnPort, (int)constants_->brakeSolenoidOffPort);

  // Subsystems
  drivebase_ = new Drive(leftDriveMotorA_, leftDriveMotorB_, rightDriveMotorA_, rightDriveMotorB_,
                         shiftSolenoid_, pizzaWheelSolenoid_, brakeSolenoid_,  leftEncoder_,
                         rightEncoder_, gyro_, accelerometerX_, accelerometerY_,
                         accelerometerZ_, bumpSensor_);
  intake_ = new Intake(intakeMotor_, intakeSolenoid_);
  shooter_ = new Shooter(conveyorMotor_, leftShooterMotor_, rightShooterMotor_, shooterEncoder_,
                         hoodSolenoid_);
  sc_ = new ShooterController(shooter_, intake_);

  // Control Board
  leftJoystick_ = new Joystick((int)constants_->leftJoystickPort);
  rightJoystick_ = new Joystick((int)constants_->rightJoystickPort);
  operatorControl_ = new OperatorControl((int)constants_->operatorControlPort);

  // Drivers
  teleopDriver_ = new TeleopDriver(drivebase_, leftJoystick_, rightJoystick_, operatorControl_);
  baselockDriver_ = new BaselockDriver(drivebase_, leftJoystick_);
  // Set the current Driver to teleop, though this will change later
  currDriver_ = teleopDriver_;

  testPid_ = new Pid(constants_->driveKP, constants_->driveKI, constants_->driveKD);
  testTimer_ = new Timer();
  testLogger_ = new Logger("/test.log", 2);

  // Watchdog
  GetWatchdog().SetExpiration(100);

  // Get a local instance of the Driver Station LCD
  lcd_ = DriverStationLCD::GetInstance();
  lcd_->PrintfLine(DriverStationLCD::kUser_Line1,"***Teleop Ready!***");

  oldBaseLockSwitch_ = operatorControl_->GetBaseLockSwitch();
}

void MainRobot::DisabledInit() {
}

void MainRobot::AutonomousInit() {
  constants_->LoadFile();
  delete testPid_;
  testPid_ = new Pid(constants_->driveKP, constants_->driveKI, constants_->driveKD);
  drivebase_->ResetGyro();
  drivebase_->ResetEncoders();
  testPid_->ResetError();
  testTimer_->Reset();
  testTimer_->Start();
  GetWatchdog().SetEnabled(false);
}

void MainRobot::TeleopInit() {
  constants_->LoadFile();
  drivebase_->ResetGyro();
  drivebase_->ResetEncoders();
  testTimer_->Start();

  // Start off with the TeleopDriver
  currDriver_ = teleopDriver_;
  currDriver_->Reset();
  GetWatchdog().SetEnabled(true);
}

void MainRobot::DisabledPeriodic() {
  //drivebase_->SetPizzaWheelDown(false);
}

void MainRobot::AutonomousPeriodic() {
  double time = testTimer_->Get();
  double inputValue = Functions::SineWave(time, 5, 2);
  double position = drivebase_->GetLeftEncoderDistance();
  double signal = testPid_->Update(inputValue, position);
  drivebase_->SetLinearPower(signal, signal);
  testLogger_->Log("%f,%f,%f,%f\n", time, inputValue, signal, position);
  PidTuner::PushData(inputValue, position);
}

void MainRobot::TeleopPeriodic() {
  GetWatchdog().Feed();
  drivebase_->SetBrakeOn(false);
  double ljoy = xbox->GetY();
  double trigger = -xbox->GetRawAxis(3);
  double rjoy = -xbox->GetRawAxis(5);
  //  printf("%f %f %f\n", ljoy, trigger, rjoy);
  printf("F: %d | r: %d | off: %d\n", intakeSolenoid_->Get() == DoubleSolenoid::kForward, intakeSolenoid_->Get() == DoubleSolenoid::kReverse, intakeSolenoid_->Get() == DoubleSolenoid::kOff  );
  // Ghetto shooter control for testing
  double tShooter = (trigger > .1) ? trigger : 0;
  shooter_->SetLinearPower(trigger);
  shooter_->SetConveyorPower(ljoy);
  //  intake_->SetIntakePower(rjoy);
  if (xbox->GetRawButton(6)){
    intake_->SetIntakePower(1);
  }
  else if (xbox->GetRawButton(5)){
    intake_->SetIntakePower(-1);
  }
  else {
    intake_->SetIntakePower(0);
  }
  if (xbox->GetRawButton(1)) {
    intake_->SetIntakePosition(Intake::INTAKE_UP);
  } else if (xbox->GetRawButton(2)) {
    intake_->SetIntakePosition(Intake::INTAKE_DOWN);
  } else if (xbox->GetRawButton(8)) {
    intake_->SetIntakePosition(Intake::INTAKE_FLOATING);
  }

  if (xbox->GetRawButton(3)) {
    shooter_->SetHoodUp(true);
  } else if (xbox->GetRawButton(4)) {
    shooter_->SetHoodUp(false);
  }

  // Only have Teleop and Baselock Drivers right now
  if (operatorControl_->GetBaseLockSwitch() && !oldBaseLockSwitch_) {
      // If the baselock switch has been flipped on, switch to baselock
      currDriver_ = baselockDriver_;
      currDriver_->Reset();
  } else if (!operatorControl_->GetBaseLockSwitch() && oldBaseLockSwitch_) {
      // If the baselock switch has been flipped off, switch back to teleop
      currDriver_ = teleopDriver_;
      currDriver_->Reset();
  }

  // Update the driver and the baselock switch status
  currDriver_->UpdateDriver();
  oldBaseLockSwitch_ = operatorControl_->GetBaseLockSwitch();

  double velocity = shooter_->GetVelocity();
  lcd_->PrintfLine(DriverStationLCD::kUser_Line2,"Pos: %d", shooterEncoder_->Get());
  lcd_->PrintfLine(DriverStationLCD::kUser_Line3,"Vel: %f", velocity);
  lcd_->UpdateLCD();
  PidTuner::PushData(70, velocity);
}
