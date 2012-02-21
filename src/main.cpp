#include "main.h"

#include <math.h>
#include <stdio.h>

#include "drivers/Driver.h"
#include "drivers/TeleopDriver.h"
#include "drivers/BaselockDriver.h"
#include "subsystems/Drive.h"
#include "subsystems/Shooter.h"
#include "subsystems/OperatorControl.h"
#include "subsystems/Pid.h"
#include "subsystems/PidCommander.h"
#include "util/Functions.h"
#include "util/Logger.h"
#include "util/PidTuner.h"
#include "vision/BackboardFinder.h"

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
  shooterEncoder_ = new Encoder((int)constants_->shooterEncoderPortA, (int)constants_->shooterEncoderPortB);
  shooterEncoder_->Start();
  gyro_ = new Gyro((int)constants_->gyroPort);
  gyro_->SetSensitivity(1.0);

  double accelerometerSensitivity = 1.0;
  accelerometerX_ = new Accelerometer((int)constants_->accelerometerXPort);
  accelerometerY_ = new Accelerometer((int)constants_->accelerometerYPort);
  accelerometerZ_ = new Accelerometer((int)constants_->accelerometerZPort);
  //accelerometerX_->SetSensitivity(accelerometerSensitivity);
  //  accelerometerY_->SetSensitivity(accelerometerSensitivity);
  //  accelerometerZ_->SetSensitivity(accelerometerSensitivity);
  bumpSensor_ = new DigitalInput((int)constants_->bumpSensorPort);

  // Pneumatics
  compressor_ = new Compressor((int)constants_->compressorPressureSwitchPort,(int)constants_->compressorRelayPort);
  compressor_->Start();
  shiftSolenoid_ = new Solenoid((int)constants_->shiftSolenoidPort);
//  hoodSolenoid_ = new Solenoid((int)constants_->hoodSolenoidPort);
  pizzaWheelSolenoid_ = new DoubleSolenoid((int)constants_->pizzaWheelSolenoidDownPort, (int)constants_->pizzaWheelSolenoidUpPort);
//  intakeSolenoid_ = new DoubleSolenoid((int)constants_->intakeSolenoidHighPort,(int)constants_->intakeSolenoidLowPort);
  brakeSolenoid_ = new DoubleSolenoid((int)constants_->brakeSolenoidHighPort, (int)constants_->brakeSolenoidLowPort);

  // Subsystems
  drivebase_ = new Drive(leftDriveMotorA_, leftDriveMotorB_, rightDriveMotorA_, rightDriveMotorB_,
                         shiftSolenoid_, pizzaWheelSolenoid_, brakeSolenoid_,  leftEncoder_,
                         rightEncoder_, gyro_, accelerometerX_, accelerometerY_,
                         accelerometerZ_, bumpSensor_);
//  shooter_ = new Shooter(intakeMotor_, conveyorMotor_, leftShooterMotor_, rightShooterMotor_,
//                         shooterEncoder_, hoodSolenoid_, intakeSolenoid_);

  // Drivers
  teleopDriver_ = new TeleopDriver(drivebase_, leftJoystick_, rightJoystick_, operatorControl_);
  baselockDriver_ = new BaselockDriver(drivebase_, leftJoystick_);
  // Set the current Driver to teleop, though this will change later
  currDriver_ = teleopDriver_;

  // Control Board
  leftJoystick_ = new Joystick((int)constants_->leftJoystickPort);
  rightJoystick_ = new Joystick((int)constants_->rightJoystickPort);
  operatorControl_ = new OperatorControl((int)constants_->operatorControlPort);

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
  //baseLockPosition_ = drivebase_->GetLeftEncoderDistance();
  //baseLockPid_->ResetError();

  // Start off with the TeleopDriver
  currDriver_ = teleopDriver_;
  currDriver_->Reset();
  GetWatchdog().SetEnabled(true);
}

void MainRobot::DisabledPeriodic() {
  drivebase_->SetPizzaWheelDown(false);
  lcd_->UpdateLCD();
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

  static int i = 0;
  lcd_->PrintfLine(DriverStationLCD::kUser_Line3, "%d, %f, %f, %f\n", i,(float) drivebase_->GetXAcceleration(),(float)drivebase_->GetXAcceleration(),(float) drivebase_->GetXAcceleration());
  lcd_->UpdateLCD();

  Logger::GetSysLog()->Log("%d, %f, %f, %f\n", i,(float) drivebase_->GetXAcceleration(),(float)drivebase_->GetXAcceleration(),(float) drivebase_->GetXAcceleration());
  
  i++;
}

