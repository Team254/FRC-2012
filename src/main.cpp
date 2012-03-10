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
  jumbleMotor_ = new Victor((int)constants_->jumblePwm);

  // Sensors
  leftEncoder_ = new Encoder((int)constants_->leftEncoderPortA, (int)constants_->leftEncoderPortB);
  leftEncoder_->Start();
  rightEncoder_ = new Encoder((int)constants_->rightEncoderPortA, (int)constants_->rightEncoderPortB);
  rightEncoder_->Start();
  shooterEncoder_ = new Encoder((int)constants_->shooterEncoderPortA, (int)constants_->shooterEncoderPortB,
                                false, CounterBase::k1X);
  shooterEncoder_->Start();
  conveyorEncoder_ = new Encoder((int)constants_->conveyorEncoderPortA,
                                 (int)constants_->conveyorEncoderPortB, true);
  conveyorEncoder_->Start();
  gyro_ = new Gyro((int)constants_->gyroPort);
  gyro_->SetSensitivity(1.0);
  poofMeter_ = new AnalogChannel((int)constants_->poofMeterPort);
  ballRanger_ = new AnalogChannel((int)constants_->ballRangerPort);

  double accelerometerSensitivity = 1.0;
  //accelerometerX_ = new Accelerometer((int)constants_->accelerometerXPort);
  //accelerometerY_ = new Accelerometer((int)constants_->accelerometerYPort);
  //accelerometerZ_ = new Accelerometer((int)constants_->accelerometerZPort);
  //accelerometerX_->SetSensitivity(accelerometerSensitivity);
  //accelerometerY_->SetSensitivity(accelerometerSensitivity);
  //accelerometerZ_->SetSensitivity(accelerometerSensitivity);
  bumpSensor_ = new DigitalInput((int)constants_->bumpSensorPort);
  conveyorBallSensor_ = new AnalogChannel((int)constants_->conveyorBallSensorPort);

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
                         hoodSolenoid_, conveyorEncoder_, conveyorBallSensor_, poofMeter_, ballRanger_);
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

  testPid_ = new Pid(&constants_->driveKP, &constants_->driveKI, &constants_->driveKD);
  testTimer_ = new Timer();
  testLogger_ = new Logger("/test.log", 1);

  // Watchdog
  GetWatchdog().SetExpiration(100);

  // Get a local instance of the Driver Station LCD
  lcd_ = DriverStationLCD::GetInstance();
  lcd_->PrintfLine(DriverStationLCD::kUser_Line1,"***Teleop Ready!***");

  oldBaseLockSwitch_ = operatorControl_->GetBaseLockSwitch();

  shooterTargetVelocity_ = 0;
  oldShooterUpSwitch_ = false;
  oldShooterDownSwitch_ = false;
  oldBallQueueSwitch_ = false;
}

void MainRobot::DisabledInit() {
}

void MainRobot::AutonomousInit() {
  constants_->LoadFile();
  delete testPid_;
  testPid_ = new Pid(&constants_->driveKP, &constants_->driveKI, &constants_->driveKD);
  drivebase_->ResetGyro();
  drivebase_->ResetEncoders();
  testPid_->ResetError();
  testTimer_->Reset();
  testTimer_->Start();
  conveyorEncoder_->Reset();
  GetWatchdog().SetEnabled(false);
  power_ = 0;
}

void MainRobot::TeleopInit() {
  constants_->LoadFile();
  drivebase_->ResetGyro();
  drivebase_->ResetEncoders();
  testTimer_->Start();
  shooter_->ResetQueueState();

  // Start off with the TeleopDriver
  currDriver_ = teleopDriver_;
  currDriver_->Reset();
  GetWatchdog().SetEnabled(true);
}

void MainRobot::DisabledPeriodic() {
  lcd_->PrintfLine(DriverStationLCD::kUser_Line1, "Convey: %d", conveyorEncoder_->Get());
  lcd_->PrintfLine(DriverStationLCD::kUser_Line2, "Ball: %d", conveyorBallSensor_->GetValue());
  lcd_->PrintfLine(DriverStationLCD::kUser_Line3, "Poof: %d", poofMeter_->GetValue());
  lcd_->UpdateLCD();
}

void MainRobot::AutonomousPeriodic() {
/*
  double time = testTimer_->Get();
  double rate = conveyorEncoder_->GetRate();

  if (time > 0.5) {
    power_ += 0.01;
    testTimer_->Reset();
    testLogger_->Log("%f,%f\n", power_, rate);
  }
  lcd_->PrintfLine(DriverStationLCD::kUser_Line5,"Pow: %.0f%%", power_ * 100);
  lcd_->PrintfLine(DriverStationLCD::kUser_Line6,"Rate: %.0f%%", rate);
  lcd_->UpdateLCD();
  shooter_->SetLinearConveyorPower(power_);
*/
  int target = (int)Functions::SquareWave(testTimer_->Get(), 10, 500) + 500;
  shooter_->SetConveyorTarget(target);
  shooter_->ConveyorPIDUpdate();
  PidTuner::PushData(target, conveyorEncoder_->Get(), 0);
}

void MainRobot::TeleopPeriodic() {
  GetWatchdog().Feed();
  drivebase_->SetBrakeOn(false);

  // Ghetto shooter control for testing
  if (xbox->GetRawButton(10)) {
    shooterTargetVelocity_ = 0;
  } else if (xbox->GetRawButton(9) && !oldShooterDownSwitch_) {
    shooterTargetVelocity_ -= 2;
  } else if (xbox->GetRawButton(8) && !oldShooterUpSwitch_) {
    shooterTargetVelocity_ += 2;
  } else if (xbox->GetRawButton(7)) {
    shooterTargetVelocity_ = 38;
  }
  oldShooterUpSwitch_ = xbox->GetRawButton(8);
  oldShooterDownSwitch_ = xbox->GetRawButton(9);
  shooter_->SetTargetVelocity(shooterTargetVelocity_);
  shooter_->PIDUpdate();

  // Automatic ball queueing control.
  if (xbox->GetTwist() < -0.75) {
    if (shooter_->QueueBall() && !oldBallQueueSwitch_) {
      shooter_->ShootBall();
    }
  } else {
    shooter_->SetLinearConveyorPower(0);
  }
  oldBallQueueSwitch_ = (xbox->GetTwist() < -0.75);

  if (xbox->GetRawButton(12)) {
    intake_->SetIntakePower(1.0);
    shooter_->SetLinearConveyorPower(-1.0);
  } else if (xbox->GetRawButton(11)) {
    intake_->SetIntakePower(1.0);
      if (xbox->GetRawButton(3)) {
        shooter_->SetLinearConveyorPower(0.45);
      } else {
        shooter_->SetLinearConveyorPower(1.0);
      }
  } else if (xbox->GetZ() < -0.75) {
    intake_->SetIntakePower(-1.0);
    shooter_->SetLinearConveyorPower(-1.0);
  } else {
    intake_->SetIntakePower(0);
  }

  if (xbox->GetRawButton(6)) {
    intake_->SetIntakePosition(Intake::INTAKE_UP);
  } else if (xbox->GetRawButton(4)) {
    intake_->SetIntakePosition(Intake::INTAKE_DOWN);
  } else if (xbox->GetRawButton(5)) {
    intake_->SetIntakePosition(Intake::INTAKE_FLOATING);
  }

  if (shooterTargetVelocity_ > 50) {
    shooter_->SetHoodUp(true);
  } else {
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
  lcd_->PrintfLine(DriverStationLCD::kUser_Line3,"Vel: %f", velocity);
  lcd_->PrintfLine(DriverStationLCD::kUser_Line4,"Shoot: %.0f%%", shooterTargetVelocity_);
  lcd_->PrintfLine(DriverStationLCD::kUser_Line5, "Ranger: %d", ballRanger_->GetValue());
  lcd_->UpdateLCD();
}
