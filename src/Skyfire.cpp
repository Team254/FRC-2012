#include "skyfire.h"

#include <cmath>
#include <cstdio>

#include "auto/AutoAlignCommand.h"
#include "auto/BridgeBallsCommand.h"
#include "auto/ConcurrentCommand.h"
#include "auto/DriveCommand.h"
#include "auto/SequentialCommand.h"
#include "auto/ShootCommand.h"
#include "auto/TurnCommand.h"
#include "drivers/AutoTurnDriver.h"
#include "drivers/BaselockDriver.h"
#include "drivers/Driver.h"
#include "drivers/TeleopDriver.h"
#include "subsystems/Drive.h"
#include "subsystems/Intake.h"
#include "subsystems/OperatorControl.h"
#include "subsystems/Shooter.h"
#include "subsystems/Pid.h"
#include "subsystems/PidCommander.h"
#include "util/Functions.h"
#include "util/Logger.h"
#include "util/PidTuner.h"
#include "util/RelativeGyro.h"
#include "vision/BackboardFinder.h"

Skyfire::Skyfire() {
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
  gyro_ = new RelativeGyro((int)constants_->gyroPort);
  bumpSensor_ = new DigitalInput((int)constants_->bumpSensorPort);
  conveyorBallSensor_ = new AnalogChannel((int)constants_->conveyorBallSensorPort);
  poofMeter_ = new AnalogChannel((int)constants_->poofMeterPort);
  ballRanger_ = new AnalogChannel((int)constants_->ballRangerPort);

  // Pneumatics
  compressor_ = new Compressor((int)constants_->compressorPressureSwitchPort,
                               (int)constants_->compressorRelayPort);
  compressor_->Start();
  shiftSolenoid_ = new Solenoid((int)constants_->shiftSolenoidPort);
  hoodSolenoid_ = new Solenoid((int)constants_->hoodSolenoidPort);
  pizzaWheelSolenoid_ = new DoubleSolenoid((int)constants_->pizzaWheelSolenoidDownPort,
                                           (int)constants_->pizzaWheelSolenoidUpPort);
  intakeSolenoid_ = new DoubleSolenoid((int)constants_->intakeSolenoidUpPort,
                                       (int)constants_->intakeSolenoidDownPort);
  brakeSolenoid_ = new DoubleSolenoid((int)constants_->brakeSolenoidOnPort,
                                      (int)constants_->brakeSolenoidOffPort);

  // Subsystems
  drivebase_ = new Drive(leftDriveMotorA_, leftDriveMotorB_, rightDriveMotorA_, rightDriveMotorB_,
                         shiftSolenoid_, pizzaWheelSolenoid_, brakeSolenoid_,  leftEncoder_, rightEncoder_,
                         gyro_, bumpSensor_);
  intake_ = new Intake(intakeMotor_, intakeSolenoid_);
  shooter_ = new Shooter(conveyorMotor_, leftShooterMotor_, rightShooterMotor_, shooterEncoder_,
                         hoodSolenoid_, conveyorBallSensor_, poofMeter_, ballRanger_);
  shooterTargetVelocity_ = 0;

  // Control board
  leftJoystick_ = new Joystick((int)constants_->leftJoystickPort);
  rightJoystick_ = new Joystick((int)constants_->rightJoystickPort);
  operatorControl_ = new OperatorControl((int)constants_->operatorControlPort);

  // Vision
  target_ = new BackboardFinder();
  target_->Start();

  // Drivers
  teleopDriver_ = new TeleopDriver(drivebase_, leftJoystick_, rightJoystick_, operatorControl_);
  baselockDriver_ = new BaselockDriver(drivebase_, leftJoystick_);
  autoAlignDriver_ = new AutoTurnDriver(drivebase_, target_);
  currDriver_ = teleopDriver_;

  // Watchdog
  GetWatchdog().SetExpiration(1000);
  GetWatchdog().SetEnabled(true);

  // Get a local instance of the Driver Station LCD
  lcd_ = DriverStationLCD::GetInstance();

  // Initialize button edge-detection variables
  oldAutoAlignButton_ = leftJoystick_->GetRawButton((int)constants_->autoAlignPort);
  oldShooterSwitch_ = operatorControl_->GetShooterSwitch();
  oldIncreaseButton_ = operatorControl_->GetIncreaseButton();
  oldDecreaseButton_ = operatorControl_->GetDecreaseButton();
  oldAutonSelectButton_ = operatorControl_->GetAutonSelectButton();

  // Initialize autonomous variables
  autonDelay_ = 0.0;
  autonTimer_ = new Timer();
  autonMode_ = AUTON_CLOSE_BRIDGE_SLOW;
  autoBaseCmd_ = NULL;
}

void Skyfire::ResetMotors() {
  drivebase_->SetLinearPower(0, 0);
  shooter_->SetLinearPower(0);
  shooter_->SetLinearConveyorPower(0);
  intake_->SetIntakePower(0);
}

void Skyfire::DisabledInit() {
  drivebase_->ResetEncoders();
  drivebase_->ResetGyro();
}

void Skyfire::AutonomousInit() {
  constants_->LoadFile();
  ResetMotors();
  autonTimer_->Reset();
  autonTimer_->Start();
  intake_->SetIntakePosition(Intake::INTAKE_UP);

  if (autoBaseCmd_) {
    delete autoBaseCmd_;
    autoBaseCmd_ = NULL;
  }

  switch (autonMode_) {
    case AUTON_NONE:
      break;
    case AUTON_FENDER:
      autoBaseCmd_ = AUTO_SEQUENTIAL(
           new DriveCommand(drivebase_, -64, false, 3),
          AUTO_CONCURRENT(
              new DriveCommand(drivebase_, -100, false, .5),
               new ShootCommand(shooter_, intake_, true, 38, 2, 6.0)));
      break;
    case AUTON_CLOSE_BRIDGE_SLOW:
      autoBaseCmd_ = AUTO_SEQUENTIAL(
          new ShootCommand(shooter_, intake_, true, 48, 2, 7.0),
           new DriveCommand(drivebase_, 146, false, 7),
           AUTO_CONCURRENT(
               new BridgeBallsCommand(intake_, shooter_, true, 3.8),
               new DriveCommand(drivebase_, -5.5, false, 4)),
           new DriveCommand(drivebase_, -76, false, 6.0),
           new AutoAlignCommand(drivebase_, autoAlignDriver_, 2.0),
          new ShootCommand(shooter_, intake_, true, 50, 2, 8.0));
      break;
    case AUTON_FAR_BRIDGE_SLOW:
      autoBaseCmd_ = AUTO_SEQUENTIAL(
          new ShootCommand(shooter_, intake_, true, 52, 2, 3.0),
          new DriveCommand(drivebase_, 43, false, 7),
           AUTO_CONCURRENT(
               new BridgeBallsCommand(intake_, shooter_, true, 4.2),
               new DriveCommand(drivebase_, -5, false, 4)),
          new DriveCommand(drivebase_, -43, false, 3),
          new ShootCommand(shooter_, intake_, true, 52, 2, 8.0));
        break;
    case AUTON_BRIDGE_FAST:
      autoBaseCmd_ = AUTO_SEQUENTIAL(
          new ShootCommand(shooter_, intake_, false,Constants::GetInstance()->autoShootKeyVel, 2, 3),
          new DriveCommand(drivebase_, 50,  false, 4),
          new BridgeBallsCommand(intake_, shooter_, true, 5.0),
          new DriveCommand(drivebase_, -50, false, 4),
          new AutoAlignCommand(drivebase_, autoAlignDriver_, 2.5),
          new ShootCommand(shooter_, intake_, true,Constants::GetInstance()->autoShootKeyVel, 2, 10.0));
      break;
    case AUTON_ALLIANCE_BRIDGE:
      autoBaseCmd_ = AUTO_SEQUENTIAL(
          new ShootCommand(shooter_, intake_, false, Constants::GetInstance()->autoShootKeyVel, 2, 3.75),
          new TurnCommand(drivebase_, 90, 3),
          new DriveCommand(drivebase_, 132, false, 7),
          new TurnCommand(drivebase_, -90, 3),
          new DriveCommand(drivebase_, 50, false, 4),
          new BridgeBallsCommand(intake_, shooter_, true, 5.0),
          new DriveCommand(drivebase_, -50, false, 4),
          new TurnCommand(drivebase_, 90, 3),
          new DriveCommand(drivebase_, -132, false, 7),
          new TurnCommand(drivebase_, -90, 3),
          new ShootCommand(shooter_, intake_, true, Constants::GetInstance()->autoShootKeyVel, 2, 10.0));
      break;
    default:
      autoBaseCmd_ = NULL;
  }

  if (autoBaseCmd_) {
    autoBaseCmd_->Initialize();
  }
}

void Skyfire::TeleopInit() {
  constants_->LoadFile();
  ResetMotors();
  drivebase_->ResetGyro();
  drivebase_->ResetEncoders();

  currDriver_ = teleopDriver_;
  currDriver_->Reset();
}

void Skyfire::DisabledPeriodic() {
  GetWatchdog().Feed();

  // Autonomous delay
  if (operatorControl_->GetIncreaseButton() && !oldIncreaseButton_) {
    autonDelay_ += 0.5;
  } else if (operatorControl_->GetDecreaseButton() && !oldDecreaseButton_) {
    autonDelay_ = max(autonDelay_ - 0.5, 0.0);
  }

  // Autonomous mode selection
  if (operatorControl_->GetAutonSelectButton() && !oldAutonSelectButton_) {
    autonMode_ = (AutonMode)(autonMode_ + 1);
    if (autonMode_ == NUM_AUTON_MODES) {
      autonMode_ = AUTON_NONE;
    }
  }

  oldIncreaseButton_ = operatorControl_->GetIncreaseButton();
  oldDecreaseButton_ = operatorControl_->GetDecreaseButton();
  oldAutonSelectButton_ = operatorControl_->GetAutonSelectButton();

  // Show selected mode and delay on the Driver Station LCD.
  switch (autonMode_) {
    case AUTON_NONE:
      lcd_->PrintfLine(DriverStationLCD::kUser_Line1, "No auton");
      break;
    case AUTON_FENDER:
      lcd_->PrintfLine(DriverStationLCD::kUser_Line1, "Fender");
      break;
    case AUTON_CLOSE_BRIDGE_SLOW:
      lcd_->PrintfLine(DriverStationLCD::kUser_Line1, "Near bridge slow");
      break;
    case AUTON_FAR_BRIDGE_SLOW:
      lcd_->PrintfLine(DriverStationLCD::kUser_Line1, "Far bridge slow");
      break;
    case AUTON_BRIDGE_FAST:
      lcd_->PrintfLine(DriverStationLCD::kUser_Line1, "Bridge fast");
      break;
    case AUTON_ALLIANCE_BRIDGE:
      lcd_->PrintfLine(DriverStationLCD::kUser_Line1, "Alliance bridge");
      break;
    default:
      lcd_->PrintfLine(DriverStationLCD::kUser_Line1, "Invalid auton");
  }
  lcd_->PrintfLine(DriverStationLCD::kUser_Line2, "Delay: %.1f", autonDelay_);

  // Show any other pre-match stuff we're interested in on the Driver Station LCD.
  lcd_->PrintfLine(DriverStationLCD::kUser_Line4, "Gyro: %f\n", gyro_->GetAngle());
  lcd_->UpdateLCD();
}

void Skyfire::AutonomousPeriodic() {
  GetWatchdog().Feed();

  if (autonTimer_->Get() > autonDelay_ && autoBaseCmd_) {
    autoBaseCmd_->Run();
  }
  shooter_->PIDUpdate();
}

void Skyfire::TeleopPeriodic() {
  GetWatchdog().Feed();

  // Update shooter power/manual control
  if (operatorControl_->GetFenderButton()) {
    shooterTargetVelocity_ = constants_->shooterFenderSpeed;
  } else if (operatorControl_->GetFarFenderButton()) {
    shooterTargetVelocity_ = constants_->shooterFarFenderSpeed;
  } else if (operatorControl_->GetKeyCloseButton()) {
    shooterTargetVelocity_ = constants_->shooterKeyCloseSpeed;
  } else if (operatorControl_->GetKeyFarButton()) {
    shooterTargetVelocity_ = constants_->shooterKeyFarSpeed;
  } else if (operatorControl_->GetIncreaseButton() && !oldIncreaseButton_) {
    shooterTargetVelocity_ += constants_->shooterSpeedIncrement;
  } else if (operatorControl_->GetDecreaseButton() && !oldDecreaseButton_) {
    shooterTargetVelocity_ -= constants_->shooterSpeedIncrement;
  }

  if (operatorControl_->GetShooterSwitch()) {
    // Re-load the shooter PID constants whenever the shooter is turned on.
    if (!oldShooterSwitch_) {
      constants_->LoadFile();
    }
    shooter_->SetTargetVelocity(shooterTargetVelocity_);
  } else {
    shooter_->SetTargetVelocity(0);
  }
  bool shooterDone = shooter_->PIDUpdate();

  // Update shooter button guards
  oldShooterSwitch_ = operatorControl_->GetShooterSwitch();
  oldIncreaseButton_ = operatorControl_->GetIncreaseButton();
  oldDecreaseButton_ = operatorControl_->GetDecreaseButton();

  if (operatorControl_->GetAutoShootButton()) {
    // In auto-shoot mode, only feed the balls if the shooter is up to speed.
    if (operatorControl_->GetShooterSwitch() && shooterDone) {
      shooter_->SetLinearConveyorPower(1.0);
      intake_->SetIntakePower(1.0);
    } else {
      shooter_->SetLinearConveyorPower(0.0);
      intake_->SetIntakePower(0.0);
    }
  } else if (operatorControl_->GetShootButton()) {
    // In manual shoot mode, run the conveyor and intake to feed the shooter.
    shooter_->SetLinearConveyorPower(1.0);
    intake_->SetIntakePower(1.0);
  } else if (operatorControl_->GetUnjamButton()) {
    // In exhaust mode, run the conveyor and intake backwards.
    intake_->SetIntakePower(-1.0);
    shooter_->SetLinearConveyorPower(-1.0);
  } else if (operatorControl_->GetIntakeButton()) {
    // In intake mode, run the intake forwards and the conveyor backwards to jumble balls in the hopper.
    intake_->SetIntakePower(1.0);
    shooter_->SetLinearConveyorPower(-1.0);
  } else {
    shooter_->SetLinearConveyorPower(0.0);
    intake_->SetIntakePower(0.0);
  }

  // Only have Teleop and AutoAlign Drivers right now
  if (leftJoystick_->GetRawButton((int)constants_->autoAlignPort) && !oldAutoAlignButton_) {
    // If the auto-align button is pressed, switch to the auto-align driver.
    currDriver_ = autoAlignDriver_;
    currDriver_->Reset();
  } else if (!leftJoystick_->GetRawButton((int)constants_->autoAlignPort) && oldAutoAlignButton_) {
    // If the auto-align button is released, switch back to the teleop driver.
    currDriver_ = teleopDriver_;
    currDriver_->Reset();
  }
  oldAutoAlignButton_ = leftJoystick_->GetRawButton((int)constants_->autoAlignPort);

  // Calculate the outputs for the drivetrain given the inputs.
  currDriver_->UpdateDriver();

  if (!drivebase_->GetPizzaUp()) {
    // If pizza wheels are down, set the intake up to prevent damage.
    intake_->SetIntakePosition(Intake::INTAKE_UP);
  } else {
    intake_->SetIntakePosition(operatorControl_->GetIntakePositionSwitch());
  }

  // Print useful information to the LCD display.
  lcd_->PrintfLine(DriverStationLCD::kUser_Line1, "Shooter set|mea:");
  lcd_->PrintfLine(DriverStationLCD::kUser_Line2, "%.1f | %.1f rps", shooterTargetVelocity_,
                   shooter_->GetVelocity());
  lcd_->PrintfLine(DriverStationLCD::kUser_Line4, "Ranger: %d", ballRanger_->GetValue());
  lcd_->PrintfLine(DriverStationLCD::kUser_Line5, "Gyro: %f", gyro_->GetAngle());
  lcd_->UpdateLCD();
}
