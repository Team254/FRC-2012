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
#include "util/RelativeGyro.h"
#include "vision/BackboardFinder.h"
#include "drivers/AutoTurnDriver.h"
#include "auto/SequentialCommand.h"
#include "auto/ShootCommand.h"
#include "auto/DriveCommand.h"
#include "auto/BridgeBallsCommand.h"
#include "auto/AutoAlignCommand.h"
#include "auto/TurnCommand.h"

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
  // Encoders
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
  gyro_ = new RelativeGyro((int)constants_->gyroPort);
  //gyro_->SetSensitivity(constants_->gyroSensitivity);
  bumpSensor_ = new DigitalInput((int)constants_->bumpSensorPort);
  conveyorBallSensor_ = new AnalogChannel((int)constants_->conveyorBallSensorPort);
  poofMeter_ = new AnalogChannel((int)constants_->poofMeterPort);
  ballRanger_ = new AnalogChannel((int)constants_->ballRangerPort);

  // Accelerometer
  //double accelerometerSensitivity = 1.0;
  //accelerometerX_ = new Accelerometer((int)constants_->accelerometerXPort);
  //accelerometerY_ = new Accelerometer((int)constants_->accelerometerYPort);
  //accelerometerZ_ = new Accelerometer((int)constants_->accelerometerZPort);
  //accelerometerX_->SetSensitivity(accelerometerSensitivity);
  //accelerometerY_->SetSensitivity(accelerometerSensitivity);
  //accelerometerZ_->SetSensitivity(accelerometerSensitivity);

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

  // Vision Tasks
  target_ = new BackboardFinder();
  target_->Start();
  ledRingSwitch_ = new DigitalOutput((int)constants_->ledRingSwitchPort);

  // Drivers
  teleopDriver_ = new TeleopDriver(drivebase_, leftJoystick_, rightJoystick_, operatorControl_);
  baselockDriver_ = new BaselockDriver(drivebase_, leftJoystick_);
  autoAlignDriver_ = new AutoTurnDriver(drivebase_, target_);

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

  shooterTargetVelocity_ = 0;
  autoBaseCmd_ = NULL;

  oldAutoAlignButton_ = leftJoystick_->GetRawButton((int)constants_->autoAlignPort);
  // Shooter button guards
  oldShooterSwitch_ = operatorControl_->GetShooterSwitch();
  oldIncreaseButton_ = operatorControl_->GetIncreaseButton();
  oldDecreaseButton_ = operatorControl_->GetDecreaseButton();

  autonDelay_ = 0.0;
  autonTimer_ = new Timer();
  autonMode_ = AUTON_NONE;
}

void MainRobot::ResetMotorPower() {
	drivebase_->SetLinearPower(0, 0);
	shooter_->SetLinearPower(0);
	shooter_->SetLinearConveyorPower(0);
	intake_->SetIntakePower(0);
}

void MainRobot::DisabledInit() {
  drivebase_->ResetEncoders();
  drivebase_->ResetGyro();
//  target_->Stop();
}

void MainRobot::AutonomousInit() {
  constants_->LoadFile();
//  target_->Start();
  GetWatchdog().SetEnabled(false);
  
  intake_->SetIntakePosition(Intake::INTAKE_UP);
  
  ResetMotorPower();
  autonTimer_->Reset();
  autonTimer_->Start();

  if (autoBaseCmd_) {
    delete autoBaseCmd_;
    autoBaseCmd_ = NULL;
  }

  switch (autonMode_) {
    case AUTON_NONE:
      break;
    case AUTON_FENDER:
      autoBaseCmd_ = new SequentialCommand(2, 
   		  new DriveCommand(drivebase_, -40, false),
   		  new ShootCommand(shooter_, intake_, false, 38, 5));
      break;
    case AUTON_BRIDGE_SLOW:
      autoBaseCmd_ = new SequentialCommand(6,
          new ShootCommand(shooter_, intake_, false,Constants::GetInstance()->autoShootKeyVel, 3),
          new DriveCommand(drivebase_, 50,  false),
          new BridgeBallsCommand(intake_, shooter_, true, 5.0),
          new DriveCommand(drivebase_, -50, false),
          new AutoAlignCommand(drivebase_, autoAlignDriver_, 2.5),
          new ShootCommand(shooter_, intake_, true,Constants::GetInstance()->autoShootKeyVel, 10.0));
      break;
    case AUTON_BRIDGE_FAST:

      break;
    case AUTON_ALLIANCE_BRIDGE:
      autoBaseCmd_ = new SequentialCommand(11,
          new ShootCommand(shooter_, intake_, false, Constants::GetInstance()->autoShootKeyVel, 3.75),
          new TurnCommand(drivebase_, 90, 3),
          new DriveCommand(drivebase_, 132, false),
          new TurnCommand(drivebase_, -90, 3),
          new DriveCommand(drivebase_, 50, false),
          new BridgeBallsCommand(intake_, shooter_, true, 5.0),
          new DriveCommand(drivebase_, -50, false),
          new TurnCommand(drivebase_, 90, 3),
          new DriveCommand(drivebase_, -132, false),
          new TurnCommand(drivebase_, -90, 3),
          new ShootCommand(shooter_, intake_, true, Constants::GetInstance()->autoShootKeyVel, 10.0));
      break;
    default:
      autoBaseCmd_ = NULL;
  }

  if (autoBaseCmd_) {
    autoBaseCmd_->Initialize();
  }
}

void MainRobot::TeleopInit() {
  constants_->LoadFile();
  drivebase_->ResetGyro();
  drivebase_->ResetEncoders();
  testTimer_->Start();
  shooter_->ResetQueueState();
  ResetMotorPower();
  // Start off with the TeleopDriver
  currDriver_ = teleopDriver_;
  currDriver_->Reset();
//  target_->Start();
  GetWatchdog().SetEnabled(true);
}

void MainRobot::DisabledPeriodic() {
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

  switch (autonMode_) {
    case AUTON_NONE:
      lcd_->PrintfLine(DriverStationLCD::kUser_Line1, "No auton", conveyorEncoder_->Get());
      break;
    case AUTON_FENDER:
      lcd_->PrintfLine(DriverStationLCD::kUser_Line1, "Fender", conveyorEncoder_->Get());
      break;
    case AUTON_BRIDGE_SLOW:
      lcd_->PrintfLine(DriverStationLCD::kUser_Line1, "Bridge slow", conveyorEncoder_->Get());
      break;
    case AUTON_BRIDGE_FAST:
      lcd_->PrintfLine(DriverStationLCD::kUser_Line1, "Bridge fast", conveyorEncoder_->Get());
      break;
    case AUTON_ALLIANCE_BRIDGE:
      lcd_->PrintfLine(DriverStationLCD::kUser_Line1, "Alliance bridge", conveyorEncoder_->Get());
      break;
    default:
      lcd_->PrintfLine(DriverStationLCD::kUser_Line1, "Invalid auton", conveyorEncoder_->Get());
  }
  lcd_->PrintfLine(DriverStationLCD::kUser_Line2, "Delay: %.1f", autonDelay_);

  lcd_->PrintfLine(DriverStationLCD::kUser_Line4, "Gyro: %f\n", gyro_->GetAngle());
  lcd_->PrintfLine(DriverStationLCD::kUser_Line5, "Sens: %f\n", constants_->gyroSensitivity);
  if (operatorControl_->GetIntakeButton()) {
    constants_->LoadFile();
    gyro_->SetSensitivity(constants_->gyroSensitivity);
    gyro_->Reset();
  }

  lcd_->UpdateLCD();
}

void MainRobot::AutonomousPeriodic() { 	
  if (autonTimer_->Get() > autonDelay_ && autoBaseCmd_) {
    autoBaseCmd_->Run();
  }
}

void MainRobot::TeleopPeriodic() {
  GetWatchdog().Feed();
  drivebase_->SetBrakeOn(false);
  static Logger* driveLog = new Logger("/turnLog.log");
  if(rightJoystick_->GetRawButton((int)constants_->quickTurnPort)) {
	  driveLog->Log("%f,%f\n", target_->GetX(), drivebase_->GetGyroAngle());
  }
  /*
  static int counter = 0;
  static Logger* driveLog = new Logger("/driveLog.log");
  if (counter % 10 == 0) {
    double left = drivebase_->GetLeftEncoderDistance();
    double right = drivebase_->GetRightEncoderDistance();
    double currX = 0;
    double currV = 0;
    double currH = 0;
    if(target_->SeesTarget()) {
    	currX = target_->GetX();
    	currV = target_->GetVDiff();
    	currH = target_->GetHDiff();
    }
    //angle, distance
    driveLog->Log("%f,%f,%f,%f\n", left, currX, currV, currH);
    //PidTuner::PushData(drivebase_->GetGyroAngle(), currX, 0);
    //target_->LogCamera();
  }
  counter++;
  */

  // Update shooter power/manual control
  if (operatorControl_->GetFenderButton()) {
    shooterTargetVelocity_ = 38;
  } else if (operatorControl_->GetFarFenderButton()) {
    shooterTargetVelocity_ = 46;
  } else if (operatorControl_->GetKeyCloseButton()) {
    shooterTargetVelocity_ = 48;
  } else if (operatorControl_->GetKeyFarButton()) {
    shooterTargetVelocity_ = 53;
  } else if (operatorControl_->GetIncreaseButton() && !oldIncreaseButton_) {
    shooterTargetVelocity_ += 1;
  } else if (operatorControl_->GetDecreaseButton() && !oldDecreaseButton_) {
    shooterTargetVelocity_ -= 1;
  }

  // LED ring transistor (when we get it)
  ledRingSwitch_->Set(operatorControl_->GetShooterSwitch());

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
    if (operatorControl_->GetShooterSwitch() && shooterDone) {
      shooter_->SetLinearConveyorPower(1.0);
    } else {
      shooter_->SetLinearConveyorPower(0.0);
    }
    intake_->SetIntakePower(1.0);
  } else if (operatorControl_->GetShootButton()) {
    shooter_->SetLinearConveyorPower(1.0);
    intake_->SetIntakePower(1.0);
  } else if (operatorControl_->GetUnjamButton()) {
    intake_->SetIntakePower(-1.0);
    shooter_->SetLinearConveyorPower(-1.0);
  } else if (operatorControl_->GetIntakeButton()) {
    intake_->SetIntakePower(1.0);
    shooter_->SetLinearConveyorPower(-1.0);
  } else {
    shooter_->SetLinearConveyorPower(0.0);
    intake_->SetIntakePower(0.0);
  }

  // Only have Teleop and AutoAlign Drivers right now
  if (leftJoystick_->GetRawButton((int)constants_->autoAlignPort) && !oldAutoAlignButton_) {
    printf("Going to AutoAlign\n");
    currDriver_ = autoAlignDriver_;
    currDriver_->Reset();
  } else if (!leftJoystick_->GetRawButton((int)constants_->autoAlignPort) && oldAutoAlignButton_) {
    // If the baselock switch has been flipped off, switch back to teleop
    printf("Teleopping\n");
    currDriver_ = teleopDriver_;
    currDriver_->Reset();
  }

  // Update the driver and the baselock switch status
  currDriver_->UpdateDriver();
  // Temp auto align control
  oldAutoAlignButton_ = leftJoystick_->GetRawButton((int)constants_->autoAlignPort);
  //oldBaseLockSwitch_ = operatorControl_->GetBaseLockSwitch();

  // If pizza wheels are up, set intake up
  
  if(!drivebase_->GetPizzaUp()) {
    intake_->SetIntakePosition(Intake::INTAKE_UP);
  } else {
    intake_->SetIntakePosition(operatorControl_->GetIntakePositionSwitch());
  }
  //intake_->SetIntakePosition(operatorControl_->GetIntakePositionSwitch());

  // LCD display
  double velocity = shooter_->GetVelocity();
  lcd_->PrintfLine(DriverStationLCD::kUser_Line1, "left:%f", drivebase_->GetLeftEncoderDistance());
  lcd_->PrintfLine(DriverStationLCD::kUser_Line2, "right:%f", drivebase_->GetRightEncoderDistance());
  lcd_->PrintfLine(DriverStationLCD::kUser_Line3, "T: %f %f", (float) target_->GetAngle(), (float) target_->GetX());
  lcd_->PrintfLine(DriverStationLCD::kUser_Line4,"Shoot: %.0f rps", shooterTargetVelocity_);
  lcd_->PrintfLine(DriverStationLCD::kUser_Line5, "Ranger: %d", ballRanger_->GetValue());
  lcd_->PrintfLine(DriverStationLCD::kUser_Line6, "Gyro: %f", gyro_->GetAngle());
  lcd_->UpdateLCD();
}
