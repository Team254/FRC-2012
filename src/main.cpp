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
#include "drivers/AutoTurnDriver.h"
#include "auto/SequentialCommand.h"
#include "auto/ShootCommand.h"
#include "auto/DriveCommand.h"
#include "auto/BridgeBallsCommand.h"
#include "auto/TurnCommand.h"

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
  gyro_->SetSensitivity(constants_->gyroSensitivity);
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

  // Vision Tasks
  target_ = new BackboardFinder();
  //target_->Start();
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
  oldShooterUpSwitch_ = false;
  oldShooterDownSwitch_ = false;
  oldBallQueueSwitch_ = false;
  autoBaseCmd_ = NULL;
}

void MainRobot::DisabledInit() {
	drivebase_->ResetEncoders();
	drivebase_->ResetGyro();
}

void MainRobot::AutonomousInit() {

  constants_->LoadFile();
  GetWatchdog().SetEnabled(false);
  power_ = 0;
  if(autoBaseCmd_) {
      delete autoBaseCmd_;
  }
  /*
  autoBaseCmd_ = new SequentialCommand(5, new ShootCommand(shooter_, intake_, false, 3.75),
		             new DriveCommand(drivebase_, 50,  false),
		             new BridgeBallsCommand(intake_, shooter_, 5.0),
		             new DriveCommand(drivebase_, -50, false),
		             new ShootCommand(shooter_, intake_, true, 10.0)
                 );
                 
  autoBaseCmd_ = new SequentialCommand(2, new DriveCommand(drivebase_, 150, false),
		                                  new DriveCommand(drivebase_, -150, false));*/
  autoBaseCmd_ = new SequentialCommand(1, new TurnCommand(drivebase_, 90, 10));
  autoBaseCmd_->Initialize();
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
  
  lcd_->PrintfLine(DriverStationLCD::kUser_Line4, "Gyro: %f\n", gyro_->GetAngle());
  lcd_->PrintfLine(DriverStationLCD::kUser_Line5, "Sens: %f\n", constants_->gyroSensitivity);
  if(operatorControl_->GetIntakeButton()) {
	  constants_->LoadFile();
	  gyro_->SetSensitivity(constants_->gyroSensitivity);
	  gyro_->Reset();
  }
  lcd_->PrintfLine(DriverStationLCD::kUser_Line1, "Convey: %d", conveyorEncoder_->Get());
  lcd_->PrintfLine(DriverStationLCD::kUser_Line2, "Ball: %d", conveyorBallSensor_->GetValue());
  lcd_->PrintfLine(DriverStationLCD::kUser_Line3, "Poof: %d", poofMeter_->GetValue());
  lcd_->UpdateLCD();
}

void MainRobot::AutonomousPeriodic() {
  autoBaseCmd_->Run();
}

void MainRobot::TeleopPeriodic() {
  GetWatchdog().Feed();
  drivebase_->SetBrakeOn(false);

  /*
  // Ghetto shooter control for testing
  if (xbox->GetRawButton(10)) {
    shooterTargetVelocity_ = 0;
  } else if (xbox->GetRawButton(9) && !oldShooterDownSwitch_) {
    shooterTargetVelocity_ -= 1;
  } else if (xbox->GetRawButton(8) && !oldShooterUpSwitch_) {
    shooterTargetVelocity_ += 1;
  } else if (xbox->GetRawButton(7)) {
    shooterTargetVelocity_ = 38;
  }
  */
  static bool oldShooterSwitch= operatorControl_->GetShooterSwitch();
  static bool increaseButton = operatorControl_->GetIncreaseButton();
  static bool decreaseButton = operatorControl_->GetDecreaseButton();
    
  static int counter = 0;
  static Logger* driveLog = new Logger("/driveLog.log");
  if(counter % 10 == 0) {
    double left = drivebase_->GetLeftEncoderDistance();
    double right = drivebase_->GetRightEncoderDistance();
    driveLog->Log("Angle: %f, Distance: %f\n", drivebase_->GetGyroAngle(), (left - right)/2);
    target_->DoVision();
  }
  counter++;
  if(operatorControl_->GetShooterSwitch()) {
  	  printf("switch on\n");
  	  if(!oldShooterSwitch) {
  		  shooterTargetVelocity_ = 38;
  		  printf("flipped\n");
  	  } else {
  		  printf("lawlnope\n");
  		  if(operatorControl_->GetIncreaseButton() && !increaseButton) {
  		  	  shooterTargetVelocity_+=1;
  		    } else if(operatorControl_->GetDecreaseButton() && !decreaseButton) {
  		  	  shooterTargetVelocity_-=1;
  		    }
  	  }
    } else {
  	  shooterTargetVelocity_ = 0;
    }
  
  
  
  oldShooterSwitch=operatorControl_->GetShooterSwitch();
  increaseButton = operatorControl_->GetIncreaseButton();
  decreaseButton = operatorControl_->GetDecreaseButton();
  
  oldShooterUpSwitch_ = xbox->GetRawButton(8);
  oldShooterDownSwitch_ = xbox->GetRawButton(9);
  shooter_->SetTargetVelocity(shooterTargetVelocity_);
  bool shooterDone = shooter_->PIDUpdate();
  
  if(operatorControl_->GetShooterSwitch() && shooterDone && 
     operatorControl_->GetAutoShootButton()) {
	  shooter_->SetLinearConveyorPower(1);
	  intake_->SetIntakePower(1);
  } else if (operatorControl_->GetAutoShootButton()) {
	  shooter_->QueueBall();
	  intake_->SetIntakePower(1);
  } else if(operatorControl_->GetShootButton()) {
  	  shooter_->SetLinearConveyorPower(1.0);
  	  intake_->SetIntakePower(1.0);
    } else if(operatorControl_->GetUnjamButton()) {
  	  intake_->SetIntakePower(-1.0);
  	  shooter_->SetLinearConveyorPower(-1.0);
    } else if(operatorControl_->GetIntakeButton()) {
  	  intake_->SetIntakePower(1.0);
  	  shooter_->SetLinearConveyorPower(-1.0);
    } else {
  	  shooter_->SetLinearConveyorPower(0.0);
  	  intake_->SetIntakePower(0.0);
    }
  
/* queueing
  // Automatic ball queueing control.
  if (xbox->GetTwist() < -0.75) {
    //    if (shooter_->QueueBall() && !oldBallQueueSwitch_) {
    //      shooter_->ShootBall();
    //    }
    intake_->SetIntakePower(1.0);
  } else {
    shooter_->SetLinearConveyorPower(0);
    intake_->SetIntakePower(0);
  }
  */
  oldBallQueueSwitch_ = (xbox->GetTwist() < -0.75);

  intake_->SetIntakePosition(operatorControl_->GetIntakePositionSwitch());


  ledRingSwitch_->Set(shooterTargetVelocity_ > 0);

  // Only have Teleop and Baselock Drivers right now
  if (operatorControl_->GetFenderButton() && !oldAutoAlignButton_) {
	  printf("Going to AutoAlign\n");
    currDriver_ = autoAlignDriver_;
    currDriver_->Reset();
  } else if (!operatorControl_->GetFenderButton() && oldAutoAlignButton_) {
      // If the baselock switch has been flipped off, switch back to teleop
	  printf("Teleopping\n");
      currDriver_ = teleopDriver_;
      currDriver_->Reset();
  }


  // Update the driver and the baselock switch status
  currDriver_->UpdateDriver();
  oldAutoAlignButton_ = operatorControl_->GetFenderButton();
  //oldBaseLockSwitch_ = operatorControl_->GetBaseLockSwitch();


  double velocity = shooter_->GetVelocity();
  lcd_->PrintfLine(DriverStationLCD::kUser_Line3,"Vel: %f", velocity);
  lcd_->PrintfLine(DriverStationLCD::kUser_Line4,"Shoot: %.0f%%", shooterTargetVelocity_);
  lcd_->PrintfLine(DriverStationLCD::kUser_Line5, "Ranger: %d", ballRanger_->GetValue());
  lcd_->UpdateLCD();
  //  
}
