/**
 * @author Eric Caldwell
 *
 * This file is read by Constants and contains the variables and default values that will be stored as member
 * variables in Constants. Use it by including it in the macro definition of the functions used below.
 */

// Motors
// NOTE(patrick): PWMs 1 and 10 currently unused.
DECLARE_DOUBLE(leftDrivePwmA, 5)
DECLARE_DOUBLE(leftDrivePwmB, 4)
DECLARE_DOUBLE(rightDrivePwmA, 6)
DECLARE_DOUBLE(rightDrivePwmB, 7)
DECLARE_DOUBLE(intakePwm, 2)
DECLARE_DOUBLE(conveyorPwm, 9)
DECLARE_DOUBLE(leftShooterPwm, 3)
DECLARE_DOUBLE(rightShooterPwm, 8)

// Sensors
DECLARE_DOUBLE(leftEncoderPortA, 2)
DECLARE_DOUBLE(leftEncoderPortB, 3)
DECLARE_DOUBLE(rightEncoderPortA, 11)
DECLARE_DOUBLE(rightEncoderPortB, 12)
DECLARE_DOUBLE(shooterEncoderPortA, 0)
DECLARE_DOUBLE(shooterEncoderPortB, 0)
DECLARE_DOUBLE(gyroPort, 0)
DECLARE_DOUBLE(accelerometerXPort, 0)
DECLARE_DOUBLE(accelerometerYPort, 0)
DECLARE_DOUBLE(accelerometerZPort, 0)

// Pneumatics
DECLARE_DOUBLE(compressorPressureSwitchPort,0)
DECLARE_DOUBLE(compressorRelayPort,0)
DECLARE_DOUBLE(shiftSolenoidPort,0)
DECLARE_DOUBLE(hoodSolenoidPort,0)
DECLARE_DOUBLE(pizzaWheelSolenoidHighPort,0)
DECLARE_DOUBLE(pizzaWheelSolenoidLowPort,0)
DECLARE_DOUBLE(intakeSolenoidHighPort,0)
DECLARE_DOUBLE(intakeSolenoidLowPort,0)

// Vision
DECLARE_DOUBLE(thresholdRMin, 0)
DECLARE_DOUBLE(thresholdRMax, 50)
DECLARE_DOUBLE(thresholdGMin, 0)
DECLARE_DOUBLE(thresholdGMax, 50)
DECLARE_DOUBLE(thresholdBMin, 80)
DECLARE_DOUBLE(thresholdBMax, 255)

// Control Board
DECLARE_DOUBLE(leftJoystickPort, 1)
DECLARE_DOUBLE(rightJoystickPort, 2)
DECLARE_DOUBLE(operatorControlPort, 3)
DECLARE_DOUBLE(conveyorUpPort, 3)
DECLARE_DOUBLE(conveyorDownPort, 4)
DECLARE_DOUBLE(conveyorIndexPort, 5)
DECLARE_DOUBLE(fineControlLeftPort, 6)
DECLARE_DOUBLE(fineControlRightPort, 7)
DECLARE_DOUBLE(intakeDeployPort, 8)
DECLARE_DOUBLE(presetFenderPort, 9)
DECLARE_DOUBLE(presetKeyPort, 10)
DECLARE_DOUBLE(presetBridgePort, 11)
DECLARE_DOUBLE(presetHalfCourtPort, 12)
DECLARE_DOUBLE(presetMaxPort, 13)
DECLARE_DOUBLE(shooterPort, 14)
DECLARE_DOUBLE(hoodIncrementPort, 15)
DECLARE_DOUBLE(hoodDecrementPort, 16)
DECLARE_DOUBLE(baseLockPort, 1)
DECLARE_DOUBLE(manualOverridePort, 18)
DECLARE_DOUBLE(bridgeLowererPort, 19)
DECLARE_DOUBLE(highGearPort, 2);

// Linearization
DECLARE_DOUBLE(linearCoeffA, 4.5504)
DECLARE_DOUBLE(linearCoeffB, -5.9762)
DECLARE_DOUBLE(linearCoeffC, 2.5895)
DECLARE_DOUBLE(linearCoeffD, -0.0869)
DECLARE_DOUBLE(linearCoeffE, 0.0913)

// PID constants
DECLARE_DOUBLE(driveKP, 1.2)
DECLARE_DOUBLE(driveKI, 0.003)
DECLARE_DOUBLE(driveKD, 8)
DECLARE_DOUBLE(baseLockKP, 1.5)
DECLARE_DOUBLE(baseLockKI, 0.01)
DECLARE_DOUBLE(baseLockKD, 0)
DECLARE_DOUBLE(shooterKP, 1.0)
DECLARE_DOUBLE(shooterKI, 0.0)
DECLARE_DOUBLE(shooterKD, 0.0)
