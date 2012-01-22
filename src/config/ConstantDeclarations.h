/* @author Eric Caldwell
 *
 * This file is read by Constants and contains the variables and default values that will be stored
 * as member variables in Constants. Use it by including it in the macro definition of the functions
 * used below.
 */

// Motors
DECLARE_DOUBLE(leftMotorPortA, 5)
DECLARE_DOUBLE(leftMotorPortB, 6)
DECLARE_DOUBLE(rightMotorPortA, 3)
DECLARE_DOUBLE(rightMotorPortB, 4)

// Sensors
DECLARE_DOUBLE(leftEncoderPortA, 2)
DECLARE_DOUBLE(leftEncoderPortB, 3)
DECLARE_DOUBLE(rightEncoderPortA, 4)
DECLARE_DOUBLE(rightEncoderPortB, 5)
DECLARE_DOUBLE(gyroPort, 1)

// Vision
DECLARE_DOUBLE(thresholdHMin, 0)
DECLARE_DOUBLE(thresholdHMax, 255)
DECLARE_DOUBLE(thresholdSMin, 0)
DECLARE_DOUBLE(thresholdSMax, 255)
DECLARE_DOUBLE(thresholdVMin, 0)
DECLARE_DOUBLE(thresholdVMax, 255)


// Control Board
DECLARE_DOUBLE(leftJoystickPort, 1)
DECLARE_DOUBLE(rightJoystickPort, 2)

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
DECLARE_DOUBLE(baseLockPort, 17)
DECLARE_DOUBLE(manualOverridePort, 18)
DECLARE_DOUBLE(bridgeLowererPort, 19)

// Linearization
DECLARE_DOUBLE(linearCoeffA, 3.7436)
DECLARE_DOUBLE(linearCoeffB, -5.6125)
DECLARE_DOUBLE(linearCoeffC, 2.3487)
DECLARE_DOUBLE(linearCoeffD, -0.0827)
DECLARE_DOUBLE(linearCoeffE, 0.0913)
