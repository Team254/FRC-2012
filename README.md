# Overview

---

Enclosed is the Team 254 2012 FRC Codebase. This code is released
under the BSD 2-Clause License. A copy of this license is included
in the COPYING file.

Most of the functions and classes are documented or otherwise
simple and self-explanatory. This year's code is slightly less 
convoluted than the previous year's, making it fairly easier to 
understand the logic and organization of the code structure.

## Contents
AUTHORS lists the main contributors to this codebase

COPYING contains a copy of the BSD License

Skyfire.cpp includes our IterativeRobot-derived class, including logic for
autonomous mode and interfacing with the control board in disbaled
and teleoperated modes.

auto/ contains various autonoumus commands mainly for use in autonomous mode,
allowing for sequentual and concurrent commands. During the IterativeRobot's
autonomous period, commands are initiated and run until their completion or time 
expiration.

config/ includes our constants, which are declared under ConstantDeclarations.
These constants are read from a file, which allows us to avoid re-compiling
the code each time the values are changed. Constants is a singleton class,
and can be accessed from other classes.

drivers/ contain various drivers to control the robot's drivebase. Since
only one Driver may be used at any particular instance, users must toggle
back and forth between drivers. During teleop mode, for instance, users may
swtich between teleop control driving and baselock driving by changing the 
reference of a Driver variable to point to a particular driver, allowing for
multiple ways to control the drivebase and an efficient way to switch between 
them.

matlab/ contains the code for our full state feedback control system.
matrix.c and matrix.h are a C matrix library developed by Parker Schuh;
it was chosen because we needed a library quickly and it was on hand.
In a future version we would write our own C++-style library (with
nice objects and methods and operators) for better code readability.

subsystems/ holds the robot's various physical subsystems (such as the shooter,
drive, and intake) as well as PID control.  Classes can interact with these
subsystems to read user input with operator control and control the robot's
components.

util/ includes various utilities, such as AccelFilterBase and ContinuousAccelFilter
for autonomous drive control, Functions for generating various functions of time,
and PIDTuner to connect to the robot and give PID output.

vision/ contains camera functionality, processing images of the backboard taken
by the camera to improve aiming and alignment.

## Functionality
Skyfire.cpp creates an IterativeRobot, and is a subclass of CheesyRobot
The robot then instantiates its motors, sensors, and pneumatics,
the joysticks and OperatorControl for reading control board values,
three different drivers for switching between driver control methods,
the Shooter, Driver, and Intake to control the robot functions,
and a Constants instance for access to pre-established constant values.

utils.hpp includes frequently used utility functions.

CheesyRobot implements the RobotBase class, and provides a number of
functionalities used by Skyfire.cpp.  CheesyRobot establishes a period
of the periodic function calls and manages the frequency of the calls.

util/AccelFilterBase and ContinuousAccelFilter are acceleration profile 
generators used in autonomous drive control.

config/Constants reads values from a file for use in the program, as declared
in ConstantDeclarations. This enables us to not have to recompile code 
to modify constants when tuning.  The constants file is re-loaded during
autonomous and teleop init.

driver/OperatorControl reads values from the control board, handles much of
its logic (semi-autonomous actions, running filters on joystick inputs,
etc. In its update loop ControlBoard updates RobotState with all its
values - control loop targets, statuses of toggle switches, etc.

Compared with our previous year's program, Skyfire's IterativeRobot-based
design and code structure is relatively more organized and easier to
understand.  Skyfire.cpp determines which competition phase we are in (disabled,
autonomous, teleop), and performs the necessary functions.  In autonomous mode,
the robot reads user input on the control board to select the appropriate
autonomous commands, allowing for sequential and concurrent execution.  During
teleop mode, the robot reads input from the control board and communicates with
the subsystems (drive, shooter, and intake) to control the robot.

If you have any questions, contact Richard Lin at [richard@team254.com](mailto:richard@team254.com)