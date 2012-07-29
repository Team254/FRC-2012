# Team 254's 2012 Robot Code

This repository contains the Team 254 2012 FRC codebase. This code is released
under the BSD 2-clause license. A copy of this license is included
in COPYING.

Most of the functions and classes are documented or otherwise
simple and self-explanatory. This year's code is slightly less 
convoluted than the previous year's, making it fairly easier to 
understand the logic and organization of the code structure.

## Contents

### Robot Source

The code for Team 254's 2012 robot is found in the src/ directory.

CheesyRobot.h/cpp contain a version of the IterativeRobot base class, modified to fix various
problems with loop timing.

Skyfire.h/cpp is the main class of the 2012 robot, derived from CheesyRobot. This class includes
logic for autonomous mode and interfacing with the control board in disabled
and teleoperated modes.

The auto/ directory contains various modular autonoumus commands mainly for use in autonomous mode.
These commands can be combined sequentially and concurrently to construct complex autonomous modes.
During Skyfire's autonomous period, commands are initiated and run until their completion or time
expiration.

The config/ directory contains a system for managing constants throughout the codebase. Constants
and their default values are declared in ConstantDeclarations.h, which Constants.h/cpp include in
order to declare them as public class members. These values can be overridden by a text file located
in the cRIO's filesystem which is read on startup and can be re-read on demand. This allows us to
tweak constants without re-compiling the code each time the values are changed. Constants is a
singleton class, and can be accessed from all other classes in the codebase.

drivers/ contain various drivers to control the robot's drivebase. Since
only one Driver may be used at any particular instance, users must toggle
back and forth between drivers. During teleop mode, for instance, users may
switch between teleop control driving and baselock driving by changing the 
reference of a Driver variable to point to a particular driver, allowing for
multiple ways to control the drivebase and an efficient way to switch between 
them.

matlab/ contains the code for our full state feedback control system.
matrix.c and matrix.h are a C matrix library developed by Parker Schuh;
it was chosen because we needed a library quickly and it was on hand.
In a future version we would write our own C++-style library (with
nice objects and methods and operators) for better code readability.

The subsystems/ directory holds classes that encapsulate the robot's various physical subsystems
(such as the shooter, drive, and intake) as well as classes for PID control. The Skyfire class
interacts with these subsystems to read user input with operator control and control the robot's
components.

util/ includes various utilities, such as AccelFilterBase and ContinuousAccelFilter
for autonomous drive control, a Functions class for generating various wave functions,
and classes to help in PID tuning by sending live robot data to a remote computer for graphing.

vision/ contains camera functionality, processing images of the backboard taken
by the camera to improve aiming and alignment.

### PID Tuning UI

The ui/ directory in the top level contains a utility that can be used in conjunction
with src/util/PidTuner.h/cpp to help with PID tuning. It consists of a node.js server which
captures data sent from PidTuner in real-time and exposes a web interface with graphs of the data.
See ui/README for usage details.

### PoofViewer

The poofviewer.html file in the top-level directory contains a static webpage that we use
in competition on a small display at the drivers' eye level. It shows the live feed from the
Axis camera and draws a crosshair on top.

## Functionality

Compared with our previous year's program, Skyfire's IterativeRobot-based
design and code structure is relatively more organized and easier to
understand.  Skyfire.cpp determines which competition phase we are in (disabled,
autonomous, teleop), and performs the necessary functions.  In autonomous mode,
the robot reads user input on the control board to select the appropriate
autonomous commands, allowing for sequential and concurrent execution.  During
teleop mode, the robot reads input from the control board and communicates with
the subsystems (drive, shooter, and intake) to control the robot.

If you have any questions, contact Richard Lin at [richard@team254.com](mailto:richard@team254.com).
