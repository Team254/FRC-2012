#ifndef AUTO_DRIVE_COMMAND_H_
#define AUTO_DRIVE_COMMAND_H_

#include "auto/AutoCommand.h"
#include "subsystems/Drive.h"
#include "subsystems/Pid.h"

/**
 * @author: Bhargava Manja
 * This class encompasses all drive functionality in autonomous mode
 * and includes PID error handling for distance (tuned by changing constants
 * in constructor.
 */

class DriveCommand : public AutoCommand {
 public:
  /**
   * Constructor. Accepts drivetrain pointer and goal in terms of distance in inches
   */
  DriveCommand(Drive* drive, double distance);

  /**
   * Clears encoders and gyro
   */
  void Initialize();

  /**
   * Uses PID update values to set motors and reach distance goal with minimum error
   */
  bool Run();

  /**
   * Destructor
   */
 ~DriveCommand();
 private:
  //drivebase object for motor control and encoder reading functionality
  Drive* drive_;
  //PIDs for distance on both sides
  Pid* leftPid_;
  Pid* rightPid_;
  //distance goal, more goals can be added (angle, velocity, etc.)
  double distanceGoal_;
};

#endif //AUTO_DRIVE_COMMAND_H_
