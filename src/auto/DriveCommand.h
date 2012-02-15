#ifndef AUTO_DRIVE_COMMAND_H_
#define AUTO_DRIVE_COMMAND_H_

#include "auto/AutoCommand.h"

class Drive;
class Pid;

/**
 * @author Bhargava Manja
 *
 * This class encompasses all drive functionality in autonomous mode and includes PID error handling for
 * distance.
 */
class DriveCommand : public AutoCommand {
 public:
  /**
   * Constructor. Accepts drivetrain pointer and goal in terms of distance in inches
   */
  DriveCommand(Drive* drive, double distance, bool usePizza);

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
  // Drivebase object for motor control and encoder reading functionality
  Drive* drive_;

  // PIDs for distance on both sides
  Pid* leftPid_;
  Pid* rightPid_;

  // Distance goal, more goals can be added (angle, velocity, etc.)
  double distanceGoal_;

  // Should pizza wheels be down?
  bool usePizza_;
  bool resetPizza_;
};

#endif  // AUTO_DRIVE_COMMAND_H_
