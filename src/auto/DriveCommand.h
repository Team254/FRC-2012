#ifndef AUTO_DRIVE_COMMAND_H_
#define AUTO_DRIVE_COMMAND_H_

#include "auto/AutoCommand.h"
#include "util/MovingAverageFilter.h"

class Drive;
class Pid;
class ss_controller;
class AccelFilterBase;
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
	// coast is defunct - take it out when we refactor
  DriveCommand(Drive* drive, double distance, double angle, bool usePizza, double timeout, double maxSpeed=240.0, double maxAcceleration=120.0, double maxAlpha=180.0, double maxOmega=120.0);

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
  
  //Timer for velocity calcs
  Timer* brakeTimer_;

  // Distance goal, more goals can be added (angle, velocity, etc.)
  double distanceGoal_;
  double angleGoal_;

  // Should pizza wheels be down?
  bool usePizza_;
  bool resetPizza_;
  double maxSpeed_;
  double maxAcceleration_;
  double maxAlpha_;
  double maxOmega_;
  double turnOffset_;
  double sumStoppedError_;
  
  // State space stuff
  double curA_;
  double curV_;
  double curX_;
  double curJeez_; // Jesusfish
  double curWubl_; // Wubbleu
  double curThet_; // Theta
  struct matrix *y_;
  struct matrix *r_;
  ss_controller *ssc_;
  AccelFilterBase* straightFilter_;
  AccelFilterBase* turnFilter_;
  
  // Old
  Pid* leftPid_;
  Pid* rightPid_;
  Timer* driveTimer_;
  double prevTime_;
  double prevLeftDist_;
  double prevRightDist_;
  double startingAngle_;
};

#endif  // AUTO_DRIVE_COMMAND_H_
