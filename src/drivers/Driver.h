#ifndef DRIVERS_DRIVER_H
#define DRIVERS_DRIVER_H

class Drive;

/**
 * @author Eric Bakan
 *
 * Represents a single drivebase controller
 * Other controllers for teleop, autonomous driving in teleop mode, etc. will be based off this class
 *
 * Only one driver will be active at any given time, ensuring that only one subsystem is actively
 * trying to control the drivebase
 *
 * This class/design is used in the following way:
 * MainRobot has a Driver* member variable for the current drive controller and then other Driver* subclasses
 * to represent other subsystems. Then given the current inputs and the current controller's status,
 * MainRobot can change the Driver* member variable to point to the correct controller and run it
 *
 * For example, the current Driver might be receiving input from the joysticks for teleop control. But then
 * the baselock enable switch is toggled so MainRobot assign the current Driver to a different BaselockDriver
 * or whatever. Then once the switch is toggled back, MainRobot will switch the current Driver to point to the
 * teleop control Driver.
 */
class Driver {
 public:

  /**
   * All drivers need a drive to drive!
   * Other things like joysticks, etc. can come later
   */
  Driver(Drive* drive);

  /**
   * Updates the driver
   * All logic goes in here - setting motor values, reading joysticks, etc.
   * @return true if the driver is done/at its goal, else false
   */
  virtual bool UpdateDriver() { return true; }

  /**
   * Resets the Driver.
   * Useful if the controller has a PID or other heuristic elements
   */
  virtual void Reset() {}

  /**
   * Clean up clean up, everybody everywhere!
   * Clean up clean up, everybody do your share!
   */
  virtual ~Driver() {}

 protected:
  Drive* drive_;

};

#endif // DRIVERS_DRIVER_H
