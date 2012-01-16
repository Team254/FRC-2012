/**
  * Represents a command to be executed in Autonomous Mode
  * Is the base class for all other Autonomous Commands (driving, shooting, etc.)
  * @author Eric Bakan
  */
class AutoCommand {
 public:
  /**
    * Initialization method
    * Is called the first time the AutoCommand is processed
    * Used to set control loop goals, etc.
    */
  virtual void Initialize();

  /**
    * Run method
    * Is called every time the autonomous loop iterates
    * Used to update control loops, etc.
    * @return true if the command is completed, otherwise false
    */
  virtual bool Run() = 0;

  /**
    * Destructor, doesn't do much...
    */
  virtual ~AutoCommand();
};

