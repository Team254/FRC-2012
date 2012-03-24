#ifndef AUTO_SEQUENTIAL_COMMAND_H_
#define AUTO_SEQUENTIAL_COMMAND_H_

#include <vector>
#include <utility>

#include "auto/AutoCommand.h"

// Macro to wrap the SequentialCommand constructor for convenience.
#define AUTO_SEQUENTIAL(...) new SequentialCommand(0, __VA_ARGS__, NULL)

/**
 * @author Eric Bakan
 *
 * Meta-command used for running multiple commands sequentially
 */
class SequentialCommand : public AutoCommand {
 public:
  /**
   * Constructor
   * Accepts a variable-length list of AutoCommand pointers to execute sequentially
   * The commands will be executed in the order they are passed into the constructor. The list of commands
   * MUST be followed by another argument with value NULL, so that the number of commands can be inferred.
   *
   * DO NOT CALL THIS CONSTRUCTOR DIRECTLY -- USE THE AUTO_SEQUENTIAL() MACRO.
   *
   * @param dummy a dummy argument to satisfy the requirement that there is at least one
   * @param ... the commands to run
   */
  SequentialCommand(int dummy, ...);

  /**
   * Destructor
   * Deallocates the AutoCommands stored in commands_
   */
  virtual ~SequentialCommand();

  /**
   * Calls Initialize() on the first AutoCommand in commands_
   */
  virtual void Initialize();

  /**
   * Calls Run() on the current command
   * If the current command is complete, calls Initialize() on the next one
   * @return true if the last command is complete, else false
   */
  virtual bool Run();

  /**
   * Adds another command to the commands_ vector
   * @param command the command to pass in
   */
  void AddCommand(AutoCommand* command);
 private:
  std::vector<AutoCommand*> commands_;
  int commandIndex_; // the index of the current command being executed
};

#endif  // AUTO_SEQUENTIAL_COMMAND_H_
