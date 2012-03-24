#ifndef AUTO_CONCURRENT_COMMAND_H_
#define AUTO_CONCURRENT_COMMAND_H_

#include <vector>
#include <utility>

#include "auto/AutoCommand.h"

// Macro to wrap the ConcurrentCommand constructor for convenience.
#define AUTO_CONCURRENT(...) new ConcurrentCommand(0, __VA_ARGS__, NULL)

/**
 * @author Eric Bakan
 *
 * Meta-command used for running multiple commands concurrently
 */
class ConcurrentCommand : public AutoCommand {
 public:
  /**
   * Constructor
   * Accepts a variable-length list of AutoCommand pointers to execute concurrently
   * The list of commands MUST be followed by another argument with value NULL, so that the number of commands
   * can be inferred.
   *
   * DO NOT CALL THIS CONSTRUCTOR DIRECTLY -- USE THE AUTO_CONCURRENT() MACRO.
   *
   * @param dummy a dummy argument to satisfy the requirement that there is at least one
   * @param ... the commands to run
   */
  ConcurrentCommand(int dummy, ...);

  /**
   * Destructor
   * Deallocates the AutoCommands stored in commands_
   */
  virtual ~ConcurrentCommand();

  /**
   * Calls Initialize() on all the AutoCommands in commands_
   */
  virtual void Initialize();

  /**
   * Calls Run() on all the unfinished AutoCommands in commands_
   * Updates the bool of each CommandPair to true once the command is completed
   * @return true if all of the commands are complete, else false
   */
  virtual bool Run();

  /**
   * Adds another command to the commands_ vector
   * @param command the command to pass in
   */
  void AddCommand(AutoCommand* command);

 private:
  // CommandPair represents a command and its completion state.
  // The completion state is initalized to false and is updated every time Run() is called.
  typedef std::pair<AutoCommand*, bool> CommandPair;
  std::vector<CommandPair> commands_;
};

#endif  // AUTO_CONCURRENT_COMMAND_H_
