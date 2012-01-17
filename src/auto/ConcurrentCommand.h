#ifndef AUTO_CONCURRENT_COMMAND_H_
#define AUTO_CONCURRENT_COMMAND_H_

#include <vector>
#include <utility>

#include "auto/AutoCommand.h"

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
   * @param numCommands the number of commands to execute
   * @param ... the commands to run
   */
  ConcurrentCommand(int numCommands, ...);

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
 private:
  // CommandPair represents a command and its completion state.
  // The completion state is initalized to false and is updated every time Run() is called.
  typedef std::pair<AutoCommand*, bool> CommandPair;
  std::vector<CommandPair> commands_;
};

#endif  // AUTO_CONCURRENT_COMMAND_H_
