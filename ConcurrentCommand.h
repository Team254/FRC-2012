#include <vector>
#include <utility>

#include "AutoCommand.h"

class ConcurrentCommand : public AutoCommand {
 public:
  ConcurrentCommand(int numCommands, ...);
  virtual ~ConcurrentCommand();
  virtual void Initialize();
  virtual bool Run();
 private:
  // vector of pairs of each command and its completion status
  // these are initialized as false and updated to true as
  // commands are completed
  typedef std::pair<AutoCommand*, bool> CommandPair;
  std::vector<CommandPair> commands_;
};

