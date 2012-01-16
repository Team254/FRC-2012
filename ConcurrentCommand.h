#include "AutoCommand.h"
#include <vector>
#include <utility>

class ConcurrentCommand  : virtual AutoCommand {
 public:
  ConcurrentCommand(int numCommands, ...);
  virtual ~ConcurrentCommand();
  virtual void Initialize();
  virtual bool Run();
 private:
  // vector of pairs of each command and its completion status
  // these are initialized as false and updated to true as
  // commands are completed
  typedef std::pair<AutoCommand*, bool> commandPair;
  std::vector<commandPair> commands_;
};

