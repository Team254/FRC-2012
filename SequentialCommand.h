#include "AutoCommand.h"
#include <vector>
#include <utility>

class SequentialCommand : virtual AutoCommand {
 public:
  SequentialCommand(int numCommands, ...);
  virtual ~SequentialCommand();
  virtual void Initialize();
  virtual bool Run();
 private:
  std::vector<AutoCommand*> commands_;
  int commandIndex_;
};

