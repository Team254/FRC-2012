#include <vector>
#include <utility>

#include "AutoCommand.h"

class SequentialCommand : public AutoCommand {
 public:
  SequentialCommand(int numCommands, ...);
  virtual ~SequentialCommand();
  virtual void Initialize();
  virtual bool Run();
 private:
  std::vector<AutoCommand*> commands_;
  int commandIndex_;
};

