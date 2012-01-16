#include <vector>
#include "AutoCommand.h"

class SequentialCommand {
 public:
  SequentialCommand(...);
  virtual void Initialize();
  virtual bool Run();
  virtual ~SequentialCommand();
 private:
  std::vector<AutoCommand*> commands_;
  int commandIndex_;
};

