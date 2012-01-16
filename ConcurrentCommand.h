#include <vector>
#include "AutoCommand.h"

class ConcurrentCommand {
 public:
  ConcurrentCommand(...);
  virtual void Initialize();
  virtual bool Run();
  virtual ~ConcurrentCommand();
 private:
  std::vector<pair<AutoCommand*, bool> > commands_;
};

