#ifndef DELAY_COMMAND_H_
#define DELAY_COMMAND_H_

#include "auto/AutoCommand.h"

class DelayCommand : public AutoCommand {

 public:
  DelayCommand(double timeout);
  void Initialize();
  bool Run();
  ~DelayCommand();

};

#endif

