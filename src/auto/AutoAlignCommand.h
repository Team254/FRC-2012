#ifndef AUTO_AUTO_ALIGN_COMMAND_H_
#define AUTO_AUTO_ALIGN_COMMAND_H_

#include "auto/AutoCommand.h"

class Drive;
class AutoTurnDriver;


class AutoAlignCommand : public AutoCommand {

 public:
  
  AutoAlignCommand(Drive* drive, AutoTurnDriver* autoTurn, double offset, double timeout);
  
  void Initialize();
  
  bool Run();
  
  ~AutoAlignCommand();
  
 private:
  
  Drive* drive_;
  AutoTurnDriver* autoTurn_;
  double offset_;
  
};

#endif 

