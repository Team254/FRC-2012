#ifndef AUTO_AUTO_ALIGN_COMMAND_H_
#define AUTO_AUTO_ALIGN_COMMAND_H_

#include "auto/AutoCommand.h"
#include "vision/BackboardFinder.h"

class Drive;
class AutoTurnDriver;


class AutoAlignCommand : public AutoCommand {

 public:

  AutoAlignCommand(Drive* drive, AutoTurnDriver* autoTurn, BackboardFinder* target, double offset, double timeout, bool useSkew=true);

  void Initialize();

  bool Run();

  ~AutoAlignCommand();

 private:

  Drive* drive_;
  AutoTurnDriver* autoTurn_;
  double offset_;
  bool useSkew_;
  BackboardFinder* target_;
  Timer* resetTimer_;
};

#endif

