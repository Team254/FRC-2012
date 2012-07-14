#ifndef SUBSYSTEMS_PIDCOMMANDER_H_
#define SUBSYSTEMS_PIDCOMMANDER_H_
#include "Pid.h"
#include <math.h>
#include <iostream>
#include "../util/Logger.h"

class PidCommander {
 public:
  PidCommander(Pid* pid, double g, double attackPercent, double maxDeltaG);
  double Update (double cur);

 private:
  Pid* pid_;
  double goal_;
  double firstVal_;
  bool firstRun_;
  double direction_; // 1.0
  // Trapezoid
  double attackPercentage_;
  double decayPercentage_;
  double maxDeltaG_;
};

#endif //SUBSYSTEMS_PIDCOMMANDER_H_

