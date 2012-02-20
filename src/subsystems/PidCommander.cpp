#include "PidCommander.h"

PidCommander::PidCommander(Pid* pid, double g, double attackPercent, double maxDeltaG) {
  pid_ = pid;
  goal_ = g;
  attackPercentage_ = attackPercent;
  decayPercentage_ = 1.0 - attackPercent;
  // make sure this doesnt break
  maxDeltaG_ = maxDeltaG;
  firstRun_ = true;
}

double PidCommander::Update(double cur) {
  double deltaG = 0;
  double minDeltaG  = maxDeltaG_ / 10.0;
  if (firstRun_) {
    firstVal_ = cur;
    firstRun_ = false;

    if (goal_ > cur)
      direction_ = 1.0;
    else
      direction_ = -1.0;
  }

  double range = goal_ - firstVal_;
  cur = cur - firstVal_;
  
  double signedCur = cur;
  cur = fabs(cur);
  range = fabs(range);

  if (cur < attackPercentage_ * range) { //attack
    deltaG = (((maxDeltaG_ - minDeltaG) / (attackPercentage_ * range)) * cur) + minDeltaG;
  }
  else if (cur < decayPercentage_ * range) { // top of trapezoid
    deltaG = maxDeltaG_;
  }
  else { //decay
    double curTemp = cur - (decayPercentage_ * range);
    deltaG = ((-maxDeltaG_ / ((1.0 - decayPercentage_) * range)) * curTemp) + maxDeltaG_;
  }

  Logger::GetSysLog()->Log("PidCommander: cur: %f, deltaG: %f\n\n", (float) cur, deltaG);
  double newgoal = signedCur + (deltaG * direction_);
  
  return pid_->Update(newgoal, cur*direction_);
}
