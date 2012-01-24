#ifndef BACKBOARD_FINDER_H_
#define BACKBOARD_FINDER_H_

#include "vision/VisionProcess.h"

class BackboardFinder : public VisionProcess {
 public:
  double GetX();
  double GetDistance();
  double GetAngle();
  bool SeesTarget();
  void DoVision();
  
 private:
  double x_;
  double distance_;
  double angle_;
  bool seesTarget_;
};


#endif
