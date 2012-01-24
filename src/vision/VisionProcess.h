#ifndef VISION_TARGET_H_
#define VISION_TARGET_H_

#include "WPILib.h"

class VisionProcess {
 public:
  static void VisionTask(VisionProcess*);
  void Start();
  void Stop();
  VisionProcess();
  virtual ~VisionProcess();

 private:
  virtual void DoVision();
  bool enabled_;
  Task* task;
};

#endif
