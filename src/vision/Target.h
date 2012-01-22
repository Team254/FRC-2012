#ifndef VISION_TARGET_H_
#define VISION_TARGET_H_

#include "WPILib.h"

class Target {
 public:
  static Target* GetInstance();
  double GetX();
  int GetDistance();
  bool CanSeeTarget();
  static void VisionTask();
  void Start();
  void Stop();

 private:
  void FindTarget();
  Target();
  ~Target();

  int distance_;
  double x_;
  bool seesTarget_;
  bool enabled_;
  Task* task;
  static Target* instance_;
};

#endif
