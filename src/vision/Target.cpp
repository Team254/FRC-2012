#include "vision/Target.h"

Target::Target() {
  task = new Task("VisionTask", (FUNCPTR) Target::VisionTask);
  task->Start();
  enabled_ = false;
}

Target::~Target(){
  task->Stop();
}

Target* Target::GetInstance() {
  if (!instance_) {
    instance_ = new Target();
  }
  return instance_;
}

void Target::VisionTask() {
  Target* t = Target::GetInstance();
  while (true) {
    t->FindTarget();
  }
}

void Target::Start() {
  enabled_ = true;
}

void Target::Stop() {
  enabled_ = false;
}

void Target::FindTarget() {
  static int i = 0;
  i++;
  AxisCamera &camera = AxisCamera::GetInstance("10.2.54.90");
  DriverStationLCD::GetInstance()->Printf(DriverStationLCD::kUser_Line1,1,"i: %d" , i);
  DriverStationLCD::GetInstance()->UpdateLCD();
}

double Target::GetX() {
  return x_;
}

int Target::GetDistance() {
  return distance_;
}

bool Target::CanSeeTarget() {
  return seesTarget_;
}

