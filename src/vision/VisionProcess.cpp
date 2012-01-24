#include "vision/VisionProcess.h"
#include "config/Constants.h"

VisionProcess::VisionProcess() {
  task = new Task("VisionTask", (FUNCPTR) VisionProcess::VisionTask);
  task->Start((UINT32) this);
  enabled_ = false;
}

VisionProcess::~VisionProcess(){
  task->Stop();
}

void VisionProcess::VisionTask(VisionProcess* vp) {
  while (true) {
    if (vp->enabled_)
      vp->DoVision();
  }
}

void VisionProcess::Start() {
  enabled_ = true;
}

void VisionProcess::Stop() {
  enabled_ = false;
}

void VisionProcess::DoVision() {
  // Override me
}
