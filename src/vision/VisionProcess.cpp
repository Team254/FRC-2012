#include "vision/VisionProcess.h"

VisionProcess::VisionProcess() {
  task_ = new Task("VisionTask", (FUNCPTR)VisionProcess::VisionTask, 200);
  task_->Start((UINT32)this);
  enabled_ = false;
}

VisionProcess::~VisionProcess(){
  task_->Stop();
}

void VisionProcess::VisionTask(VisionProcess* vp) {
  while (true) {
    if (vp->enabled_) {
      vp->DoVision();
    }
  }
}

void VisionProcess::Start() {
  enabled_ = true;
}

void VisionProcess::Stop() {
  enabled_ = false;
}
