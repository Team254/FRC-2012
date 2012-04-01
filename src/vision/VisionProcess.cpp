#include "vision/VisionProcess.h"

VisionProcess::VisionProcess() {
  task_ = new Task("VisionTask", (FUNCPTR)VisionProcess::VisionTask, 200);
  task_->Start((UINT32)this);
  enabled_ = false;
  timer_ = new Timer();
}

VisionProcess::~VisionProcess(){
  task_->Stop();
}

void VisionProcess::VisionTask(VisionProcess* vp) {
  while (false) {
    if (vp->enabled_ && vp->timer_->Get() > (1.0 / 30.0)) {
      vp->DoVision();
      vp->timer_->Reset();
    }
  }
}

void VisionProcess::Start() {
  enabled_ = true;
  timer_->Start();
}

void VisionProcess::Stop() {
  enabled_ = false;
  timer_->Stop();
}
