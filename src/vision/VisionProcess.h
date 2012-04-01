#ifndef VISION_VISION_PROCESS_H_
#define VISION_VISION_PROCESS_H_

#include "WPILib.h"

/**
 * @author Tom Bottglieri
 *
 * Abstract superclass of tasks (run on a separate thread) involving vision targets.
 * Call Start() to begin the task and Stop() to terminate it.
 */
class VisionProcess {
 public:
  /**
   * Do not call this function externally as it's only passed to the constructor to initialize the object.
   *
   * This function perpetually loops, calling DoVision() if Start() has been called and Stop() hasn't.
   * @param VisionProcess the given VisionProcess object to run.
   */
  static void VisionTask(VisionProcess* vp);

  /**
   * Starts the task.
   */
  void Start();

  /**
   * Stops the task.
   */
  void Stop();

  /**
   * Sole constructor.
   */
  VisionProcess();

  /**
   * Virtual destructor.
   */
  virtual ~VisionProcess();

  /**
   * A task to execute perpetually when Start() is called until Stop() is
   * called.
   *
   * Subclasses should override this function to specify their own behavior.
   */
  virtual void DoVision() = 0;

 private:
  bool enabled_;
  Task* task_;
  Timer* timer_;
};

#endif  // VISION_VISION_PROCESS_H_
