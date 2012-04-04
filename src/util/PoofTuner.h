#ifndef UTIL_POOF_TUNER_H_
#define UTIL_POOF_TUNER_H_

#include "inetLib.h"

/**
 * Singleton class which connects to the robot and gives Poof output. 
 */
class PoofTuner {
 public:
  static void PushData(double setpoint, double value, double control);
  /**
   * Returns the current instance of the PoofTuner object.
   */
  static PoofTuner* GetInstance();

 private:
  PoofTuner();
  void Push(double setpoint, double value, double control);

  struct sockaddr_in serverAddr_;  // Server's socket address
  char display_;  // If true, server prints message
  int sockAddrSize_;  // Size of socket address structure
  int sFd_;  // Socket file descriptor
  int mlen_;  // Length of message

  static PoofTuner* instance_;
};

#endif  // UTIL_POOF_TUNER_H_
