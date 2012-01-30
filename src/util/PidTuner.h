#ifndef UTIL_PID_TUNER_H_
#define UTIL_PID_TUNER_H_

#include "inetLib.h"

class PidTuner {
 public:
  static void PushData(double setpoint, double value);
  static PidTuner* GetInstance();

 private:
  PidTuner();
  void Push(double setpoint, double value);

  struct sockaddr_in serverAddr_;  // Server's socket address
  char display_;  // If true, server prints message
  int sockAddrSize_;  // Size of socket address structure
  int sFd_;  // Socket file descriptor
  int mlen_;  // Length of message

  static PidTuner* instance_;
};

#endif  // UTIL_PID_TUNER_H_
