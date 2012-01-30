#ifndef PID_TUNER_H_
#define PID_TUNER_H_

class PidTuner {
 public: 
  static void PushData(double setpoint, double value);
  static PidTuner* GetInstance();

 private:
  void Push(double setpoint, double value);
  PidTuner();
  ~PidTuner();


 struct sockaddr_in  serverAddr;    /* server's socket address */ 
 char                display;       /* if TRUE, server prints message */ 
 int                 sockAddrSize;  /* size of socket address structure */ 
 int                 sFd;           /* socket file descriptor */  
 int                 mlen;          /* length of message */  
  
  static PidTuner* instance;
};

#endif
