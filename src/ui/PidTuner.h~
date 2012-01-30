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
  
  static PidTuner* instance;
};

#endif
