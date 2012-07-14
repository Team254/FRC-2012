#include "util/PoofTuner.h"

#include "vxWorks.h"
#include "errno.h"
#include "fioLib.h"
#include "hostLib.h"
#include "sigLib.h"
#include "signal.h"
#include "sockLib.h"
#include <string>
#include "time.h"
#include "usrLib.h"

#define SERVER_PORT_NUM 1180

PoofTuner* PoofTuner::instance_ = NULL;

PoofTuner* PoofTuner::GetInstance() {
  if (instance_ == NULL) {
    instance_ = new PoofTuner();
  }
  return instance_;
}

void PoofTuner::PushData(double setpoint, double value, double control) {
  PoofTuner::GetInstance()->Push(setpoint, value, control);
}

PoofTuner::PoofTuner() {
  /* build server socket address */
  sockAddrSize_ = sizeof (struct sockaddr_in);
  bzero ((char *) &serverAddr_, sockAddrSize_);
  serverAddr_.sin_len = (u_char) sockAddrSize_;
  serverAddr_.sin_family = AF_INET;
  serverAddr_.sin_port = htons (SERVER_PORT_NUM);

  if (((serverAddr_.sin_addr.s_addr = inet_addr ("10.2.54.5")) == ERROR)) {
    close (sFd_);
  }
}

void PoofTuner::Push(double setpoint, double value, double control) {
  char myRequest[50];
  sprintf(myRequest, "{\"S\":%f, \"V\":%f, \"C\":%f}\0", (float) setpoint, (float) value, (float) control);
  if (sendto(sFd_, (caddr_t) myRequest, strlen(myRequest), 0,
     (struct sockaddr *) &serverAddr_, sockAddrSize_) == ERROR) {
    close (sFd_);
  }
}
