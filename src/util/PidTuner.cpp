#include "util/PidTuner.h"

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

#define SERVER_PORT_NUM 41234

PidTuner* PidTuner::instance_ = NULL;

PidTuner* PidTuner::GetInstance() {
  if (instance_ == NULL) {
    instance_ = new PidTuner();
  }
  return instance_;
}

void PidTuner::PushData(double setpoint, double value) {
  PidTuner::GetInstance()->Push(setpoint, value);
}

PidTuner::PidTuner() {
/* create client's socket */

 if ((sFd_ = socket (AF_INET, SOCK_DGRAM, 0)) == ERROR)
   {
     perror ("socket");
     // return (ERROR);
   }

 /* bind not required - port number is dynamic */

 /* build server socket address */

 sockAddrSize_ = sizeof (struct sockaddr_in);
 bzero ((char *) &serverAddr_, sockAddrSize_);
 serverAddr_.sin_len = (u_char) sockAddrSize_;
 serverAddr_.sin_family = AF_INET;
 serverAddr_.sin_port = htons (SERVER_PORT_NUM);

 if (((serverAddr_.sin_addr.s_addr = inet_addr ("10.2.54.125")) == ERROR))
   {
     perror ("unknown server name");
     close (sFd_);
     //    return (ERROR);
   }

}

void PidTuner::Push(double setpoint, double value) {
  char myRequest[50];
  sprintf(myRequest, "{\"S\":%f, \"V\":%f}\0", (float) setpoint, (float) value);
  if (sendto(sFd_, (caddr_t) myRequest, strlen(myRequest), 0,
	     (struct sockaddr *) &serverAddr_, sockAddrSize_) == ERROR)
   {
     perror ("sendto");
     close (sFd_);
   }

 // close (sFd);
}
