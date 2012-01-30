
#include "sockLib.h"
#include "vxWorks.h"

#include "errno.h"
#include "fioLib.h"
#include "hostLib.h"
#include "inetLib.h"
#include "signal.h"
#include "sigLib.h"             // for signal                                                                                                                                            
#include <string>
#include "time.h"
#include "usrLib.h"
#include "ui/PidTuner.h"

#define BUFLEN 512
#define NPACK 10
#define SERVER_PORT_NUM 41234

PidTuner* PidTuner::instance = NULL;

PidTuner* PidTuner::GetInstance() {
  if (instance == NULL) {
    instance = new PidTuner();
  }
  return instance;
}

void PidTuner::PushData(double setpoint, double value) {
  PidTuner::GetInstance()->Push(setpoint, value);
}

PidTuner::PidTuner() {

 struct sockaddr_in si_other;
 int s, i, slen=sizeof(si_other);
 char buf[BUFLEN];

/* create client's socket */ 
 
 if ((sFd = socket (AF_INET, SOCK_DGRAM, 0)) == ERROR) 
   { 
     perror ("socket"); 
     // return (ERROR); 
   } 
 
 /* bind not required - port number is dynamic */ 
 
 /* build server socket address */ 
 
 sockAddrSize = sizeof (struct sockaddr_in); 
 bzero ((char *) &serverAddr, sockAddrSize); 
 serverAddr.sin_len = (u_char) sockAddrSize; 
 serverAddr.sin_family = AF_INET; 
 serverAddr.sin_port = htons (SERVER_PORT_NUM); 
 
 if (((serverAddr.sin_addr.s_addr = inet_addr ("10.2.54.125")) == ERROR)) 
   { 
     perror ("unknown server name"); 
     close (sFd); 
     //    return (ERROR); 
   } 

}

void PidTuner::Push(double setpoint, double value) {
  char myRequest[50];
  sprintf(myRequest, "{\"S\":%f, \"V\":%f}\0", (float) setpoint, (float) value);
  if (sendto (sFd, (caddr_t) myRequest, strlen(myRequest), 0, 
	     (struct sockaddr *) &serverAddr, sockAddrSize) == ERROR) 
   { 
     perror ("sendto"); 
     close (sFd); 
   } 
 
 // close (sFd);
 }
