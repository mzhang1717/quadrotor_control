/**
 * Simple UDP Receiver Program.
 * Parameters:
 *    1) Number of doubles in a packet.
 *    2) Local Network Port used to receive the packet.
 * 
 * Outputs: 
 *    1) Data:    Array of doubles.
 *  
 */


/* -------------------------------------------------------------------------
 * Include System Headers.
 * ------------------------------------------------------------------------- */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <fcntl.h>
#include <time.h>

#ifdef _WIN32
   #include <windows.h>
#else 
#ifdef __QNX__
   #include <unix.h>
   #include <errno.h>
   #include <sys/ioctl.h>
   #include <unistd.h>
#endif
   #include <netinet/in.h>
   #include <netdb.h>
   #include <sys/socket.h>
   #include <sys/time.h>
   #include <sys/select.h>
   #include <unistd.h>
#endif /* _WIN32 */

#include <fstream>
#include <iostream>

/* -------------------------------------------------------------------------
 * Constants.
 * ------------------------------------------------------------------------- */

#define BUFFER_SIZE    64

#define SFCT_NUM_ARGS			(3)
#define SFCT_ARG_BUFFER_LEN		8
#define SFCT_ARG_PORT_ID		16001
#define SFCT_ARG_SAMPLE_TIME		0.01

//#define _DEBUG_


/* =========================================================================
 * TYPEDEF
 * ========================================================================= */


int main() {
  
  /* Simulink work elements. */
  struct sockaddr_in        their_address;
  socklen_t iaddr_len; 
  int inumber_of_bytes_receive;

  struct sockaddr_in listener_address;
  int uisocket;
  double *pdpacket;

#ifdef _WIN32
  WSADATA wsaData;      /* Used to initialise the ws2_32.dll link. */
  int ireturn_value;
#endif

#ifdef _WIN32
  if ((ireturn_value = WSAStartup(MAKEWORD(2, 0),&wsaData)) != 0) {
    printf ("WSAStartup failed");
    return 1;
  }
#endif /* _WIN32 */
   
  uisocket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
//  uisocket = socket(AF_INET, SOCK_DGRAM, 0);
  if (uisocket < 0){
    printf ("Unable to create the UDP socket");
    return 2;
  }
   
  /* Fill in the listener address structure. */
  memset(&listener_address, 0, sizeof(listener_address));   
  listener_address.sin_family = AF_INET;
  listener_address.sin_port = htons(SFCT_ARG_PORT_ID);
  listener_address.sin_addr.s_addr = INADDR_ANY;
   
  if (bind(uisocket, (struct sockaddr *) &listener_address, sizeof(struct sockaddr)) == -1) {
    printf ("Unable to bind the UDP socket");
    return 3;
  }

  /* Allocate memory for the packet buffer. */
  pdpacket = (double *) malloc(sizeof(double) * SFCT_ARG_BUFFER_LEN);
  memset(pdpacket, 0, sizeof(double) * SFCT_ARG_BUFFER_LEN);

  iaddr_len = sizeof(struct sockaddr);
  
  std::ofstream debug_out("vicon_debug.csv");
  debug_out.precision(15);
  struct timeval currTime;
  double currTimeDbl;
  
  //while(1) {
  for(int i=0; i < 2e4; ++i)
  {
    /* Receive the data. */
    inumber_of_bytes_receive = 
       recvfrom(uisocket,
		            (char *) pdpacket,
		            SFCT_ARG_BUFFER_LEN * sizeof(double),
                0,
                (struct sockaddr *)&their_address,
                &iaddr_len);
    //printf("\r %f %f %f %f %f %f %f %f", pdpacket[0], pdpacket[1], pdpacket[2], pdpacket[3], pdpacket[4], pdpacket[5], pdpacket[6], pdpacket[7]);
    gettimeofday(&currTime, NULL);
    currTimeDbl = currTime.tv_sec + (currTime.tv_usec*1e-6);
    
    std::cout<<pdpacket[0]<<", "<<currTimeDbl<<"\n";
    debug_out<<pdpacket[0]<<", "<<currTimeDbl<<"\n";
  
  };


  /* Close the socket used. */
#ifdef _WIN32
  closesocket(uisocket);
  WSACleanup();
#else
  close(uisocket);
#endif
  
  free(pdpacket);
  
  return 0;
};

