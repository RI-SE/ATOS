/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2017 CHRONOS project
  ------------------------------------------------------------------------------
  -- File        : remotecontrol.c
  -- Author      : Sebastian Loh Lindholm
  -- Description : CHRONOS
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <sys/types.h>
#include "objectcontrol.h"
#include "util.h"
#include "logging.h"

#define BUF_SIZE 500

#define REMOTE_CONTROL_CONF_FILE_PATH  "conf/test.conf"
#define REMOTE_CONTROL_BUFFER_SIZE_20 20
#define REMOTE_CONTROL_BUFFER_SIZE_52 52
#define REMOTE_CONTROL_HTTP_HEADER "POST /esss HTTP/1.1\r\nHost: maestro.sebart.net\r\n"

#define REMOTE_CONTROL_HTTP_BUFFER 2048
#define REMOTE_CONTROL_RECV_BUFFER 2048

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
static void RemoteControlSendBytes(const char* data, int length, int* sockfd, int debug);

//void RemoteControlConnectServer(int* sockfd, const char* name, const uint32_t port);

//U32 RemoteControlSignIn(I32 ServerSocketI32, C8 *TablenameC8, C8 *UsernameC8, C8 *PasswordC8, U8 Debug);
static void vCreateTimeChannel(const char* name,const uint32_t port, int* sockfd, struct sockaddr_in* addr);
static void vRecvTime(int* sockfd, char* buffer, int length, int* recievedNewData);
int SendUDPData(int* sockfd, struct sockaddr_in* addr, char* SendData, int Length, char debug);

void getIP();


C8 httpbuffer[REMOTE_CONTROL_HTTP_BUFFER];
C8 recvbuffer[REMOTE_CONTROL_RECV_BUFFER];

/*------------------------------------------------------------
-- Private variables
------------------------------------------------------------*/
#define MODULE_NAME "RemoteControl"
static const LOG_LEVEL logLevel = LOG_LEVEL_DEBUG;

/*------------------------------------------------------------
-- The main function.
------------------------------------------------------------*/
int remotecontrol_task(TimeType *GPSTime)
{

  U8 ModeU8 = 0;
  C8 TextBufferC8[REMOTE_CONTROL_BUFFER_SIZE_20];
  C8 ServerIPC8[REMOTE_CONTROL_BUFFER_SIZE_20];
  U16 ServerPortU16;
  I32 SocketfdI32=-1;
  struct sockaddr_in time_addr;
  I32 iExit = 0;
  C8 TimeBuffer[REMOTE_CONTROL_BUFFER_SIZE_52];
  I32 RecievedNewData, i;
  C8 SendData[4] = {0, 0, 3, 0xe8};

  LogInit(MODULE_NAME,logLevel);
  LogMessage(LOG_LEVEL_INFO,"Remote control task running with PID: %i",getpid());


  UtilSearchTextFile(REMOTE_CONTROL_CONF_FILE_PATH, "RemoteMode=", "", TextBufferC8);
  ModeU8 = (U8)atoi(TextBufferC8);
  UtilSearchTextFile(REMOTE_CONTROL_CONF_FILE_PATH, "RemoteServerIP=", "", TextBufferC8);
  bzero(ServerIPC8, REMOTE_CONTROL_BUFFER_SIZE_20);
  strcat(ServerIPC8, TextBufferC8);
  UtilSearchTextFile(REMOTE_CONTROL_CONF_FILE_PATH, "RemoteServerPort=", "", TextBufferC8);
  ServerPortU16 = (U16)atoi(TextBufferC8);
  /*
  vCreateTimeChannel("192.168.0.42", 53000, &SocketfdI32,  &time_addr);
  SendUDPData(&SocketfdI32, &time_addr, SendData, 4, 1);
  printf("Checking time...\n");
  while(!iExit)
  {

    bzero(TimeBuffer,REMOTE_CONTROL_BUFFER_SIZE_52);
    vRecvTime(&SocketfdI32,TimeBuffer, REMOTE_CONTROL_BUFFER_SIZE_52, &RecievedNewData);
    if(RecievedNewData)
    {
      for(i=0; i < REMOTE_CONTROL_BUFFER_SIZE_52; i++) printf("%x-", TimeBuffer[i]);
      printf("\n");
    }

  }
  */
  usleep(1000);

}


U32 RemoteControlSendServerStatus(I32 ServerSocketI32, ServiceSessionType *SessionData, U32 StatusU32, U8 Debug)
{

  I32 ClientResultI32;
  I32 i;
  C8 StatusC8[20];
  C8 UserIdC8[20];
  C8 LengthC8[4];
  U32 LengthU32;
  C8 FirstResponseC8[5];

  bzero(FirstResponseC8, 5);
  bzero(StatusC8, 20);
  sprintf(StatusC8, "%" PRIu32, StatusU32);

  bzero(UserIdC8, 20);
  sprintf(UserIdC8, "%" PRIu32, SessionData->UserIdU32);


  bzero(httpbuffer, REMOTE_CONTROL_HTTP_BUFFER);
  bzero(recvbuffer, REMOTE_CONTROL_RECV_BUFFER);
  strcat(httpbuffer, REMOTE_CONTROL_HTTP_HEADER);
  strcat(httpbuffer, "SQL[UPDATE ");
  strcat(httpbuffer, "serverstatus");
  strcat(httpbuffer, " SET");
  strcat(httpbuffer, " status=");
  strcat(httpbuffer, StatusC8);
  strcat(httpbuffer, " WHERE userId=");
  strcat(httpbuffer, UserIdC8);
  strcat(httpbuffer, "];");
  if(Debug)printf("httpbuffer = %s\n", httpbuffer);
  LengthU32 = strlen(httpbuffer);
  LengthC8[0] = (C8)(LengthU32 >> 24); LengthC8[1] = (C8)(LengthU32 >> 16); LengthC8[2] = (C8)(LengthU32 >> 8); LengthC8[3] = (C8)(LengthU32);
  RemoteControlSendBytes(LengthC8, 4, &ServerSocketI32, Debug);
  RemoteControlSendBytes(httpbuffer, LengthU32, &ServerSocketI32, Debug);

  struct timeval tv;
  tv.tv_sec = 5;
  tv.tv_usec = 0;
  setsockopt(ServerSocketI32, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

  ClientResultI32 = recv(ServerSocketI32, FirstResponseC8, 1024, 0);

  if (ClientResultI32 > 0)
  {
    if(Debug)
    {
      printf("First response from Maestro service: %d bytes\n", ClientResultI32);
      for(i = 0; i < ClientResultI32; i ++) printf("%d-", FirstResponseC8[i]);
      printf("\n");
    }
  }

  ClientResultI32 = recv(ServerSocketI32, recvbuffer, 1024, 0);

  if (ClientResultI32 <= -1)
  {
    if(errno != EAGAIN && errno != EWOULDBLOCK)
    {
      perror("[RemoteControl]ERR: Failed to receive from command socket.");
      exit(1);
    }
  }
  else if(ClientResultI32 == 0)
  {
      printf("[RemoteControl] Client closed connection.\n");
      close(ServerSocketI32);
      exit(1);
  }
  if (ClientResultI32 > 0)
  {
    if(Debug)
    {
      printf("Second response from Maestro service: %d bytes\n", ClientResultI32);
      for(i = 0; i < ClientResultI32; i ++) printf("%d-", recvbuffer[i]);
      printf("\n");
    }
  }

  return 0;
}




U32 RemoteControlSignIn(I32 ServerSocketI32, C8 *TablenameC8, C8 *UsernameC8, C8 *PasswordC8, ServiceSessionType *SessionData, U8 Debug)
{

  I32 ClientResultI32;
  I32 i;
  SessionData->SessionIdU32 = 0;
  SessionData->UserIdU32 = 0;
  SessionData->UserTypeU8 = 0;

  bzero(httpbuffer, REMOTE_CONTROL_HTTP_BUFFER);
  bzero(recvbuffer, REMOTE_CONTROL_RECV_BUFFER);
  strcat(httpbuffer, REMOTE_CONTROL_HTTP_HEADER);
  strcat(httpbuffer, "SignIn(");
  strcat(httpbuffer, TablenameC8);
  strcat(httpbuffer, ",");
  strcat(httpbuffer, UsernameC8);
  strcat(httpbuffer, ",");
  strcat(httpbuffer, PasswordC8);
  strcat(httpbuffer, ");");
  printf("httpbuffer = %s\n", httpbuffer);

  RemoteControlSendBytes(httpbuffer, strlen(httpbuffer), &ServerSocketI32, 0);

  struct timeval tv;
  tv.tv_sec = 5;
  tv.tv_usec = 0;
  setsockopt(ServerSocketI32, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

  ClientResultI32 = recv(ServerSocketI32, recvbuffer, 1024, 0);

  if (ClientResultI32 <= -1)
  {
    if(errno != EAGAIN && errno != EWOULDBLOCK)
    {
      perror("[RemoteControl]ERR: Failed to receive from command socket.");
      exit(1);
    }
  }
  else if(ClientResultI32 == 0)
  {
      printf("[RemoteControl] Client closed connection.\n");
      close(ServerSocketI32);
      exit(1);
  }
  if (ClientResultI32 > 0)
  {
    if(Debug)
    {
      printf("Response from Maestro service: %d bytes\n", ClientResultI32);
      for(i = 0; i < ClientResultI32; i ++) printf("%d-", recvbuffer[i]);
      printf("\n");
    }
    SessionData->SessionIdU32 = recvbuffer[6];
    SessionData->SessionIdU32 = (SessionData->SessionIdU32<<8) | recvbuffer[7];
    SessionData->SessionIdU32 = (SessionData->SessionIdU32<<8) | recvbuffer[8];
    SessionData->SessionIdU32 = (SessionData->SessionIdU32<<8) | recvbuffer[9];

    SessionData->UserIdU32 = recvbuffer[10];
    SessionData->UserIdU32 = (SessionData->UserIdU32<<8) | recvbuffer[11];
    SessionData->UserIdU32 = (SessionData->UserIdU32<<8) | recvbuffer[12];
    SessionData->UserIdU32 = (SessionData->UserIdU32<<8) | recvbuffer[13];

    SessionData->UserTypeU8 = recvbuffer[14];
  }

  return 0;
}


void RemoteControlConnectServer(int* sockfd, const char* name, const uint32_t port)
{
  struct sockaddr_in serv_addr;
  struct hostent *server;

  char buffer[256];
  int iResult;

  *sockfd = socket(AF_INET, SOCK_STREAM, 0);

  if (*sockfd < 0)
  {
    util_error("ERR: Failed to open control socket");
  }

  server = gethostbyname(name);
  if (server == NULL)
  {
    util_error("ERR: Unknown host ");
  }

  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;

  bcopy((char *) server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
  serv_addr.sin_port = htons(port);

  LogMessage(LOG_LEVEL_INFO,"Attempting to connect to control socket: %s:%i",name,port);


  do
  {
    iResult = connect(*sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr));

    if ( iResult < 0)
    {
      if(errno == ECONNREFUSED)
      {
          LogMessage(LOG_LEVEL_WARNING,"Unable to connect to object, retrying in 3 sec...");
        (void)sleep(3);
      }
      else
      {
        util_error("ERR: Failed to connect to control socket");
      }
    }
  } while(iResult < 0);


  /* set socket to non-blocking */
    //iResult = fcntl(*sockfd, F_SETFL, fcntl(*sockfd, F_GETFL, 0) | O_NONBLOCK);
    LogMessage(LOG_LEVEL_INFO,"Client connected to server: %s:%i",name,port);

}


static void RemoteControlSendBytes(const char* data, int length, int* sockfd, int debug)
{
  int i, n;

  if(debug == 1){ printf("Bytes sent: "); int i = 0; for(i = 0; i < length; i++) printf("%x-", (unsigned char)*(data+i)); printf("\n");}

  n = write(*sockfd, data, length);
  if (n < 0)
  {
    util_error("[RemoteControl] ERR: Failed to send on control socket");
  }
}



void getIP()
{
          struct addrinfo hints;
           struct addrinfo *result, *rp;
           int sfd, s, j;
           size_t len;
           ssize_t nread;
           char buf[BUF_SIZE];
          const char *hostm = "maestro.sebart.net";

           /* Obtain address(es) matching host/port */

          printf("Maestro check address\n");
           memset(&hints, 0, sizeof(struct addrinfo));
           hints.ai_family = 0;    /* Allow IPv4 or IPv6 */
           hints.ai_socktype = 0; /* Datagram socket */
           hints.ai_flags = 0;
           hints.ai_protocol = 0;          /* Any protocol */

           s = getaddrinfo(NULL, hostm, &hints, &result);
           if (s != 0) {
               fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(s));
               //exit(EXIT_FAILURE);
           }

           printf("Maestro service IP:%s\n", result->ai_addr->sa_data);
           /* getaddrinfo() returns a list of address structures.
              Try each address until we successfully connect(2).
              If socket(2) (or connect(2)) fails, we (close the socket
              and) try the next address. */


            freeaddrinfo(result);           /* No longer needed */
 }



static void vCreateTimeChannel(const char* name,const uint32_t port, int* sockfd, struct sockaddr_in* addr)
{
  int result;
  struct hostent *object;

  /* Connect to object safety socket */
    LogMessage(LOG_LEVEL_INFO,"Creating safety socket");


  *sockfd= socket(AF_INET, SOCK_DGRAM, 0);
  if (*sockfd < 0)
  {
    util_error("ERR: Failed to connect to monitor socket");
  }

  /* Set address to object */
  object = gethostbyname(name);

  if (object==0)
  {
    util_error("ERR: Unknown host");
  }

  bcopy((char *) object->h_addr,
    (char *)&addr->sin_addr.s_addr, object->h_length);
  addr->sin_family = AF_INET;
  addr->sin_port = htons(port);

   /* set socket to non-blocking */
  result = fcntl(*sockfd, F_SETFL,
    fcntl(*sockfd, F_GETFL, 0) | O_NONBLOCK);
  if (result < 0)
  {
    util_error("ERR: calling fcntl");
  }
    LogMessage(LOG_LEVEL_INFO,"Created socket and safety address: %s:%d",name,port);


}


static void vRecvTime(int* sockfd, char* buffer, int length, int* recievedNewData)
{
  int result;
  *recievedNewData = 0;
    do
    {
      result = recv(*sockfd, buffer, length, 0);

      if (result < 0)
      {
        if(errno != EAGAIN && errno != EWOULDBLOCK)
        {
          util_error("ERR: Failed to receive from monitor socket");
        }
        else
        {
            LogMessage(LOG_LEVEL_DEBUG,"No data received");

        }
      }
      else
      {

          LogMessage(LOG_LEVEL_DEBUG,"Received: <%s>",buffer);

      }
    } while(result > 0 );
}

int SendUDPData(int* sockfd, struct sockaddr_in* addr, char* SendData, int Length, char debug)
{
    int result;

    result = sendto(*sockfd, SendData, Length, 0, (const struct sockaddr *) addr, sizeof(struct sockaddr_in));

    if (result < 0)
    {
      util_error("ERR: Failed to send on monitor socket");
    }

    return 0;
}
