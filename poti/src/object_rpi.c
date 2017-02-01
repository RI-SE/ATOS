/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  --------------------------------------------------------------------------------
  -- File        : object.c
  -- Author      : Karl-Johan Ode
  -- Description : CHRONOS object
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

/*------------------------------------------------------------
  -- Include files.
  ------------------------------------------------------------*/

#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <inttypes.h>
#include <stdint.h>
#include <limits.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include "nmea2etsi.h"

/*------------------------------------------------------------
  -- Defines.
  ------------------------------------------------------------*/

#define SAFETY_CHANNEL_PORT    53240       // Default port, safety channel
#define CONTROL_CHANNEL_PORT   53241       // Default port, control channel
#define ORIGIN_LATITUDE        57.7773602  // Default latitude,  origin
#define ORIGIN_LONGITUDE       012.7804715 // Default longitude, origin
#define ORIGIN_ALTITUDE        201.485     // Default altitude,  origin
#define DATALEN                10000
#define SAVED_TRAJECTORY_LINES 100
#define PI                     acos(-1)
#define DEBUG 0

#define BufferLength 80

#define SERVER "127.0.0.1"

#define SERVPORT 2948

/* 34 years between 1970 and 2004, 8 days for leap year between 1970 and 2004      */
/* Calculation: 34 * 365 * 24 * 3600 * 1000 + 8 * 24 * 3600 * 1000 = 1072915200000 */
#define MS_FROM_1970_TO_2004_NO_LEAP_SECS 1072915200000

/* Number of leap seconds since 1970 */
#define NBR_LEAP_SECONDS_FROM_1970 27

//GPS
char nmea_msg[20]              = "N/A";
char utc[20]                   = "N/A";
char status[20]                = "N/A";
char latitude[20]              = "N/A";
char northsouth[20]            = "N/A";
char longitude[20]             = "N/A";
char eastwest[20]              = "N/A";
char gps_speed[20]             = "N/A";
char gps_heading[20]           = "N/A";
char date[20]                  = "N/A";
char gps_quality_indicator[20] = "N/A";
char satellites_used[20]       = "N/A";
char pdop[20]                  = "N/A";
char hdop[20]                  = "N/A";
char vdop[20]                  = "N/A";
char antenna_altitude[20]      = "N/A";
char satellites_in_view[20]    = "N/A";
char sentence[BufferLength];

void getField(char* buffer, int index)
{
  int sentencePos = 0;
  int fieldPos = 0;
  int commaCount = 0;
  while (sentencePos < BufferLength)
  {
    if (sentence[sentencePos] == ',')
    {
      commaCount ++;
      sentencePos ++;
    }
    if (commaCount == index)
    {
      buffer[fieldPos] = sentence[sentencePos];
      fieldPos ++;
    }
    sentencePos ++;
  }
  buffer[fieldPos] = '\0';
}


/*------------------------------------------------------------
  -- The main function.
  ------------------------------------------------------------*/

int main(int argc, char *argv[])
{
  FILE *fp;
  char bFileName[6];
  char *bFileLine;
  size_t len;
  //int read;
  int monitor_socket_fd;
  int command_server_socket_fd;
  int command_com_socket_fd;
  int n;
  int pid;
  int fpos;
  int trig_time = INT_MAX;
  socklen_t cli_length;
  struct sockaddr_in monitor_server_addr;
  struct sockaddr_in monitor_from_addr;
  struct sockaddr_in command_server_addr;
  struct sockaddr_in cli_addr;
  struct hostent *hp;
  char buffer[256];
  char bMonitorBuffer[256];
  char pcPosition[256];
  struct timespec sleep_time;
  struct timespec ref_time;
  char pcTimeString[15];
  char *ret = NULL;
  char bCurrentCommand[10] = "NOOP"; 
  const char bEndDOPM[10]= "ENDDOPM";
  const char bEndTRIG[10]= "ENDTRIG";
  char bData[DATALEN];
  int newFileData = 0;
  char bFileTrajName[100];
  char bCommand[10];
  char bFileCommand[10];
  char bFileTraj[10];
  char bLine[10];
  char bEndLine[10];
  float fileTrajVersion;
  int row = 0;
  int fileRows = 0;
  int result = 0;
  float   time;
  double  x;
  double  y;
  double  z;
  float   hdg;
  float   vel;
  double cal_lat = 0.0;
  double cal_lon = 0.0;
  double cal_alt = 0.0;
  double earth_radius = 6378137.0;
  double lat_start = ORIGIN_LATITUDE;
  double lon_start = ORIGIN_LONGITUDE;
  double alt_start = ORIGIN_ALTITUDE;
  unsigned int safety_port = SAFETY_CHANNEL_PORT;
  unsigned int control_port = CONTROL_CHANNEL_PORT;
  int optval = 1;

  struct timeval tv;

  int sd, rc, length = sizeof(int);
  struct sockaddr_in serveraddr;
  char character; 
  char server[255];
  char temp;
  int totalcnt = 0;
  int i = 0;

  char etsi_time_string[20];
  TimestampIts etsi_time = 0;
  HeadingValue etsi_heading = 0;
  SpeedValue etsi_speed = 0;
  Latitude etsi_lat = 0;
  Longitude etsi_lon = 0;
  AltitudeValue etsi_alt = 0;

  struct hostent *hostp;


  
  if ( !( (argc == 1) || (argc == 4) || (argc == 6))){
    perror("Please use 0, 3, or 5 arguments: object <lat,lon,alt><safety port,control port>");
    exit(1);
  }
  
  if (argc > 1){
    lat_start = atof(argv[1]);
    lon_start = atof(argv[2]);
    alt_start = atof(argv[3]);
  }
  
#ifdef DEBUG
  printf("INF: Start point set to: lat:%.7lf lon:%.7lf alt:%f\n",lat_start,lon_start,alt_start);
#endif
  
  if (argc > 4){
    safety_port = atoi(argv[4]);
    control_port = atoi(argv[5]);
  }
#ifdef DEBUG
  printf("INF: Ports set to: safety:%d control:%d\n",safety_port,control_port);
#endif
  
#ifdef DEBUG
  printf("INF: Starting object\n");
  fflush(stdout);
#endif


  if((sd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
  {
    perror("Client-socket() error");
    exit(-1);
  }
  else
    printf("Client-socket() OK\n");
    /*If the server hostname is supplied*/
  if(argc > 1)
  {
  /*Use the supplied argument*/
   strcpy(server, argv[1]);
   printf("Connecting to %s, port %d ...\n", server, SERVPORT);
 }
 else
 {
      /*Use the default server name or IP*/
  strcpy(server, SERVER);
 }

memset(&serveraddr, 0x00, sizeof(struct sockaddr_in));
serveraddr.sin_family = AF_INET;
serveraddr.sin_port = htons(SERVPORT);

if((serveraddr.sin_addr.s_addr = inet_addr(server)) == (unsigned long)INADDR_NONE)
{
  hostp = gethostbyname(server);

  if(hostp == (struct hostent *)NULL)
  {
   printf("HOST NOT FOUND --> ");
   printf("h_errno = %d\n",h_errno);
   printf("---This is a client program---\n");
   printf("Command usage: %s <server name or IP>\n", argv[0]);
   close(sd);
   exit(-1);
 }
 memcpy(&serveraddr.sin_addr, hostp->h_addr, sizeof(serveraddr.sin_addr));
}

if((rc = connect(sd, (struct sockaddr *)&serveraddr, sizeof(serveraddr))) < 0)
{
  perror("Client-connect() error");
  close(sd);
  exit(-1);
}
else
  printf("Connection established...\n");

/* set socket to non-blocking */
(void)fcntl(sd, F_SETFL, fcntl(sd, F_GETFL, 0) | O_NONBLOCK);


  /* Open a drive file */
sprintf(bFileName,"%d",safety_port);
fp = fopen(bFileName, "w+");
if (fp == NULL)
{
  perror("ERR: Failed to create local drive file");
  exit(1);
}

  /* Init monitor socket */
#ifdef DEBUG
printf("INF: Init monitor socket\n");
fflush(stdout);
#endif

monitor_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
if (monitor_socket_fd < 0)
{
  perror("ERR: Failed to create monitor socket");
  exit(1);
}
bzero(&monitor_server_addr, sizeof(monitor_server_addr));
monitor_server_addr.sin_family = AF_INET;
monitor_server_addr.sin_addr.s_addr = INADDR_ANY;
monitor_server_addr.sin_port = htons(safety_port);

if (bind(monitor_socket_fd,(struct sockaddr *) &monitor_server_addr, sizeof(monitor_server_addr)) < 0)
{
  perror("ERR: Failed to bind to monitor socket");
  exit(1);
}

socklen_t fromlen = sizeof(struct sockaddr_in);

  /* Init control socket */
#ifdef DEBUG
printf("INF: Init control socket\n");
fflush(stdout);
#endif

command_server_socket_fd = socket(AF_INET, SOCK_STREAM, 0);
if (command_server_socket_fd < 0)
{
  perror("ERR: Failed to create control socket");
  exit(1);
}
bzero((char *) &command_server_addr, sizeof(command_server_addr));

command_server_addr.sin_family = AF_INET;
command_server_addr.sin_addr.s_addr = INADDR_ANY; 
command_server_addr.sin_port = htons(control_port);

optval = 1;
result = setsockopt(command_server_socket_fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof optval);

if (result < 0)
{
  perror("ERR: Failed to call setsockopt");
  exit(1);
}

if (bind(command_server_socket_fd, (struct sockaddr *) &command_server_addr, sizeof(command_server_addr)) < 0) 
{
  perror("ERR: Failed to bind to control socket");
  exit(1);
}

  /* Monitor and control sockets up. Wait for central to connect to control socket to get server address*/
#ifdef DEBUG
printf("INF: Listen on control socket\n");
fflush(stdout);
#endif

listen(command_server_socket_fd, 5);
cli_length = sizeof(cli_addr);

#ifdef DEBUG
printf("INF: Accept on control socket: %i \n", htons(command_server_addr.sin_port));
fflush(stdout);
#endif

command_com_socket_fd = accept(command_server_socket_fd, (struct sockaddr *) &cli_addr, &cli_length);
if (command_com_socket_fd < 0) 
{
  perror("ERR: Failed to accept from central");
  exit(1);
}

  /* Receive heartbeat from server just to save the server address */
n = recvfrom(monitor_socket_fd, buffer, 1024, 0, (struct sockaddr *) &monitor_from_addr, &fromlen);
if (n < 0) 
{
  perror("ERR: Failed to receive from central");
  exit(1);
}

  /* set socket to non-blocking */
(void)fcntl(command_com_socket_fd, F_SETFL, fcntl(command_com_socket_fd, F_GETFL, 0) | O_NONBLOCK);

#ifdef DEBUG
printf("INF: Received: <%s>\n", buffer);
#endif

char workingBuffer[512];
bzero(workingBuffer,512);

int nbrOfBytesLeft = 0;
while(1)
{
  bzero(buffer,256);
  rc = recv(sd, buffer, 256, 0);

  (void)strncat(&workingBuffer[nbrOfBytesLeft],buffer,strlen(buffer));

  #ifdef DEBUG
    printf("INF: Received from RTK: %s \n", buffer);
    fflush(stdout);
  #endif

  #ifdef DEBUG
    printf("INF: workingBuffer: %s \n", workingBuffer);
    fflush(stdout);
  #endif

  if(rc < 0)
  {
   perror("Client-read() error");
   close(sd);
   exit(-1);
  }

  /* loop until message has been parsed */
  int i = 0;
  while(i < rc)
  {
    int k = 0;
    /* get next message */
    while((workingBuffer[i] != '\n') && (i < rc))
    {
     sentence[k] = workingBuffer[i];
     k++;
     i++;
    }
    i++;

    /* Check if we stopped due to new message */
    if(workingBuffer[i-1] == '\n')
    {
      /* Calc how many bytes left */
      nbrOfBytesLeft = rc-i;
      #ifdef DEBUG
        printf("INF: nbrOfBytesLeft rc k i: %d %d %d %d \n", nbrOfBytesLeft,rc,k,i);
        fflush(stdout);
      #endif

      sentence[k] = '\0';

      #ifdef DEBUG
        printf("INF: Message to handle: %s \n", sentence);
        fflush(stdout);
      #endif

      getField(nmea_msg, 0);
      if (strcmp(nmea_msg, "$GPRMC") == 0) 
      {
        getField(utc, 1);
        getField(status, 2);
        getField(latitude, 3);
        getField(northsouth, 4);
        getField(longitude, 5);
        getField(eastwest, 6);
        getField(gps_speed, 7);
        getField(gps_heading, 8);
        getField(date, 9);
      }
      else if (strcmp(nmea_msg, "$GPGGA") == 0) 
      {
        getField(utc, 1);
        getField(latitude, 2);
        getField(northsouth, 3);
        getField(longitude, 4);
        getField(eastwest, 5);
        getField(gps_quality_indicator, 6);
        getField(satellites_used, 7);
        getField(antenna_altitude, 9);
      }
      else if (strcmp(nmea_msg, "$GPGSV") == 0) 
      {
        getField(satellites_in_view, 3);
      }
      else if (strcmp(nmea_msg, "$GPGSA") == 0) 
      {
        getField(pdop, 4);
        getField(hdop, 5);
        getField(vdop, 6);
      }
      etsi_lat          = ConvertLatitudeNMEAtoETSICDD(latitude, northsouth, status);
      etsi_lon          = ConvertLongitudeNMEAtoETSICDD(longitude, eastwest, status);
      etsi_speed        = ConvertSpeedValueNMEAtoETSICDD(gps_speed, status);
      etsi_heading      = ConvertHeadingValueNMEAtoETSICDD(gps_heading, status);
      etsi_alt          = ConvertAltitudeValueNMEAtoETSICDD(antenna_altitude, status);
      etsi_time         = ConvertTimestapItsNMEAtoETSICDD(utc, date, status);
    }

    /* Copy bytes to beginning */
    char tempBuffer[512];
    bzero(tempBuffer,512);
    strncpy(tempBuffer,&workingBuffer[i],nbrOfBytesLeft);
    bzero(workingBuffer,512);
    strncpy(workingBuffer,tempBuffer,nbrOfBytesLeft);
  }

  sprintf(etsi_time_string, "%" PRIu64 "", etsi_time);
  bzero(bMonitorBuffer, 256);
  sprintf(bMonitorBuffer,"MONR;%s;%09d;%010d;%06d;%05d;%04d;0;\n",etsi_time_string,etsi_lat,etsi_lon,etsi_alt,etsi_speed,etsi_heading);
  n = sendto(monitor_socket_fd, bMonitorBuffer, sizeof(bMonitorBuffer), 0, (struct sockaddr *) &monitor_from_addr, fromlen);
  if (n < 0)
  {
   perror("ERR: Failed to send monitor message");
   exit(1);
  }

  #ifdef DEBUG
  printf("INF: Sending: <%s>\n", bMonitorBuffer);
  #endif

  sleep_time.tv_sec = 0;
  sleep_time.tv_nsec = 1000000000;
  (void) nanosleep(&sleep_time, &ref_time);
}

close(monitor_socket_fd);
close(command_com_socket_fd);
close(sd);
fclose(fp);

return 0;
}
