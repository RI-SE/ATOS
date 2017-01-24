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
#define DEBUG

/* 34 years between 1970 and 2004, 8 days for leap year between 1970 and 2004      */
/* Calculation: 34 * 365 * 24 * 3600 * 1000 + 8 * 24 * 3600 * 1000 = 1072915200000 */
#define MS_FROM_1970_TO_2004_NO_LEAP_SECS 1072915200000

/* Number of leap seconds since 1970 */
#define NBR_LEAP_SECONDS_FROM_1970 27

/*------------------------------------------------------------
  -- The main function.
  ------------------------------------------------------------*/

int main(int argc, char *argv[])
{
  FILE *fp;
  char bFileName[6];
  char *bFileLine;
  size_t len;
  int read;
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
  result = fcntl(command_com_socket_fd, F_SETFL, fcntl(command_com_socket_fd, F_GETFL, 0) | O_NONBLOCK);

#ifdef DEBUG
  printf("INF: Received: <%s>\n", buffer);
#endif

  while(1)
    {
      int receivedNewData = 0;
      bzero(bData,DATALEN);
      bzero(buffer,256);
      do
	{
#ifdef DEBUG
	  printf("INF: Start receive\n");
#endif

	  result = recv(command_com_socket_fd, buffer, 256, 0);

	  if (result < 0)
	    {
	      if(errno != EAGAIN && errno != EWOULDBLOCK)
		{
		  perror("ERR: Failed to receive from command socket");
		  exit(1);
		}
	      else
		{
#ifdef DEBUG
		  printf("INF: No data received\n");
		  fflush(stdout);
#endif
		}
	    }
	  else
	    {
	      receivedNewData = 1;
#ifdef DEBUG
	      printf("INF: Received: <%s>\n",buffer);
	      fflush(stdout);
#endif
	      if (!strncmp(buffer, "DOPM", 4))
		{
#ifdef DEBUG
		  printf("INF: Current command set to DOPM\n");
#endif
		  bzero(bCurrentCommand, 10);
		  strcpy(bCurrentCommand, "DOPM");
		}
	      else if (!strncmp(buffer, "TRIG", 4))
		{
#ifdef DEBUG
		  printf("INF: Current command set to TRIG\n");
#endif
		  bzero(bCurrentCommand, 10);
		  strcpy(bCurrentCommand, "TRIG");
		  trig_time = 0;  
		}
	      if(!strncmp(bCurrentCommand, "DOPM", 4))
		{
		  if (result > 0 && result < 256)
		    buffer[result] = '\0';
		  fputs(buffer, fp);
		  fflush(fp);
		}
	      else
		{
		  strncat(bData, buffer, result);
		}
	    }
	} while(result > 0);

    
      /* Did we receive anything on command? */
      if(receivedNewData)
	{
	  if(!strncmp(bCurrentCommand, "DOPM", 4))
	    {

	      rewind(fp);
	      read = getline(&bFileLine, &len, fp);
	      if (read == -1)
		{
		 perror("ERR: Failed to read first line in drive file");
		 exit(1); 
		}

	      int l = 0;
	      int li = 0;
	      for(l = 0; l <= len; l++){
		if(bFileLine[l] == ';'){
		  bFileLine[l] = '\0';
		  sscanf(&bFileLine[li],"%s",bFileCommand);
		  li = l+1;
		  break;
		}
	      }
	      for(l; l <= len; l++){
		if(bFileLine[l] == ';'){
		  bFileLine[l] = '\0';
		  sscanf(&bFileLine[li],"%s",bFileTraj);
		  li = l+1;
		  break;
		}
	      }
	      for(l; l <= len; l++){
		if(bFileLine[l] == ';'){
		  bFileLine[l] = '\0';
		  sscanf(&bFileLine[li],"%s",bFileTrajName);
		  li = l+1;
		  break;
		}
	      }
	      for(l; l <= len; l++){
		if(bFileLine[l] == ';'){
		  bFileLine[l] = '\0';
		  sscanf(&bFileLine[li],"%f",&fileTrajVersion);
		  li = l+1;
		  break;
		}
	      }
	      for(l; l <= len; l++){
		if(bFileLine[l] == ';'){
		  bFileLine[l] = '\0';
		  sscanf(&bFileLine[li],"%d",&fileRows);
		  li = l+1;
		  break;
		}
	      }
	      fpos = ftell(fp);
	      
	    } // current command = DOPM
	} // receivedNewData

#ifdef DEBUG
      printf("INF: Done handling incoming message\n");
#endif

      /* Send monitor start */
      bzero(bMonitorBuffer, 256);
      strcpy(bMonitorBuffer, "MONR;");

      gettimeofday(&tv, NULL);

      unsigned long long msSinceEpochETSI = (unsigned long long) tv.tv_sec * 1000 + (unsigned long long) tv.tv_usec / 1000 - 
	MS_FROM_1970_TO_2004_NO_LEAP_SECS + NBR_LEAP_SECONDS_FROM_1970 * 1000;
    
      sprintf(pcTimeString, "%llu", msSinceEpochETSI); 
      strcat(bMonitorBuffer,pcTimeString);

      static int sent_rows = 0;
      
#ifdef DEBUG
      printf("INF: trig_time %d sent_rows %d rows %d \n",trig_time,sent_rows,fileRows);
#endif
      
      if (trig_time < INT_MAX && sent_rows < fileRows)
	{
#ifdef DEBUG
	  printf("INF: Create row %d from drive file.\n",sent_rows);
#endif
	  bzero(pcPosition,256);

	  read = getline(&bFileLine, &len, fp);
	  if (read == -1)
	    {
	      perror("ERR: Failed to read first line in drive file");
	      exit(1); 
	    }
	  
	  int l = 0;
	  int li = 0;
	  for(l; l <= len; l++){
	    if(bFileLine[l] == ';'){
	      bFileLine[l] = '\0';
	      bzero(bLine,10);
	      sscanf(&bFileLine[li],"%s",bLine);
	      li = l+1;
	      break;
	    }
	  }
	  for(l; l <= len; l++){
	    if(bFileLine[l] == ';'){
	      bFileLine[l] = '\0';
	      sscanf(&bFileLine[li],"%f",&time);
	      li = l+1;
	      break;
	    }
	  }
	  for(l; l <= len; l++){
	    if(bFileLine[l] == ';'){
	      bFileLine[l] = '\0';
	      sscanf(&bFileLine[li],"%lf",&x);
	      li = l+1;
	      break;
	    }
	  }
	  for(l; l <= len; l++){
	    if(bFileLine[l] == ';'){
	      bFileLine[l] = '\0';
	      sscanf(&bFileLine[li],"%lf",&y);
	      li = l+1;
	      break;
	    }
	  }
	  for(l; l <= len; l++){
	    if(bFileLine[l] == ';'){
	      bFileLine[l] = '\0';
	      sscanf(&bFileLine[li],"%lf",&z);
	      li = l+1;
	      break;
	    }
	  }
	  
	  for(l; l <= len; l++){
	    if(bFileLine[l] == ';'){
	      bFileLine[l] = '\0';
	      sscanf(&bFileLine[li],"%f",&hdg);
	      li = l+1;
	      break;
	    }
	  }
	  for(l; l <= len; l++){
	    if(bFileLine[l] == ';'){
	      bFileLine[l] = '\0';
	      sscanf(&bFileLine[li],"%f",&vel);
	      li = l+1;
	      break;
	    }
	  }

	  cal_lat = ORIGIN_LATITUDE - ((y*180)/(PI*earth_radius));
	  cal_lon = ORIGIN_LONGITUDE - ((x*180)/(PI*earth_radius))*(1/(cos((PI/180)*(0.5*(ORIGIN_LATITUDE+cal_lat)))));
	  cal_alt = ORIGIN_ALTITUDE - z;
      
	  sprintf(pcPosition,";%.0lf;%.0lf;%.0f;%.0f;3600;0;",cal_lat*10000000,cal_lon*10000000,cal_alt*100,vel*100);
	  sent_rows++;
	}
      else if (sent_rows == fileRows)
	{
	  bzero(pcPosition,256);
	  sprintf(pcPosition,";%.0lf;%.0lf;%.0f;%.0f;3600;0;",cal_lat*10000000,cal_lon*10000000,cal_alt*100,vel*100);
	}
      else
	{
	  bzero(pcPosition,256);
	  sprintf(pcPosition,";%.0lf;%.0lf;0;0;3600;0;",lat_start*10000000,lon_start*10000000);
	}

      strcat(bMonitorBuffer,pcPosition);
#ifdef DEBUG
      printf("INF: Start send monitor message\n");
#endif
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
      sleep_time.tv_nsec = 100000000;
      (void) nanosleep(&sleep_time, &ref_time);
    }

  close(monitor_socket_fd);
  close(command_com_socket_fd);
  fclose(fp);

  return 0;
}
