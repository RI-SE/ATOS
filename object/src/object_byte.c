/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  --------------------------------------------------------------------------------
  -- File        : object.c
  -- Author      : Henrik Eriksson
  -- Description : CHRONOS software prototype object
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
#define TIME_FROM_DRIVE_FILE 1

/* 34 years between 1970 and 2004, 8 days for leap year between 1970 and 2004      */
/* Calculation: 34 * 365 * 24 * 3600 * 1000 + 8 * 24 * 3600 * 1000 = 1072915200000 */
#define MS_FROM_1970_TO_2004_NO_LEAP_SECS 1072915200000

/* Difference of leap seconds between UTC and ETSI */
#define DIFF_LEAP_SECONDS_UTC_ETSI 5

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/

uint8_t ReadOneUByteFromFile(FILE *f);
uint16_t ReadTwoUBytesFromFile(FILE *f);
uint32_t ReadFourUBytesFromFile(FILE *f);
int8_t ReadOneSByteFromFile(FILE *f);
int16_t ReadTwoSBytesFromFile(FILE *f);
int32_t ReadFourSBytesFromFile(FILE *f);

/*------------------------------------------------------------
  -- The main function.
  ------------------------------------------------------------*/

int32_t main(int32_t argc, int8_t *argv[])
{
  FILE *fp;
  int8_t bFileName[6];
  int8_t *bFileLine;
  //int8_t *bFileLine_ptr = bFileLine;
  size_t len;
  size_t read;
  int32_t monitor_socket_fd;
  int32_t command_server_socket_fd;
  int32_t command_com_socket_fd;
  int32_t n;
  int32_t pid;
  int32_t fpos;
  socklen_t cli_length;
  struct sockaddr_in monitor_server_addr;
  struct sockaddr_in monitor_from_addr;
  struct sockaddr_in command_server_addr;
  struct sockaddr_in cli_addr;
  struct hostent *hp;
  int8_t buffer[256];
  int8_t bMonitorBuffer[25];
  int8_t pcPosition[256];
  struct timespec sleep_time;
  struct timespec ref_time;
  int8_t pcTimeString[15];
  int8_t *ret = NULL;
  int8_t bCurrentCommand[10] = "NOOP"; 
  int8_t bData[DATALEN];
  int32_t newFileData = 0;
  int8_t bFileTrajName[100];
  int8_t bCommand[10];
  int8_t bFileCommand[10];
  int8_t bFileTraj[10];
  int8_t bLine[10];
  int8_t bEndLine[10];
  float fileTrajVersion;
  int32_t row = 0;
  int32_t fileRows = 0;
  int32_t result = 0;
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
  uint32_t safety_port = SAFETY_CHANNEL_PORT;
  uint32_t control_port = CONTROL_CHANNEL_PORT;
  int32_t optval = 1;
  uint64_t msSinceEpochETSI = 0;

  uint16_t buffer_ptr = 0;
  uint32_t payload = 0;
  
  int32_t  origin_latitude  = 0;
  int32_t  origin_longitude = 0;
  int32_t  origin_altitude  = 0;
  uint16_t origin_heading   = 0;

  uint8_t start_type  = 0;
  uint64_t start_time = INT_MAX;
   
  uint8_t arm_mode  = 0;

  uint8_t  cmd  = 0;
  uint32_t size = 0;
  uint32_t btim = 0;
  uint32_t bx   = 0;
  uint32_t by   = 0;
  uint32_t bz   = 0;
  int16_t  bhdg = 0;
  int16_t  bspd = 0;
  int16_t  bacc = 0;
  uint16_t bcur = 0;
  uint8_t  mod  = 0;
  uint8_t  mon_status = 0;
  uint32_t dopm_bytes = 0;
  uint8_t firstTime = 0;
  
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
  fp = fopen(bFileName, "wb+");
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
  bzero((int8_t *) &command_server_addr, sizeof(command_server_addr));

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

  /* Set socket to non-blocking */
  result = fcntl(command_com_socket_fd, F_SETFL, fcntl(command_com_socket_fd, F_GETFL, 0) | O_NONBLOCK);

#ifdef DEBUG
  printf("INF: Received: <%s>\n", buffer);
#endif

  while(1)
    {
      int32_t receivedNewData = 0;
      int32_t commandSet = 0;
      bzero(bData,DATALEN);
      bzero(buffer,256);
      do
	{
#ifdef DEBUG
	  printf("INF: Start receive\n");
#endif
	  uint16_t handled_payload = 0;
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
	      buffer_ptr = 0;

#ifdef DEBUG
	      printf("INF: Received: Result = %d, <%d> <%x>\n",result, (uint8_t) buffer[buffer_ptr], (uint8_t) buffer[buffer_ptr]);
	      fflush(stdout);
#endif

	      /* int32_t k = 0; */
	      /* for (k = 0; k < result; k++) */
	      /* 	printf("%x,",(uint8_t) buffer[k]); */
	      /* printf("\n"); */

	      do
		{
		  if (commandSet == 0){
		    payload = 0;
		    handled_payload = 0;
		    switch(buffer[buffer_ptr++]){
		    case 0x01:
#ifdef DEBUG
		      printf("INF: Current command set to DOPM\n");
#endif
		      bzero(bCurrentCommand, 10);
		      strcpy(bCurrentCommand, "DOPM");
		      commandSet = 1;
		      payload |= (uint8_t) buffer[buffer_ptr] << 24;
		      payload |= (uint8_t) buffer[buffer_ptr+1] << 16;
		      payload |= (uint8_t) buffer[buffer_ptr+2] << 8;
		      payload |= (uint8_t) buffer[buffer_ptr+3];
		      printf("Payload = %d\n", payload);
		      dopm_bytes = payload;
		      mon_status = 0x0;
		      firstTime = 0;
		      buffer_ptr--;
		      break;
		    case 0x02:
#ifdef DEBUG
		      printf("INF: Current command set to OSEM\n");
#endif
		      bzero(bCurrentCommand, 10);
		      strcpy(bCurrentCommand, "OSEM");
		      commandSet = 1;
		      payload |= (uint8_t) buffer[buffer_ptr++] << 24;
		      payload |= (uint8_t) buffer[buffer_ptr++] << 16;
		      payload |= (uint8_t) buffer[buffer_ptr++] << 8;
		      payload |= (uint8_t) buffer[buffer_ptr++];
		      printf("Payload = %d\n", payload);
		      origin_latitude  |= (uint8_t) buffer[buffer_ptr++] << 24;
		      origin_latitude  |= (uint8_t) buffer[buffer_ptr++] << 16;
		      origin_latitude  |= (uint8_t) buffer[buffer_ptr++] << 8;
		      origin_latitude  |= (uint8_t) buffer[buffer_ptr++];
		      origin_longitude |= (uint8_t) buffer[buffer_ptr++] << 24;
		      origin_longitude |= (uint8_t) buffer[buffer_ptr++] << 16;
		      origin_longitude |= (uint8_t) buffer[buffer_ptr++] << 8;
		      origin_longitude |= (uint8_t) buffer[buffer_ptr++];
		      origin_altitude  |= (uint8_t) buffer[buffer_ptr++] << 24;
		      origin_altitude  |= (uint8_t) buffer[buffer_ptr++] << 16;
		      origin_altitude  |= (uint8_t) buffer[buffer_ptr++] << 8;
		      origin_altitude  |= (uint8_t) buffer[buffer_ptr++];
		      origin_heading   |= (uint8_t) buffer[buffer_ptr++] << 8;
		      origin_heading   |= (uint8_t) buffer[buffer_ptr++];
		      handled_payload += 14;
		      commandSet = 0;
		      printf("olat: %d <%x> olon: %d <%x> oalt: %d <%x> ohdg: %d <%x> Handled payload: %d\n", origin_latitude, origin_latitude, origin_longitude, origin_longitude, origin_altitude, origin_altitude, origin_heading, origin_heading, payload);
		      break;
		    case 0x03:
#ifdef DEBUG
		      printf("INF: Current command set to AROM\n");
#endif
		      bzero(bCurrentCommand, 10);
		      strcpy(bCurrentCommand, "AROM");
		      commandSet = 1;
		      payload |= (uint8_t) buffer[buffer_ptr++] << 24;
		      payload |= (uint8_t) buffer[buffer_ptr++] << 16;
		      payload |= (uint8_t) buffer[buffer_ptr++] << 8;
		      payload |= (uint8_t) buffer[buffer_ptr++];
		      printf("Payload = %d\n", payload);
		      arm_mode |= (uint8_t) buffer[buffer_ptr++];
		      handled_payload += 1;
		      if (arm_mode == 1){
			printf("Armed.\n");
			mon_status = 0x1;
		      }
		      else if (arm_mode == 2){
			printf("Disarmed.\n");
			mon_status = 0x0;
		      }
		      else{
			printf("Unknown arm mode.\n");
			mon_status = 0x4;
		      }
		      commandSet = 0;
		      break;
		    case 0x04:
#ifdef DEBUG
		      printf("INF: Current command set to STRT\n");
#endif
		      bzero(bCurrentCommand, 10);
		      strcpy(bCurrentCommand, "STRT");
		      commandSet = 1;
		      payload |= (uint8_t) buffer[buffer_ptr++] << 24;
		      payload |= (uint8_t) buffer[buffer_ptr++] << 16;
		      payload |= (uint8_t) buffer[buffer_ptr++] << 8;
		      payload |= (uint8_t) buffer[buffer_ptr++];
		      printf("Payload = %d\n", payload);
		      start_type |= (uint8_t) buffer[buffer_ptr++];
		      handled_payload += 1;
		      printf("start_type: %d ",start_type);
		      if (start_type == 2){
			start_time |= (uint8_t) buffer[buffer_ptr++] << 40;
			start_time |= (uint8_t) buffer[buffer_ptr++] << 32;
			start_time |= (uint8_t) buffer[buffer_ptr++] << 24;
			start_time |= (uint8_t) buffer[buffer_ptr++] << 16;
			start_time |= (uint8_t) buffer[buffer_ptr++] << 8;
			start_time |= (uint8_t) buffer[buffer_ptr++];
			handled_payload += 6;
			printf("start_time: %ld", start_time);
		      }
		      else
			start_time = 0;
		      printf("\n");
		      commandSet = 0;
		      break;
		    default:
#ifdef DEBUG
		      printf("INF: Unknown command\n");
#endif
		      bzero(bCurrentCommand, 10);
		      strcpy(bCurrentCommand, "UKNW");
		      commandSet = 0;
		    }
		  }

		 if (!strncmp(bCurrentCommand, "DOPM", 4))
		   {
		     while ((buffer_ptr < result) && (buffer_ptr < 5 + payload))
		       {
			 fwrite(&buffer[buffer_ptr], sizeof(buffer[buffer_ptr]), 1, fp);
			 buffer_ptr++;
			 handled_payload++;
		       }
		   } 
		} while(buffer_ptr < result);
	      
	    }
	} while(result > 0);

      /* Send monitor start */
      bzero(bMonitorBuffer, 25);
      if (TIME_FROM_DRIVE_FILE)
	{
	  msSinceEpochETSI = 0;
	}
      else
	{
	  gettimeofday(&tv, NULL);
	  msSinceEpochETSI = (uint64_t) tv.tv_sec * 1000 + (uint64_t) tv.tv_usec / 1000 - 
	    MS_FROM_1970_TO_2004_NO_LEAP_SECS + DIFF_LEAP_SECONDS_UTC_ETSI * 1000;
	}

      static int32_t handled_bytes = 0;
      
#ifdef DEBUG
      printf("INF: start_time %ld handled_bytes %d bytes %d \n",start_time,handled_bytes,dopm_bytes);
#endif
      
      if ((msSinceEpochETSI > start_time) && (sent_rows < fileRows))
	{
#ifdef DEBUG
	  printf("INF: Create row %d from drive file.\n",sent_rows);
#endif
	  if (firstTime == 1){
	    cmd = ReadOneUByteFromFile(fd);
	    size = ReadFourUBytesFromFile(fd);
	    firstTime = 0;
	  }
	  
	  btim = ReadFourUBytesFromFile(fd);
	  bx = ReadFourUBytesFromFile(fd);
	  by = ReadFourUBytesFromFile(fd);
	  bz = ReadFourUBytesFromFile(fd);
	  bhdg = ReadTwoSBytesFromFile(fd);
	  bspd = ReadTwoSBytesFromFile(fd);
	  bacc = ReadTwoSBytesFromFile(fd);
	  bcur = ReadTwoUBytesFromFile(fd);
	  mod = ReadOneUByteFromFile(fd);
	  handled_bytes += 25;

	  cal_lat = (((((double) by) / 1000) * 180) / (PI * earth_radius)) + (((double) origin_latitude) / 10000000);
	  cal_lon = (((((double) bx) / 1000) * 180) / (PI * earth_radius)) * (1 / (cos((PI / 180) * (0.5 * ((((double) origin_latitude) / 10000000)+cal_lat))))) + (((double) origin_longitude) / 10000000);
	  cal_alt = (((double) bz) / 1000) + (((double) origin_altitude) / 100);

	  if (TIME_FROM_DRIVE_FILE)
	    {
	      msSinceEpochETSI = btime;
	    }
	  else
	    {
	      gettimeofday(&tv, NULL);
	      msSinceEpochETSI = (uint64_t) tv.tv_sec * 1000 + (uint64_t) tv.tv_usec / 1000 - 
		MS_FROM_1970_TO_2004_NO_LEAP_SECS + DIFF_LEAP_SECONDS_UTC_ETSI * 1000;
	    }

	}
      else if (handled_bytes == dopm_bytes)
	{
	  if (TIME_FROM_DRIVE_FILE)
	    {
	      msSinceEpochETSI = btime;
	    }
	  else
	    {
	      gettimeofday(&tv, NULL);
	      msSinceEpochETSI = (uint64_t) tv.tv_sec * 1000 + (uint64_t) tv.tv_usec / 1000 - 
		MS_FROM_1970_TO_2004_NO_LEAP_SECS + DIFF_LEAP_SECONDS_UTC_ETSI * 1000;
	    }
	}
      else
	{
	  if (firstTime == 1){
	    cmd = ReadOneUByteFromFile(fd);
	    size = ReadFourUBytesFromFile(fd);
	    btim = ReadFourUBytesFromFile(fd);
	    bx = ReadFourUBytesFromFile(fd);
	    by = ReadFourUBytesFromFile(fd);
	    bz = ReadFourUBytesFromFile(fd);
	    bhdg = ReadTwoSBytesFromFile(fd);
	    bspd = ReadTwoSBytesFromFile(fd);
	    bacc = ReadTwoSBytesFromFile(fd);
	    bcur = ReadTwoUBytesFromFile(fd);
	    mod = ReadOneUByteFromFile(fd);
	  }
	  frewind(fd);
	  cal_lat = (((((double) by) / 1000) * 180) / (PI * earth_radius)) + (((double) origin_latitude) / 10000000);
	  cal_lon = (((((double) bx) / 1000) * 180) / (PI * earth_radius)) * (1 / (cos((PI / 180) * (0.5 * ((((double) origin_latitude) / 10000000)+cal_lat))))) + (((double) origin_longitude) / 10000000);
	  cal_alt = (((double) bz) / 1000) + (((double) origin_altitude) / 100);
	}

      strcat(bMonitorBuffer,pcTimeString);
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

uint8_t ReadOneUByteFromFile(FILE *f){
  uint8_t data = 0;
  fread(&data, 1, 1, f);
  return data;
}

uint16_t ReadTwoUBytesFromFile(FILE *f){
  uint8_t tmp = 0;
  uint16_t data = 0;
  fread(&tmp, 1, 1, f);
  data |= tmp << 8;
  fread(&tmp, 1, 1, f);
  data |= tmp;
  return data;
}

uint32_t ReadFourUBytesFromFile(FILE *f){
  uint8_t tmp = 0;
  uint32_t data = 0;
  fread(&tmp, 1, 1, f);
  data |= tmp << 24;
  fread(&tmp, 1, 1, f);
  data |= tmp << 16;
  fread(&tmp, 1, 1, f);
  data |= tmp << 8;
  fread(&tmp, 1, 1, f);
  data |= tmp;
  return data;
}

int8_t ReadOneSByteFromFile(FILE *f){
  int8_t data = 0;
  fread(&data, 1, 1, f);
  return data;
}

int16_t ReadTwoSBytesFromFile(FILE *f){
  uint8_t tmp = 0;
  int16_t data = 0;
  fread(&tmp, 1, 1, f);
  data |= tmp << 8;
  fread(&tmp, 1, 1, f);
  data |= tmp;
  return data;
}

  int32_t ReadFourSBytesFromFile(FILE *f){
  uint8_t tmp = 0;
  int32_t data = 0;
  fread(&tmp, 1, 1, f);
  data |= tmp << 24;
  fread(&tmp, 1, 1, f);
  data |= tmp << 16;
  fread(&tmp, 1, 1, f);
  data |= tmp << 8;
  fread(&tmp, 1, 1, f);
  data |= tmp;
  return data;
}
