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

#define SAFETY_CHANNEL_PORT 53240
#define CONTROL_CHANNEL_PORT 53241
#define ORIGIN_LATITUDE 57.7773602
#define ORIGIN_LONGITUDE 012.7804715
#define ORIGIN_ALTITUDE 201.485
#define DATALEN 10000
#define SAVED_TRAJECTORY_LINES 100
#define PI acos(-1)
#define DEBUG

typedef struct {
  float time;
  double x;
  double y;
  double z;
  float hdg;
  float vel;
  float acc;
  float curv;
  uint8_t mode;
} traj_row_t;

/*------------------------------------------------------------
  -- The main function.
  ------------------------------------------------------------*/

int main(int argc, char *argv[])
{
  int monitor_socket_fd,command_server_socket_fd, command_com_socket_fd, n,pid;
  int trig_time = INT_MAX;
  socklen_t cli_length;
  struct sockaddr_in monitor_server_addr, monitor_from_addr, command_server_addr, cli_addr;
  struct hostent *hp;
  char buffer[256];
  char bMonitorBuffer[256];
  char pcPosition[256];
  struct timespec sleep_time, ref_time;
  char pcTimeString[15];
  char *ret = NULL;
  char bCurrentCommand[10] = "NOP!"; 
  const char bEndDOPM[10]= "ENDDOPM";
  const char bEndTRIG[10]= "ENDTRIG";
  char bData[DATALEN];
  char cmd_complete = 0;
  traj_row_t traj[SAVED_TRAJECTORY_LINES];
  char bTrajName[100];
  char bCommand[10];
  char bTraj[10];
  char bLine[10];
  char bEndLine[10];
  float trajVersion;
  int row = 0;
  int rows;
  int k = 0;
  int result;
  double cal_lat = 0.0;
  double cal_lon = 0.0;
  double cal_alt = 0.0;
  double earth_radius = 6378137.0;
  double lat_start = ORIGIN_LATITUDE;
  double lon_start = ORIGIN_LONGITUDE;
  double alt_start = ORIGIN_ALTITUDE;
  unsigned int safety_port = SAFETY_CHANNEL_PORT;
  unsigned int control_port = CONTROL_CHANNEL_PORT;

  struct timeval tv;
  unsigned long long msFrom1970To2004 = ((unsigned long long) 34 * 365 * 24 * 3600 * 1000) + ((unsigned long long) 8 * 24 * 3600 * 1000) + ((unsigned long long) 22 * 1000); 
  

  
  if ( !( (argc == 1) || (argc == 4) || (argc == 6))){
    perror("Please use no, 3, or 5 arguments: object <lat,lon,alt><safety port,control port>");
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

  /* Init monitor socket */
  #ifdef DEBUG
  printf("INF: Init monitor socket\n");
  fflush(stdout);
  #endif

  monitor_socket_fd=socket(AF_INET, SOCK_DGRAM, 0);
  if (monitor_socket_fd < 0)
  {
    perror("ERR: Failed to create monitor socket");
    exit(1);
  }
  bzero(&monitor_server_addr,sizeof(monitor_server_addr));
  monitor_server_addr.sin_family=AF_INET;
  monitor_server_addr.sin_addr.s_addr=INADDR_ANY;
  monitor_server_addr.sin_port=htons(safety_port);

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

  if (result < 0)
  {
    perror("calling fcntl");
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

  listen(command_server_socket_fd,5);
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
  n = recvfrom(monitor_socket_fd,buffer,1024,0,
    (struct sockaddr *) &monitor_from_addr, &fromlen);
  if (n < 0) 
  {
    perror("ERR: Failed to receive from central");
    exit(1);
  }

  /* set socket to non-blocking */
  result = fcntl(command_com_socket_fd, F_SETFL, 
    fcntl(command_com_socket_fd, F_GETFL, 0) | O_NONBLOCK);

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
      printf("INF: Start for receive\n");
      #endif

      result = recv(command_com_socket_fd, 
        buffer,
        256,
        0);

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
        strncat(bData,buffer,result);
      }
    } while(result > 0 );

    
    /* Did we recieve anything on command? */
    if(receivedNewData)
    {
      if (!strncmp(bData,"DOPM",4))
        {
          #ifdef DEBUG
	  printf("INF: Current command set to DOPM\n");
          #endif
          bzero(bCurrentCommand,10);
          strcpy(bCurrentCommand,"DOPM");
        }
      else if (!strncmp(bData,"TRIG",4))
        {
          #ifdef DEBUG
	  printf("INF: Current command set to TRIG\n");
          #endif
          bzero(bCurrentCommand,10);
          strcpy(bCurrentCommand,"TRIG");
          trig_time = 0;  
        }

      if(!strncmp(bCurrentCommand,"DOPM",4))
      {
        #ifdef DEBUG
          printf("INF: DOPM is current command\n");
        #endif

	  //printf("!!! %s !!!\n",bData);

        ret = strstr(bData,bEndDOPM);
        if(ret)
        {
          #ifdef DEBUG
            printf("INF: Complete DOPM is received\n");
          #endif
          cmd_complete = 1;
          strcat(bData,"\0");

          int k = 0;
          int ki = 0;
          for(k = 0; k < DATALEN; k++){
            if(bData[k] == ';'){
              bData[k] = '\0';
              sscanf(&bData[ki],"%s",bCommand);
              ki = k+1;
              break;
            }
          }
          for(k; k < DATALEN; k++){
            if(bData[k] == ';'){
              bData[k] = '\0';
              sscanf(&bData[ki],"%s",bTraj);
              ki = k+1;
              break;
            }
          }
          for(k; k < DATALEN; k++){
            if(bData[k] == ';'){
              bData[k] = '\0';
              sscanf(&bData[ki],"%s",bTrajName);
              ki = k+1;
              break;
            }
          }
          for(k; k < DATALEN; k++){
            if(bData[k] == ';'){
              bData[k] = '\0';
              sscanf(&bData[ki],"%f",&trajVersion);
              ki = k+1;
              break;
            }
          }
          for(k; k < DATALEN; k++){
            if(bData[k] == ';'){
              bData[k] = '\0';
              sscanf(&bData[ki],"%d",&rows);
              ki = k+1;
              break;
            }
          }
          k++;
          ki++;
          for (row = 0; row < rows; row++){
            for(k; k < DATALEN; k++){
              if(bData[k] == ';'){
                bData[k] = '\0';
                bzero(bLine,10);
                sscanf(&bData[ki],"%s",bLine);
                ki = k+1;
                break;
              }
            }
            for(k; k < DATALEN; k++){
              if(bData[k] == ';'){
                bData[k] = '\0';
                sscanf(&bData[ki],"%f",&traj[row].time);
                ki = k+1;
                break;
              }
            }
            for(k; k < DATALEN; k++){
              if(bData[k] == ';'){
                bData[k] = '\0';
                sscanf(&bData[ki],"%lf",&traj[row].x);
                ki = k+1;
                break;
              }
            }
            for(k; k < DATALEN; k++){
              if(bData[k] == ';'){
                bData[k] = '\0';
                sscanf(&bData[ki],"%lf",&traj[row].y);
                ki = k+1;
                break;
              }
            }
            for(k; k < DATALEN; k++){
              if(bData[k] == ';'){
                bData[k] = '\0';
                sscanf(&bData[ki],"%lf",&traj[row].z);
                ki = k+1;
                break;
              }
            }

            for(k; k < DATALEN; k++){
              if(bData[k] == ';'){
                bData[k] = '\0';
                sscanf(&bData[ki],"%f",&traj[row].hdg);
                ki = k+1;
                break;
              }
            }
            for(k; k < DATALEN; k++){
              if(bData[k] == ';'){
                bData[k] = '\0';
                sscanf(&bData[ki],"%f",&traj[row].vel);
                ki = k+1;
                break;
              }
            }
            for(k; k < DATALEN; k++){
              if(bData[k] == ';'){
                bData[k] = '\0';
                sscanf(&bData[ki],"%f",&traj[row].acc);
                ki = k+1;
                break;
              }
            }
            for(k; k < DATALEN; k++){
              if(bData[k] == ';'){
                bData[k] = '\0';
                sscanf(&bData[ki],"%f",&traj[row].curv);
                ki = k+1;
                break;
              }
            }
            for(k; k < DATALEN; k++){
              if(bData[k] == ';'){
                bData[k] = '\0';
                sscanf(&bData[ki],"%" SCNu8 "",&traj[row].mode);
                ki = k+1;
                break;
              }
            }
            for(k; k < DATALEN; k++){
              if(bData[k] == ';'){
                bData[k] = '\0';
                bzero(bEndLine,10);
                sscanf(&bData[ki],"%s",bEndLine);
                ki = k+1;
                break;
              }
            }
	    //printf("??? %s;%f;%f;%f;%f;%f;%f;%f;%f;%d;%s %d %d\n",bLine,traj[row].time,traj[row].x,traj[row].y,traj[row].z,traj[row].hdg,traj[row].vel,traj[row].acc,traj[row].curv,traj[row].mode,bEndLine,k,ki);
            k++;
            ki++;
          } // row
        } // endcommand
      } // current command DOPM
    } // receivedNewData

    #ifdef DEBUG
      printf("INF: Done handling incoming message\n");
    #endif

    /* Send monitor start */
    bzero(bMonitorBuffer,256);
    strcpy(bMonitorBuffer,"MONR;");

    gettimeofday(&tv, NULL);

    unsigned long long msSinceEpoch = (unsigned long long) (tv.tv_sec) * 1000 + (unsigned long long) (tv.tv_usec) / 1000;
    
    sprintf(pcTimeString, "%llu", msSinceEpoch - msFrom1970To2004); 
    strcat(bMonitorBuffer,pcTimeString);

    static int sent_rows = 0;
    #ifdef DEBUG
      printf("INF: trig_time %d sent_rows %d rows %d \n",trig_time,sent_rows,rows);
    #endif
    if (trig_time < INT_MAX && sent_rows < rows)
      {
      #ifdef DEBUG
    	 printf("INF: Create row %d from drive file.\n",sent_rows);
      #endif
      bzero(pcPosition,256);

      cal_lat = ORIGIN_LATITUDE - ((traj[sent_rows].y*180)/(PI*earth_radius));
      cal_lon = ORIGIN_LONGITUDE - ((traj[sent_rows].x*180)/(PI*earth_radius))*(1/(cos((PI/180)*(0.5*(ORIGIN_LATITUDE+cal_lat)))));
      cal_alt = ORIGIN_ALTITUDE - traj[sent_rows].z;
      
      sprintf(pcPosition,";%.0lf;%.0lf;%.0f;%.0f;3600;0;",cal_lat*10000000,cal_lon*10000000,cal_alt*100,traj[sent_rows].vel*100);
      sent_rows++;
    }
    else if (sent_rows == rows)
      {
	bzero(pcPosition,256);
	sprintf(pcPosition,";%.0lf;%.0lf;%.0f;%.0f;3600;0;",cal_lat*10000000,cal_lon*10000000,cal_alt*100,traj[sent_rows].vel*100);
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
    n=sendto(monitor_socket_fd,
      bMonitorBuffer,
      sizeof(bMonitorBuffer),
      0,
      (struct sockaddr *) &monitor_from_addr,
      fromlen);
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
    (void)nanosleep(&sleep_time,&ref_time);
  }

  close(monitor_socket_fd);
  close(command_com_socket_fd);

  return 0;
}
