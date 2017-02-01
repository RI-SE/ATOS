/************tcpclient.c************************/
/* Header files needed to use the sockets API. */
/* File contains Macro, Data Type and */
/* Structure definitions along with Function */
/* prototypes. */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <errno.h>

#include "nmea2etsi.h"

#define BufferLength 80

#define SERVER "127.0.0.1"

#define SERVPORT 2948

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


int main(int argc, char *argv[])
{
  /* Variable and structure definitions. */
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
      /*Use the default server name or IP*/
      strcpy(server, SERVER);
   
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

    for(;;){
      rc = read(sd, &character, 1);
	  if(rc < 0)
	    {
	      perror("Client-read() error");
	      close(sd);
	      exit(-1);
	    }
	  else if (rc == 0)
	    {
	      printf("Server program has issued a close()\n");
	      close(sd);
	      exit(-1);
	    }
	  if (character != '\n' && i < BufferLength)
	    {
	      sentence[i] = character;
	      i++;
	    }
	  else
	    {
	      sentence[i] = '\0';
	      i = 0;

	      getField(nmea_msg, 0);
	      if (strcmp(nmea_msg, "$GPRMC") == 0) {
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
	      else if (strcmp(nmea_msg, "$GPGGA") == 0) {
		getField(utc, 1);
		getField(latitude, 2);
		getField(northsouth, 3);
		getField(longitude, 4);
		getField(eastwest, 5);
		getField(gps_quality_indicator, 6);
		getField(satellites_used, 7);
		getField(antenna_altitude, 9);
	      }
	      else if (strcmp(nmea_msg, "$GPGSV") == 0) {
		getField(satellites_in_view, 3);
	      }
	      else if (strcmp(nmea_msg, "$GPGSA") == 0) {
		getField(pdop, 4);
		getField(hdop, 5);
		getField(vdop, 6);
	      }
	    }
	  if(i == 0)
	    {
	      etsi_lat          = ConvertLatitudeNMEAtoETSICDD(latitude, northsouth, status);
	      etsi_lon          = ConvertLongitudeNMEAtoETSICDD(longitude, eastwest, status);
	      etsi_speed        = ConvertSpeedValueNMEAtoETSICDD(gps_speed, status);
	      etsi_heading      = ConvertHeadingValueNMEAtoETSICDD(gps_heading, status);
	      etsi_alt          = ConvertAltitudeValueNMEAtoETSICDD(antenna_altitude, status);
	      etsi_time         = ConvertTimestapItsNMEAtoETSICDD(utc, date, status);
	      sprintf(etsi_time_string, "%" PRIu64 "", etsi_time);
	      printf("MONR;%s;%09d;%010d;%06d;%05d;%04d;0;\n",etsi_time_string,etsi_lat,etsi_lon,etsi_alt,etsi_speed,etsi_heading);
	    }
	  
      }

    close(sd);
    exit(0);
    return 0;
}
