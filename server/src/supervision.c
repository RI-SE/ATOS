/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  ------------------------------------------------------------------------------
  -- File        : supervision.c
<<<<<<< HEAD
  -- Author      :
=======
  -- Author      : Karl-Johan Ode
>>>>>>> 0b7cb4faa4eebcad7a5244c83897fd7de13278b5
  -- Description : CHRONOS
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

/*------------------------------------------------------------
  -- Include files.
  ------------------------------------------------------------*/

#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <netdb.h>
#include <netinet/in.h>

#include "util.h"
#include "supervision.h"
#include "logging.h"

/*------------------------------------------------------------
  -- Defines
  ------------------------------------------------------------*/
#define SUPERVISOR_CONF_FILE_PATH "conf/supervision.conf"

#define CONF_SUPERVISION_LEVEL_STR "SupervisionLevel="
#define CONF_SUPERVISION_FULL 1
#define CONF_SUPERVISION_PRINT 4
#define CONF_SUPERVISION_PRINT_STATUS 8

#define CONF_SUPERVISION_SAFETY_MARGIN_POS_STR "SafetyMarginPosition="
#define CONF_SUPERVISION_SAFETY_MARGIN_TIM_STR "SafetyMarginTime="
#define CONF_SUPERVISION_SAFETY_MARGIN_MON_TIM_STR "SafetyMarginMonTime="
#define CONF_SUPERVISION_ALERT_MARGIN_POS_STR "AlertMarginPosition="
#define CONF_SUPERVISION_ALERT_MARGIN_TIM_STR "AlertMarginTime="

#define SMALL_BUFFER_SIZE 20
#define STATUS_MESSAGE_BUFFER_SIZE 256

#define TIME_FP_MULTIPLIER 100
#define MS_PER_SEC 1000

#define POS_FP_MULTIPLIER 10000000
#define ALT_FP_MULTIPLIER 100

<<<<<<< HEAD
#define LDM_SIZE            5
#define RECV_MESSAGE_BUFFER 1024
#define SAFETY_MARGIN_POS   0.5
#define SAFETY_MARGIN_TIM   0.5
=======
#define ACTIVE_POINT_VISUALISATION_MESSAGE "SAP"
#define ABORT_VISUALISATION_MESSAGE "ABRT"
#define ALERT_VISUALISATION_MESSAGE "ALRT" 
>>>>>>> 0b7cb4faa4eebcad7a5244c83897fd7de13278b5


/*------------------------------------------------------------
  -- typedefs
  ------------------------------------------------------------*/

typedef struct {
  float     time;
  double    x;
  double    y;
  double    z;
  float     hdg;
  float     vel;
} traj_point_info;

/*------------------------------------------------------------
  -- globals.
  ------------------------------------------------------------*/
int  SupervisionLevel = 0;
const char *SupervisionLevelNames[4] = {"[Live]","[X]","[Print]","[Status]"};

// maximum deviation in position (metres)
float SafetyMarginPos=0.0;
// maximum deviation in time (seconds)
float SafetyMarginTim=0.0;
// If last MON received from any object is older than this (seconds) abort
float SafetyMarginMonTim=0.0;

/* Will not abort at this deviation, but will send alert messages to visualiser */
float AlertMarginPos=0.0;
float AlertMarginTim=0.0;

char VisualisationServerName[128]; 

#define MODULE_NAME "Supervisor"
static const LOG_LEVEL logLevel = LOG_LEVEL_DEBUG;

/*------------------------------------------------------------
  -- Function declarations & definitions.
  ------------------------------------------------------------*/

/* time in number of seconds since 1970-01-01 00:00:00 UTC to ETSI in num ms */
uint64_t tv2ETSI(struct timeval tv) {
  return( (uint64_t) tv.tv_sec  * 1000 +
	  (uint64_t) tv.tv_usec / 1000 -
	  MS_FROM_1970_TO_2004_NO_LEAP_SECS +
	  DIFF_LEAP_SECONDS_UTC_ETSI * 1000 );
}

/* create a string indicating the activated supervision features */
void activeLevelsStr(char *message) {
  int s = SupervisionLevel;
  for (int i = 0; i < 4; i ++) {
    if ( s & 1 )
      strncat(message, SupervisionLevelNames[i], 25);
    s = s >> 1;
  }
}

/* printMessage */
void printMessage(char *message) {
  if (SupervisionLevel & CONF_SUPERVISION_PRINT){
    printf("[Supervision] %s\n", message);
    fflush(stdout);
  }
}

/* printStatusMessage */
void printStatusMessage(char *message) {
  if (SupervisionLevel & CONF_SUPERVISION_PRINT_STATUS){
    printf("[Supervision] %s\n", message);
    fflush(stdout);
  }
}

/* Find distance in metres between two x,y,z points.
   Assuming input unit is metres */
double trajPointDistance(traj_point_info p1, traj_point_info p2){

  traj_point_info t;
  t.x = p1.x - p2.x;
  t.y = p1.y - p2.y;
  t.z = p1.z - p2.z;

  return sqrt( t.x * t.x + t.y * t.y + t.z * t.z);

}

double xyDistance(double x1, double y1, double x2, double y2){
  double xd = x1 - x2;
  double yd = y1 - y2;
  return sqrt( xd*xd + yd*yd);
}

int connectVisualServer(int *sockfd, struct sockaddr_in *addr){
  struct hostent *server;

  *sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

  if (*sockfd < 0) {
    return 0; 
  }

  server = gethostbyname(VisualisationServerName);

  if (server == NULL) {
    return 0; /* for now */
  }
  bcopy((char*) server->h_addr, (char*)&addr->sin_addr.s_addr, server->h_length);

  addr->sin_family = AF_INET;
  addr->sin_port   = htons(53400);

  return 1; 
} 

/*------------------------------------------------------------
  -- Task
  ------------------------------------------------------------*/

void supervision_task(TimeType *GPSTime) {

  char           object_traj_file[MAX_OBJECTS][MAX_FILE_PATH];
  char           object_address_name[MAX_OBJECTS][MAX_FILE_PATH];
  int            nbr_objects = 0;

  FILE*          fp[MAX_OBJECTS] = {NULL};
  char           bFileLine[MAX_OBJECTS][3][100];
  char*          bFileLine_p[MAX_OBJECTS] = {NULL};
  size_t         len;
  int            read;
  int            i;

  int            bestFit[MAX_OBJECTS][3];
  double         align[MAX_OBJECTS];
  int            bestFitDone[MAX_OBJECTS];

  float     time       ;
  double    x          ;
  double    y          ;
  double    z          ;
  float     hdg        ;
  float     vel        ;

  uint8_t  iIndex = 0     ;
  uint8_t  jIndex = 0     ;

  uint64_t  timestamp      ;
  int32_t   latitude       ;
  int32_t   longitude      ;
  int32_t   altitude       ;
  uint16_t  speed          ;
  uint16_t  heading        ;
  uint8_t   drivedirection ;
  monitor_t monitor        ;

  char cpBuffer[MQ_MAX_MESSAGE_LENGTH];

  char           pcTempBuffer [512];

  double         devPos = 0.0;
  float          devTim = 0.0;

  struct timeval tv;
  uint64_t msSinceEpochETSI;

  char TextBuffer[SMALL_BUFFER_SIZE];
  char StatusMessage[STATUS_MESSAGE_BUFFER_SIZE];


<<<<<<< HEAD
  struct timeval tv ;
  uint64_t msSinceEpochETSI ;
  LogInit(MODULE_NAME,logLevel);

  LogMessage( LOG_LEVEL_INFO, "Supervision task running with PID: %i", getpid());
=======
  /* Did we get ARM and START messages */
  int Armed = 0;
  int Started = 0;
  int Initialize = 1; /* first iteration should initialize all datastructures */

  uint64_t StartTimestamp = 0;
  uint64_t StartTimeDelay = 0;

  uint64_t CurrentTime    = 0;
>>>>>>> 0b7cb4faa4eebcad7a5244c83897fd7de13278b5

  /* Origo long/lat/alt) */
  double OrigoLatitude = 0.0;
  double OrigoLongitude = 0.0;
  double OrigoAltitude = 0.0;
  double OrigoHeading = 0.0;
  double OrigoLlh[3];

  /* temporary storage during conversion from llh to xyz */
  double PosLlh[3];
  double PosXyz[3];

  traj_point_info traj_point;
  traj_point_info traj_points[MAX_OBJECTS][3]; /* three consequtive points from
						  trajectory file */
  traj_point_info moni_point;
  traj_point_info prev_moni_point[MAX_OBJECTS];
  int             have_prev_moni_point[MAX_OBJECTS];
  float           max_speed[MAX_OBJECTS]; /* km/h */
  float           last_speed[MAX_OBJECTS];

  int traj_done[MAX_OBJECTS]; /* Trajectory has been followed until completion */

  /*--------------------------------------------------
    -- Distance in position during initial alignment.
    -- Distance in time and position during correlation
    --------------------------------------------------*/
  double distance[MAX_OBJECTS][3];

  /*------------------------------------------------------------
    -- udp 
    ------------------------------------------------------------*/
#define TX_BUF_SIZE 4096
  struct sockaddr_in visual_server_addr;
  int    visual_server;
  int    visual_connected = 0; 
  char txBuffer[TX_BUF_SIZE]; 
  
  
  /*----------------------------------------------------------------------
    -- Read SUPERVISION configuration from file
    ----------------------------------------------------------------------*/

  bzero(TextBuffer, SMALL_BUFFER_SIZE);
  UtilSearchTextFile(SUPERVISOR_CONF_FILE_PATH, CONF_SUPERVISION_LEVEL_STR, "", TextBuffer);
  SupervisionLevel = atoi(TextBuffer);

  bzero(TextBuffer, SMALL_BUFFER_SIZE);
  UtilSearchTextFile(SUPERVISOR_CONF_FILE_PATH, CONF_SUPERVISION_SAFETY_MARGIN_POS_STR,
		     "", TextBuffer);
  SafetyMarginPos = atof(TextBuffer);

  bzero(TextBuffer, SMALL_BUFFER_SIZE);
  UtilSearchTextFile(SUPERVISOR_CONF_FILE_PATH, CONF_SUPERVISION_SAFETY_MARGIN_TIM_STR,
		     "", TextBuffer);
  SafetyMarginTim = atof(TextBuffer);

  bzero(TextBuffer, SMALL_BUFFER_SIZE);
  UtilSearchTextFile(SUPERVISOR_CONF_FILE_PATH, CONF_SUPERVISION_SAFETY_MARGIN_MON_TIM_STR,
		     "", TextBuffer);
  SafetyMarginMonTim = atof(TextBuffer);

  bzero(TextBuffer, SMALL_BUFFER_SIZE);
  UtilSearchTextFile(SUPERVISOR_CONF_FILE_PATH, CONF_SUPERVISION_ALERT_MARGIN_POS_STR,
		     "", TextBuffer);
  AlertMarginPos = atof(TextBuffer);

  bzero(TextBuffer, SMALL_BUFFER_SIZE);
  UtilSearchTextFile(SUPERVISOR_CONF_FILE_PATH, CONF_SUPERVISION_ALERT_MARGIN_TIM_STR,
		     "", TextBuffer);
  AlertMarginTim = atof(TextBuffer);
  

  /* Print information about the configuration read from file. */
  snprintf(StatusMessage, STATUS_MESSAGE_BUFFER_SIZE,
	   "Started: Supervision modes ");

  activeLevelsStr(StatusMessage);
  printMessage(StatusMessage);
  bzero(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE);

  snprintf(StatusMessage, STATUS_MESSAGE_BUFFER_SIZE,
	   "Safety margins: pos = %f, time=%f", SafetyMarginPos, SafetyMarginTim);
  printMessage(StatusMessage);

  bzero(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE);

  snprintf(StatusMessage, STATUS_MESSAGE_BUFFER_SIZE,
	   "Alert margins: pos = %f, time=%f", AlertMarginPos, AlertMarginTim);
  printMessage(StatusMessage);


  

  /*----------------------------------------------------------------------
    -- Read SERVER configuration from file
    ----------------------------------------------------------------------*/

  bzero(TextBuffer, SMALL_BUFFER_SIZE);
  UtilSearchTextFile(CONF_FILE_PATH, "OrigoLatitude=", "", TextBuffer);
  OrigoLatitude = atof(TextBuffer);

  bzero(TextBuffer, SMALL_BUFFER_SIZE);
  UtilSearchTextFile(CONF_FILE_PATH, "OrigoLongitude=", "", TextBuffer);
  OrigoLongitude = atof(TextBuffer);

  bzero(TextBuffer, SMALL_BUFFER_SIZE);
  UtilSearchTextFile(CONF_FILE_PATH, "OrigoAltitude=", "", TextBuffer);
  OrigoAltitude = atof(TextBuffer);

  bzero(TextBuffer, SMALL_BUFFER_SIZE);
  UtilSearchTextFile(CONF_FILE_PATH, "OrigoHeading=", "", TextBuffer);
  OrigoHeading = atof(TextBuffer);

  bzero(TextBuffer, SMALL_BUFFER_SIZE);
  UtilSearchTextFile(CONF_FILE_PATH, "VisualizationServerName=", "", TextBuffer);
  snprintf(VisualisationServerName, 128, "%s",TextBuffer);

  
  bzero(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE);
  snprintf(StatusMessage, STATUS_MESSAGE_BUFFER_SIZE,
	   "Origo: lat=%f, long=%f, alt=%f, hdg=%f\nVisualServer %s",
	   OrigoLatitude,
	   OrigoLongitude,
	   OrigoAltitude,
	   OrigoHeading,
	   VisualisationServerName);
  printMessage(StatusMessage);

  OrigoLlh[0] = OrigoLatitude;
  OrigoLlh[1] = OrigoLongitude;
  OrigoLlh[2] = OrigoAltitude;

  /*----------------------------------------------------------------------
    -- One time initialization:
    --
    --    get objects; name and drive file
    ----------------------------------------------------------------------*/

  vFindObjectsInfo ( object_traj_file,
                     object_address_name,
                     &nbr_objects);

  snprintf(StatusMessage, STATUS_MESSAGE_BUFFER_SIZE,
	   "Number of objects: %d", nbr_objects);
  printStatusMessage(StatusMessage);
  bzero(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE);



  /*----------------------------------------------------------------------
    -- Listen loop.
    ----------------------------------------------------------------------*/

  (void) iCommInit ( IPC_RECV_SEND , /* mode */
                     MQ_SV    ,      /* name */
                     1        );     /* blocking */

  int iExit                  = 0 ;
  int iCommand                   ;
  int doInitialAlignmentLoop = 0 ;
  int doCorrelationLoop      = 0 ;

  while ( ! iExit ) {

    /*---------------------------------------------------------------------- 
      -- Connect to visual server 
      ----------------------------------------------------------------------*/
    if (!visual_connected){
      visual_connected = connectVisualServer(&visual_server, &visual_server_addr);
    }

    if (!visual_connected) { 
      snprintf(StatusMessage, STATUS_MESSAGE_BUFFER_SIZE,
	       "Visualisation not connected");
      printStatusMessage(StatusMessage);
      bzero(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE);
    }
   
    /*----------------------------------------------------------------------
      -- **** Initialize ****
      ----------------------------------------------------------------------*/
    if ( Initialize ){

      /*--------------------------------------------------
        -- Init datastructures
	--------------------------------------------------*/

      for ( iIndex = 0           ;
	    iIndex < MAX_OBJECTS ;
	    ++iIndex             )
	{
	  bestFitDone  [ iIndex ] = 0 ;
	  align        [ iIndex ] = 0 ;
	  have_prev_moni_point[iIndex] = 0;
	  last_speed[iIndex] = 0.0;
	  max_speed[iIndex]  = 0.0;

	  traj_done[iIndex] = 0;

	  for ( i = 0;
		i < 3;
		i++ ) {

	    distance[iIndex][i] = 0;
	    bestFit[iIndex][i] = -1;
	  }
	}

      /*--------------------------------------------------
        -- Init Trajectories
	--------------------------------------------------*/

      for (iIndex = 0; iIndex < nbr_objects; iIndex ++){
	/* if traj file not open, open it */
	if (!fp[iIndex]) {

	  fp[iIndex] = fopen(object_traj_file[iIndex],"rb");
	  if (fp[iIndex] == NULL) {
	    util_error("ERR: Failed to open trajectory file");
	  }
	} else {
	  /* rewind trajectory files for restart from beginning */
	  rewind(fp[iIndex]);
	}
      }

      /*--------------------------------------------------
        -- Read initial trajectory file lines (prime the pump)
	--------------------------------------------------*/

      for ( iIndex = 0; iIndex < nbr_objects; ++iIndex)	{

	/* Just read the first line - header line - and ignore */

	len = sizeof(bFileLine [ iIndex ] [0]);
	bzero ( &bFileLine [ iIndex ] [0], len );
	bFileLine_p [ iIndex ] = bFileLine [ iIndex ] [0];
	read = getline ( &bFileLine_p [ iIndex ], &len, fp [ iIndex ]);

	/* Then read the next lines, the real first 'LINE'
	   Put the first line twice into the buffer
	   this will make the first line appear twice in the buffer,
	   first position and center position
	   Then read the second 'LINE' from file and put it at end of buffer,
	   i.e. position 3
	*/

	len = sizeof(bFileLine [ iIndex ] [0]);
	bzero ( &bFileLine [ iIndex ] [0], len);
	bzero ( &bFileLine [ iIndex ] [1], len);
	bzero ( &bFileLine [ iIndex ] [2], len);

	bFileLine_p [ iIndex ] = bFileLine [ iIndex ] [0];
	read = getline ( &bFileLine_p [ iIndex ] ,
			 &len,
			 fp [ iIndex ] );

	strcpy (bFileLine [ iIndex ] [1],
		bFileLine [ iIndex ] [0]);

	bFileLine_p [ iIndex ] = bFileLine [ iIndex ] [2];
	read = getline ( &bFileLine_p [ iIndex ] ,
			 &len ,
			 fp [ iIndex ] );

      } // for ( iIndex = 0
      /*----------------------------------------------------------------------
	-- Parse traj file lines
	----------------------------------------------------------------------*/

      for ( iIndex = 0; iIndex < nbr_objects; ++iIndex) {

	for ( i = 0; i < 3; i++ ) {

	  sscanf ( &bFileLine [ iIndex ] [i] [5],
		   "%f;%lf;%lf;%lf;%f;%f;",
		   &time ,
		   &x    ,
		   &y    ,
		   &z    ,
		   &hdg  ,
		   &vel  );

	  /* new format for comparisons */
	  traj_points[iIndex][i].time = time;
	  traj_points[iIndex][i].x = x;
	  traj_points[iIndex][i].y = y;
	  traj_points[iIndex][i].z = z;
	  traj_points[iIndex][i].hdg = hdg;
	  traj_points[iIndex][i].vel = vel;

	  snprintf (StatusMessage, STATUS_MESSAGE_BUFFER_SIZE,
		    "DBG : SV : (%d, %d) : %f, %lf, %lf, %lf, %f, %f",
		    iIndex,
		    i,
		    traj_points[iIndex][i].time,
		    traj_points[iIndex][i].x,
		    traj_points[iIndex][i].y,
		    traj_points[iIndex][i].z,
		    traj_points[iIndex][i].hdg,
		    traj_points[iIndex][i].vel);
	  printMessage(StatusMessage);
	  bzero(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE);

	}

      } // for ( iIndex = 0

      Initialize = 0;
    }

    /*----------------------------------------------------------------------
      --  Get current time in supervision thread.
      --  Use this to check if timestamps reveived in moni's
      --  are very old (for example).
      ----------------------------------------------------------------------*/

    gettimeofday ( &tv, NULL );
    CurrentTime = tv2ETSI(tv);



    /*----------------------------------------------------------------------
      --  Check age of previously received MON messages.
      --  Abort if too old!
      ----------------------------------------------------------------------*/
    if (Started) {
      uint64_t t = CurrentTime - StartTimestamp;
      float  ts  = (float)t / MS_PER_SEC;

      for (i = 0; i < nbr_objects; i ++){

	if ( have_prev_moni_point[i] && !traj_done[i]) {

	  if (ts - prev_moni_point[i].time > SafetyMarginMonTim) {
	    snprintf(StatusMessage, STATUS_MESSAGE_BUFFER_SIZE,
		     "ABORTING: Last received MON message from Object %d is too old!", i);

	    printStatusMessage(StatusMessage);
	    bzero(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE);

	    (void) iCommSend ( COMM_ABORT, NULL );
	    Started=0;
	    doCorrelationLoop=0;
	    doInitialAlignmentLoop=0;
	  }

	} 
      } // for (i = 0; ...
    } // if (Started)

    /*----------------------------------------------------------------------
      --  Check if we are completely done! 
      ----------------------------------------------------------------------*/
    int done=0;
    for (i = 0; i < nbr_objects; i++) {
      done += traj_done[i];
    }
    if (Started && done == nbr_objects) {
      snprintf(StatusMessage, STATUS_MESSAGE_BUFFER_SIZE,
	       "All objects are done with their trajectories");

      printStatusMessage(StatusMessage);
      bzero(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE);

      Started = 0;
      doCorrelationLoop=0;
      doInitialAlignmentLoop=0;
      /* disarm/arm is needed to recover */
    }

    /*----------------------------------------------------------------------
      --  Receive message
      ----------------------------------------------------------------------*/

    bzero(cpBuffer, MQ_MAX_MESSAGE_LENGTH);
    (void) iCommRecv ( &iCommand,
                       cpBuffer,
<<<<<<< HEAD
                       RECV_MESSAGE_BUFFER, NULL);
=======
                       MQ_MAX_MESSAGE_LENGTH);
>>>>>>> 0b7cb4faa4eebcad7a5244c83897fd7de13278b5


    /*--------------------------------------------------
      --  START
      --------------------------------------------------*/

    if ( iCommand == COMM_STRT )
      {
	uint32_t unknownQuantity = 0;
	uint64_t strtTimestamp = 0;

	sscanf( cpBuffer,
		"%d;%ld;",
		&unknownQuantity,
		&strtTimestamp);

	/* The start message timestamp has the delay already
	   added to it! */

	StartTimestamp = CurrentTime;
	Started = 1; /* We have received the start signal */

	snprintf(StatusMessage, STATUS_MESSAGE_BUFFER_SIZE,
		 "COMM_STRT: timestamp = %lu", StartTimestamp);
	printStatusMessage(StatusMessage);
	bzero(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE);

        doCorrelationLoop = 1;
      }

    /*--------------------------------------------------
      --  ARM
      --------------------------------------------------*/

    if ( iCommand == COMM_ARMD )
      {
	if ( cpBuffer[0] == 1) {
	  snprintf(StatusMessage, STATUS_MESSAGE_BUFFER_SIZE,
		   "COMM_ARMD");
	  printStatusMessage(StatusMessage);
	  bzero(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE);

	  Armed = 1;
	  doInitialAlignmentLoop = 1;
	} else if (cpBuffer[0] == 2) {
	  snprintf(StatusMessage, STATUS_MESSAGE_BUFFER_SIZE,
		   "COMM_DISARM");
	  printStatusMessage(StatusMessage);
	  bzero(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE);

	  /* rerun initialization part of the loop */
	  Initialize = 1;
	  Started = 0;
	  Armed = 0;
	  doCorrelationLoop = 0;
	  doInitialAlignmentLoop = 0;


	} else {
	  snprintf(StatusMessage, STATUS_MESSAGE_BUFFER_SIZE,
		   "FAULTY ARM COMMAND RECEIVED!");
	  printStatusMessage(StatusMessage);
	  bzero(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE);

	}
      }

    /*--------------------------------------------------
      --  MON
      --
      --  Two different execution paths depending on
      --  the value of doInitialAlignmentLoop and
      --  doCorrelationLoop.
      --  First MON message after an ARM message goes into
      --  initialAlignment.
      --  Mon messages after START (and after ARM) goes
      --  into correlation.
      --------------------------------------------------*/

    else if ( iCommand == COMM_MONI )
      {
	uint8_t unknown_quantity;
        /* Start parsing messages */
	sscanf ( cpBuffer,
                 "%" SCNu8 ";%" SCNu8 ";%" SCNu64 ";%" SCNd32 ";%" SCNd32 ";%" SCNd32 ";%" SCNu16 ";%" SCNu16 ";%" SCNu8 ";",

		 &iIndex,
		 &unknown_quantity, /* always zero? */
                 &timestamp,
                 &latitude,
                 &longitude,
                 &altitude,
                 &speed,
                 &heading,
                 &drivedirection );

	if (Started) {
	  snprintf(StatusMessage, STATUS_MESSAGE_BUFFER_SIZE,
		   "MONITOR message:\nid: %d\nuknwn: %d\ntimestamp: %lu\nlat: %d\nlong: %d"
		   ,iIndex,unknown_quantity,timestamp,latitude,longitude);
	  printStatusMessage(StatusMessage);
	  bzero(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE);
	}

	/* ------------------------------------------------------------
	   Convert data from MON message to coordinates on plane.
           Same format as the traj file.
           ------------------------------------------------------------ */
	PosLlh[0] = (double)latitude / POS_FP_MULTIPLIER;
	PosLlh[1] = (double)longitude / POS_FP_MULTIPLIER;
	PosLlh[2] = (double)altitude / ALT_FP_MULTIPLIER;

	llhToEnu(OrigoLlh, PosLlh, PosXyz); 
	
	if (Started) {
	  snprintf(StatusMessage, STATUS_MESSAGE_BUFFER_SIZE,
		   "Position conversion: LLH (%lf, %lf, %lf), XYZ ( %lf, %lf, %lf)",
		   PosLlh[0], PosLlh[1], PosLlh[2],
		   PosXyz[0], PosXyz[1], PosXyz[2]
		   );
	  printStatusMessage(StatusMessage);
	  bzero(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE);
	}

	// DONE: Check the relationship between timestamp and starttimestamp here.
	// TODO: what about  delayed starts... Maybe a problem too.
	if (Started && timestamp > StartTimestamp){
	  moni_point.time = (float)(timestamp - StartTimestamp) / MS_PER_SEC;
	} else {
	  moni_point.time = 0.0; // TODO: double check if this makes sense.
	}
	moni_point.x = PosXyz[0];
	moni_point.y = PosXyz[1];
	moni_point.z = PosXyz[2];

	moni_point.hdg = heading;  /* Does this need conversion? */
	moni_point.vel = speed;

	if (Started){
	  snprintf(StatusMessage, STATUS_MESSAGE_BUFFER_SIZE,
		   "SPEED(id=%d): %f\nMAX SPEED: %f",
		   iIndex,
		   last_speed[iIndex],
		   max_speed[iIndex]
		   );
	  printStatusMessage(StatusMessage);
	  bzero(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE);
	}

	if (have_prev_moni_point[iIndex] &&
	    (moni_point.x != prev_moni_point[iIndex].x ||
	     moni_point.y != prev_moni_point[iIndex].y ||
	     moni_point.time != prev_moni_point[iIndex].time)) {

	  /* calculate speed */
	  double m  = trajPointDistance(moni_point, prev_moni_point[iIndex]);
	  double km = m / 1000.0;
	  float  ts = (moni_point.time - prev_moni_point[iIndex].time);
	  float  th = ts / 3600.0;

	  if (km/th > max_speed[iIndex]) max_speed[iIndex] = km/th;
	  if (km/th > 0.0) last_speed[iIndex] = km/th;

      	  prev_moni_point[iIndex] = moni_point;

	}
	if (Started && !have_prev_moni_point[iIndex]) {
	  prev_moni_point[iIndex] = moni_point;
	  have_prev_moni_point[iIndex] = 1;
	}

        /*------------------------------------------------------------
          -- Find traj point closest to "starting point".
	  -- This matching is done on position only.
          ------------------------------------------------------------*/
        if ( doInitialAlignmentLoop == 1 )
          {
	   snprintf(StatusMessage, STATUS_MESSAGE_BUFFER_SIZE,
		    "Performing initial aligment: %d", iIndex);
	   printStatusMessage(StatusMessage);
	   bzero(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE);

	   while ( bestFitDone [ iIndex ] == 0 )
	     {
	       snprintf(StatusMessage, STATUS_MESSAGE_BUFFER_SIZE,
			  "ENTERING BEST FIT LOOP");
	       printStatusMessage(StatusMessage);
	       bzero(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE);


	       /* Calculate deviation (devPos) per buffer : 0 1 2 */

	       for ( i = 0 ;
		     i < 3 ;
		     i++   )
		 {
		   distance[iIndex][i] =
		     trajPointDistance(moni_point,
				       traj_points[iIndex][i]);

		   snprintf(StatusMessage, STATUS_MESSAGE_BUFFER_SIZE,
			    "DISTANCE: (%d, %d) : %lf", iIndex, i, distance[iIndex][i]);
		   printStatusMessage(StatusMessage);
		   bzero(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE);

		   snprintf(StatusMessage, STATUS_MESSAGE_BUFFER_SIZE,
			    "POINTS: (%d, %d) : (%lf, %lf, %lf) : (%lf, %lf, %lf)",
			    iIndex,
			    i,
			    moni_point.x,
			    moni_point.y,
			    moni_point.z,
			    traj_points[iIndex][i].x,
			    traj_points[iIndex][i].y,
			    traj_points[iIndex][i].z
			    );
		   printStatusMessage(StatusMessage);
		   bzero(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE);

		 } // for ( i

	       if ( ( distance[iIndex][0] >= distance[iIndex][1] ) &&
		    ( distance[iIndex][1] <= distance[iIndex][2] ) )
		 {

		   bestFitDone [ iIndex ] = 1 ;

		 } else if ( distance[iIndex][1] < distance[iIndex][2] )
		 {

		   bestFitDone [ iIndex ] = 1 ;

		 } else
		 {

		   traj_points[iIndex][0] = traj_points[iIndex][1];
		   traj_points[iIndex][1] = traj_points[iIndex][2];

		   bFileLine_p[iIndex] = bFileLine[iIndex][2];
		   len = sizeof(bFileLine[iIndex][2]);
		   bzero ( &bFileLine[iIndex][2], len);

		   read = getline ( &bFileLine_p [ iIndex ] ,
				    &len,
				    fp [ iIndex ] );

		   sscanf ( &bFileLine [ iIndex ] [ 2 ] [ 5 ],
			    "%f;%lf;%lf;%lf;%f;%f;",
			    &time ,
			    &x    ,
			    &y    ,
			    &z    ,
			    &hdg  ,
			    &vel  );

		   traj_points[iIndex][2].time = time;
		   traj_points[iIndex][2].x = x;
		   traj_points[iIndex][2].y = y;
		   traj_points[iIndex][2].z = z;
		   traj_points[iIndex][2].hdg = hdg;
		   traj_points[iIndex][2].vel = vel;

		 } // if ... else ..

	     } // while ( bestFitDone

	   /* Check to see if all nbr_objects has been aligned & correlated */

	   doInitialAlignmentLoop = 0;

	   for ( jIndex = 0           ;
		 jIndex < nbr_objects ;
		 ++jIndex             )
	     {
	       if ( bestFitDone [ jIndex ] == 0 ) {

		 snprintf(StatusMessage, STATUS_MESSAGE_BUFFER_SIZE,
			  "BEST FIT NOT DONE!");
		 printStatusMessage(StatusMessage);
		 bzero(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE);

		 doInitialAlignmentLoop = 1;
	       }
	     }
          } // if ( doInitialAlignmentLoop


        /*------------------------------------------------------------
          -- Supervision loop.           CORRELATION
          ------------------------------------------------------------*/

        else if ( doCorrelationLoop == 1 )
          {
	    snprintf(StatusMessage, STATUS_MESSAGE_BUFFER_SIZE,
		     "Performing correlation");
	    printStatusMessage(StatusMessage);
	    bzero(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE);

            bestFitDone [ iIndex ] = 0 ;

            while ( bestFitDone [ iIndex ] == 0 )
              {
                /* Calculate deviation (devPos) per buffer : 0 1 2 */

                for (i = 0; i < 3; i++) {
		  
		  double dist = trajPointDistance(moni_point,
						  traj_points[iIndex][i]);

		  double t_diff = fabs( traj_points[iIndex][i].time - moni_point.time); 

		  // distance in time and position
		  distance[iIndex][i] = sqrt (dist*dist + t_diff*t_diff);

		  snprintf(StatusMessage, STATUS_MESSAGE_BUFFER_SIZE,
			   "DISTANCE: (%d, %d) : %lf", iIndex, i, distance[iIndex][i]);
		  printStatusMessage(StatusMessage);
		  bzero(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE);

		  snprintf(StatusMessage, STATUS_MESSAGE_BUFFER_SIZE,
			   "POINTS: (%d, %d) : (%lf, %lf, %lf) : (%lf, %lf, %lf)",
			   iIndex,
			   i,
			   moni_point.x,
			   moni_point.y,
			   moni_point.z,
			   traj_points[iIndex][i].x,
			   traj_points[iIndex][i].y,
			   traj_points[iIndex][i].z
			   );
		  printStatusMessage(StatusMessage);
		  bzero(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE);
		  
		} // for ( i
		
                if ( ( distance[iIndex][0] >= distance[iIndex][1] ) &&
                     ( distance[iIndex][1] <= distance[iIndex][2] ) ) {
		  
		  bestFitDone [ iIndex ] = 1 ;
		  
		} else if ( distance[iIndex][1] < distance[iIndex][2] ){
		  
		  bestFitDone [ iIndex ] = 1 ;
		  
		} else {
		  
		  traj_points[iIndex][0] = traj_points[iIndex][1];
		  traj_points[iIndex][1] = traj_points[iIndex][2];
		  
		  
		  bFileLine_p [ iIndex ] = bFileLine [ iIndex ] [ 2 ];
		  len = sizeof(bFileLine [ iIndex ] [ 2 ]);
		  bzero ( &bFileLine [ iIndex ] [ 2 ], len);
		  read = getline ( &bFileLine_p [ iIndex ] ,
				   &len,
				   fp [ iIndex ] );
		  
		  
		  if (strncmp(bFileLine[iIndex][2], "ENDTRAJECTORY",12) == 0){
		    snprintf(StatusMessage, STATUS_MESSAGE_BUFFER_SIZE,
			     "END OF TRAJECTORY REACHED");
		    printStatusMessage(StatusMessage);
		    bzero(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE);
		    
		    /* This object is done with its trajectory */
		    traj_done[iIndex] = 1;
		  }
		  
		  if (!traj_done[iIndex]){
		    int sscanf_res = 0;
		    
		    sscanf_res = sscanf ( &bFileLine[iIndex][2][5],
					  "%f;%lf;%lf;%lf;%f;%f;",
					  &time,
					  &x,
					  &y,
					  &z,
					  &hdg,
					  &vel);
		    
		    if (sscanf_res < 6) {
		      printf ("ERROR READING TRAJ FILE\n");
		      fflush(stdout);
		    }
		    
		    traj_points[iIndex][2].time = time;
		    traj_points[iIndex][2].x = x;
		    traj_points[iIndex][2].y = y;
		    traj_points[iIndex][2].z = z;
		    traj_points[iIndex][2].hdg = hdg;
		    traj_points[iIndex][2].vel = vel;
		    
		  }
		} // if ... else ..
              } // while ( bestFitDone
	    /* Print info about the best fit point */
	    snprintf(StatusMessage, STATUS_MESSAGE_BUFFER_SIZE,
	    	     "BEST FIT: timestamp=%f, x=%lf, y=%lf, z=%lf",
		     traj_points[iIndex][1].time,
	    	     traj_points[iIndex][1].x,
	    	     traj_points[iIndex][1].y,
	    	     traj_points[iIndex][1].z
	    	     );
	    printStatusMessage(StatusMessage);
	    bzero(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE);

    	    /* Now the best fitting values are in buffer 1 */

	    /* Send best fit to visualiser */ 
	    
	    if (visual_connected) {
	      int txSize = 0;
	      /* Send Supervision "Active" point */ 
	      txSize = snprintf(txBuffer,(size_t)TX_BUF_SIZE,"%s;%d;%.8f;%.8f;%.8f;%.8f",
				ACTIVE_POINT_VISUALISATION_MESSAGE,
				iIndex,
				traj_points[iIndex][1].x,
				traj_points[iIndex][1].y,
				traj_points[iIndex][1].z,
				SafetyMarginPos);	      

	      if (!sendto(visual_server, txBuffer,
			  txSize,
			  0,
			  (const struct sockaddr *)&visual_server_addr,
			  sizeof(struct sockaddr_in))){
	      }
	    }
	    
	    
	    /* Calculate deviation  */

	    devTim = fabs(traj_points[iIndex][1].time - moni_point.time);

	    devPos = trajPointDistance(moni_point,
				       traj_points[iIndex][1]);

	    snprintf(StatusMessage, STATUS_MESSAGE_BUFFER_SIZE,
	    	     "TIME DEVIATION: moni_point.time=%f, deviation=%f",
	    	     moni_point.time,
	    	     devTim
	    	     );
	    printStatusMessage(StatusMessage);
	    bzero(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE);

            if ( devPos > SafetyMarginPos ||
                 devTim > SafetyMarginTim  ) {
	      if (SupervisionLevel & CONF_SUPERVISION_FULL) {

		bzero(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE);
		snprintf(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE,
			 "Aborting due to deviation in: %s %s %s\ndevPos %f\tdevTime %f",
			 devPos > SafetyMarginPos ?  "position" : "",
			 devTim > SafetyMarginTim &&
			 devPos > SafetyMarginPos ? "and" : "",
			 devTim > SafetyMarginTim ? "time" : "",
			 devPos, devTim);
		printStatusMessage(StatusMessage);

		/* -----------------------------------------------------------
		   -- SEND ABORT MESSAGE (AND STOP THE CORRELATION LOOP) 
		   ------------------------------------------------------------*/ 
		(void) iCommSend ( COMM_ABORT, NULL );

		Started = 0;
		doCorrelationLoop=0;
		doInitialAlignmentLoop=0;
		
		/* ------------------------------------------------------------ 
		   -- SEND ABORT VISUALISATION MESSAGE 
		   ------------------------------------------------------------ */
		if (visual_connected) {
		  int txSize = 0;
		  /* Send Supervision "Active" point */ 
		  txSize = snprintf(txBuffer,(size_t)TX_BUF_SIZE,"%s;%d;%.8f;%.8f;%.8f",
				    ABORT_VISUALISATION_MESSAGE,
				    iIndex,
				    moni_point.x,
				    moni_point.y,
				    moni_point.z);	      
		  
		  sendto(visual_server, txBuffer,
			 txSize,
			 0,
			 (const struct sockaddr *)&visual_server_addr,
			 sizeof(struct sockaddr_in));
		  
		}

	      }
	      if (SupervisionLevel & CONF_SUPERVISION_PRINT){
		printf("[Supervision] Aborting!\n");
	      }
	    } else {
	      bzero(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE);
	      snprintf(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE, "Ok");
	      printStatusMessage(StatusMessage);
	    } 

	    if ( (devPos > AlertMarginPos ||
		  devTim > AlertMarginTim) &&
		 visual_connected  ) {
	      
	      int txSize = 0;
	      /* Send Supervision "Active" point */ 
	      txSize = snprintf(txBuffer,(size_t)TX_BUF_SIZE,"%s;%d;%.8f;%.8f;%.8f",
				ALERT_VISUALISATION_MESSAGE,
				iIndex,
				moni_point.x,
				moni_point.y,
				moni_point.z);	      
	      
	      sendto(visual_server, txBuffer,
		     txSize,
		     0,
		     (const struct sockaddr *)&visual_server_addr,
		     sizeof(struct sockaddr_in));

	    bzero(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE);
	    snprintf(StatusMessage,STATUS_MESSAGE_BUFFER_SIZE, "SENDING ALERT MESSAGE!!");
	    printStatusMessage(StatusMessage);
	    
	    }
          } // if ( doInitialAlignmentLoop == 1 ) .. else if ( doCorrelationLoop
      } // if ( iCommand == COMM_MONI

    /*--------------------------------------------------
      --  Stop
      --------------------------------------------------*/
    else if (iCommand == COMM_STOP) {

      Started = 0;
      doCorrelationLoop = 0; /* so to not send any abort messages */
    }

    /*--------------------------------------------------
      --  REPLAY
      --------------------------------------------------*/

    else if ( iCommand == COMM_REPLAY ) {
      /* printf ( "INF : SV : Received REPLAY message: %s\n", */
      /*          cpBuffer ); */
    }


    /*--------------------------------------------------
      --  EXIT
      --------------------------------------------------*/

    else if ( iCommand == COMM_EXIT ) {
      iExit = 1;
    }

  } // while(!iExit)


  /*--------------------------------------------------
    --  Clean up.
    --------------------------------------------------*/

 (void) iCommClose();

  for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
    fclose ( fp [ iIndex ] );
  }
}
