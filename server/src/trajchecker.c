/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  ------------------------------------------------------------------------------
  -- File        : trajchecker.c
  -- Author      : CHRONOS
  -- Description : CHRONOS
  -- Purpose     : checking of trajectory file suitability
  --             : for supervision purposes
  -- Reference   :
  ------------------------------------------------------------------------------*/

/*------------------------------------------------------------
  -- Include files.
  ------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <strings.h>
#include <string.h>

#include "util.h"

/*------------------------------------------------------------
  -- Defines.
  ------------------------------------------------------------*/

#define LINE_BUFFER_SIZE 8192

#define SPEED_LIMIT 80

#define SEC_PER_HOUR 3600
#define POS_FP_MULTIPLIER 10000000

#define DEBUG 0

/*------------------------------------------------------------
  -- data structures. 
  ------------------------------------------------------------*/

typedef struct {
  double x;
  double y;
  double z;
} xyz_t; 



/*------------------------------------------------------------
  -- Function definitions.
  ------------------------------------------------------------*/

double xyzDistanceMetres(xyz_t p1, xyz_t p2){

  xyz_t t;
  t.x = p1.x - p2.x;
  t.y = p1.y - p2.y;
  t.z = p1.z - p2.z;
  
  return( sqrt( t.x * t.x + t.y * t.y + t.z * t.z));  
  
}

/*------------------------------------------------------------
  -- Main.
  ------------------------------------------------------------*/

int main(int argc, char **argv){

  FILE *fp = NULL;

  char *line_buffer = NULL;
  int done = 0;

  size_t len;


  float time;
  double x;
  double y;
  double z;
  float hdg;
  float vel;

  /* hold 2 consecutive trajectory points */ 
  //  monitor_t traj_point[2];
  //monitor_t tmp_traj;

  xyz_t xyz_point[2];
  float  timestamps[2]; 
  
  
  /* statistics collection*/ 
  unsigned int index = 0;

  unsigned int numBrokenSpeedLimit = 0;
  unsigned int numCoincindentPoints = 0;
  unsigned int numTimeTravel = 0;
  unsigned int numInstantTransport = 0;
  float highestSpeedRequired = 0.0; 

  if (argc != 2) {
    fprintf(stderr,"Wrong number of arguments: Provide a trajectory file.\n"); 
    exit(0);
  }


  fp = fopen(argv[1],"r");


  line_buffer = malloc(LINE_BUFFER_SIZE);
  
  
  printf("Reading trajectory file: %s\n", argv[1]);

  /* skip header line */
  len = LINE_BUFFER_SIZE;// strlen(line_buffer); 
  bzero(line_buffer, LINE_BUFFER_SIZE);
  getline(&line_buffer, &len, fp);
  
  if (feof(fp)) {
    fprintf(stderr,"ERROR: No data in traj file\n");
    exit(0); 
  }

  /* --------------------------------------------------
   *  read first traj point into traj_point[0]
   *  TODO: add feof check
   * -------------------------------------------------- */
  len = strlen(line_buffer); 
  bzero(line_buffer, len);
  getline(&line_buffer, &len, fp);
  
  sscanf ( &line_buffer[5],
	   "%f;%lf;%lf;%lf;%f;%f;",
	   &time ,
	   &x    ,
	   &y    ,
	   &z    ,
	   &hdg  ,
	   &vel  );

  xyz_point[0].x = x;
  xyz_point[0].y = y;
  xyz_point[0].z = z;
  timestamps[0] = time; 
  

  /* --------------------------------------------------
   *  read second traj point into traj_point[1] 
   * -------------------------------------------------- */
  len = strlen(line_buffer); 
  bzero(line_buffer, len);
  getline(&line_buffer, &len, fp);

  sscanf ( &line_buffer[5],
	   "%f;%lf;%lf;%lf;%f;%f;",
	   &time ,
	   &x    ,
	   &y    ,
	   &z    ,
	   &hdg  ,
	   &vel  );

  xyz_point[1].x = x;
  xyz_point[1].y = y;
  xyz_point[1].z = z;
  timestamps[1] = time;
 
  do {

    /* ---------------------------------------------------- 
     * Perform validity checks on the 2 avaialable 
     * consecutive points. 
     * ---------------------------------------------------- */
    
    /* Check if points are identical in space*/
    int coincides_xyz =
      xyz_point[0].x == xyz_point[1].x &&
      xyz_point[0].y == xyz_point[1].y &&
      xyz_point[0].z == xyz_point[1].z;
    
    if (coincides_xyz ) {
      if (numCoincindentPoints < 10)
	printf("FAULT: Indices (%d,%d) consecutive points identical (x,y,z)\n", index, index+1);
      numCoincindentPoints++;
    }
    
    double d_xyz = xyzDistanceMetres(xyz_point[0], xyz_point[1]);     
    
    
    float dt = timestamps[1] - timestamps[0];
    if ( dt < 0) {
      if (numTimeTravel < 10)
	printf("FAULT: Indices (%d,%d) Timetraveling required to follow trajectory\n", index, index +1);
      numTimeTravel++;
    }
    

    if ( dt == 0 & d_xyz > 0) {
      if (numInstantTransport < 10)
	printf("FAULT: Indices (%d,%d) instant matter transportation required to follow trajectory\n", index, index+1);
      numInstantTransport++;
    }
   
    if (dt > 0 & d_xyz > 0) {

      double th = (double)dt / (SEC_PER_HOUR);

      double km_xyz = d_xyz / 1000.0; /* most likely metres */

      double kmh = km_xyz/th; 
      if (kmh > highestSpeedRequired) highestSpeedRequired = kmh;

      if (kmh > SPEED_LIMIT) {
	if (numBrokenSpeedLimit < 10)
	  printf("FAULT: Indices (%d,%d)\n       trajectory requires breaking the speed limit(%d) at %f km/h\n", index, index+1,SPEED_LIMIT, kmh);
	numBrokenSpeedLimit++;
      }
      
    } 
       
    /* ---------------------------------------------------- 
     * Either done or prepare for the next pair of points
     * ---------------------------------------------------- */

    index++;
    
    len = strlen(line_buffer); 
    bzero(line_buffer, len);
    getline(&line_buffer, &len, fp);

    /* ---------------------------------------------------- 
     * Check if we reached end of trajectory file
     * ---------------------------------------------------- */
    if (!strncmp(line_buffer, "ENDTRAJECTORY", 12) || feof(fp)){
      done = 1; 
    } else { 
      
      /* ----------------------------------------
       * Read new line
       * ---------------------------------------- */
      
      sscanf ( &line_buffer[5],
	       "%f;%lf;%lf;%lf;%f;%f;",
	       &time ,
	       &x    ,
	       &y    ,
	       &z    ,
	       &hdg  ,
	       &vel  );
      
      /* ----------------------------------------
       * Shift in 
       * ---------------------------------------- */
      xyz_point[0] = xyz_point[1];
      timestamps[0] = timestamps[1];
      
      /* ----------------------------------------
       * Update traj_point[1]
       * ---------------------------------------- */
      xyz_point[1].x = x;
      xyz_point[1].y = y;
      xyz_point[1].z = z;
      timestamps[1] = time;       
    }
    
  } while(!done);

  printf("Trajectory file violation statistics\n");
  printf("NumBrokenSpeedLimit(%d): %d\nNumCoincindentPoints: %d\nNumTimeTravel: %d\nNumInstantTransport: %d\nHighestSpeedRequired: %f\n",
	 SPEED_LIMIT,
	 numBrokenSpeedLimit,
	 numCoincindentPoints,
	 numTimeTravel,
	 numInstantTransport,
	 highestSpeedRequired);
  
  
  free(line_buffer); 

  return 0; 
}
