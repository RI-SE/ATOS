/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  ------------------------------------------------------------------------------
  -- File        : supervision.c
  -- Author      : 
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

#include "util.h"
#include "supervision.h"

/*------------------------------------------------------------
  -- Defines
  ------------------------------------------------------------*/

#define LOCALHOST "127.0.0.1"

#define LDM_SIZE            5
#define RECV_MESSAGE_BUFFER 1024
#define SAFETY_MARGIN_POS   0.5
#define SAFETY_MARGIN_TIM   0.5
#define DEBUG 1

/* 34 years between 1970 and 2004, 8 days for leap year between 1970 and 2004      */
/* Calculation: 34 * 365 * 24 * 3600 * 1000 + 8 * 24 * 3600 * 1000 = 1072915200000 */
#define MS_FROM_1970_TO_2004_NO_LEAP_SECS 1072915200000

/* Difference of leap seconds between UTC and ETSI */
#define DIFF_LEAP_SECONDS_UTC_ETSI 5


/*------------------------------------------------------------
  -- Function declarations & definitions.
  ------------------------------------------------------------*/

/* This code is a straight copy from objectcontrol.c */

static void vFindObjectsInfo ( char object_traj_file    [MAX_OBJECTS][MAX_FILE_PATH],
                               char object_address_name [MAX_OBJECTS][MAX_FILE_PATH],
                               int* nbr_objects )
{
  DIR*           traj_directory;
  struct dirent* directory_entry;
  int            iForceObjectToLocalhost;
  int i = 0;

  iForceObjectToLocalhost = 0;

  traj_directory = opendir(TRAJECTORY_PATH);
  if(traj_directory == NULL)
    {
      util_error("ERR: Failed to open trajectory directory");
    }

  (void)iUtilGetIntParaConfFile("ForceObjectToLocalhost",
                                &iForceObjectToLocalhost);

  printf("Filenames:\n");
  while ((directory_entry = readdir(traj_directory)) &&
         ((*nbr_objects) < MAX_OBJECTS))
    {
      /* Check so it's not . or .. */
      if (strncmp(directory_entry->d_name,".",1))
        {
          bzero(object_address_name[(*nbr_objects)],MAX_FILE_PATH);
          printf("Id %d = %s\n", i++, directory_entry->d_name);
          bzero(object_traj_file[(*nbr_objects)],MAX_FILE_PATH);
          (void)strcat(object_traj_file[(*nbr_objects)],TRAJECTORY_PATH);
          (void)strcat(object_traj_file[(*nbr_objects)],directory_entry->d_name);

          if(0 == iForceObjectToLocalhost)
            {
              (void)strncat(object_address_name[(*nbr_objects)],
                            directory_entry->d_name,
                            strlen(directory_entry->d_name));
            }
          else
            {
              (void)strcat(object_address_name[(*nbr_objects)],LOCALHOST);
            }

          /* #ifdef DEBUG */
          /*       printf ( "DBG : SV : otf = %s , oan = %s \n", */
          /*                object_traj_file    [(*nbr_objects)], */
          /*                object_address_name [(*nbr_objects)] ); */
          /*         ; */
          /*       fflush(stdout); */
          /* #endif */

          ++(*nbr_objects);
        }
    }
  (void)closedir(traj_directory);
}


/*------------------------------------------------------------
  -- Task
  ------------------------------------------------------------*/

void supervision_task() {

  char           object_traj_file    [ MAX_OBJECTS ][ MAX_FILE_PATH ];
  char           object_address_name [ MAX_OBJECTS ][ MAX_FILE_PATH ];
  int            nbr_objects = 0 ;

  FILE*          fp          [ MAX_OBJECTS ]              ;
  char           bFileLine   [ MAX_OBJECTS ] [3] [100]    ;
  char*          bFileLine_p [ MAX_OBJECTS ]              ;
  size_t         len                                      ;
  int            read                                     ;
  int            i                                        ;
  monitor_t      traj                                     ;
  monitor_t      trajs       [ MAX_OBJECTS ] [3]          ;
  ObjectPosition deviation   [ MAX_OBJECTS ] [3]          ;
  double         distance    [ MAX_OBJECTS ] [3]          ;
  int            bestFit     [ MAX_OBJECTS ] [3]          ;
  double         align       [ MAX_OBJECTS ]              ;
  uint64_t       timeX       [ MAX_OBJECTS ]              ;
  uint64_t       time0       [ MAX_OBJECTS ]              ;
  int            bestFitDone [ MAX_OBJECTS ]              ;

  float     time       ;
  double    x          ;
  double    y          ;
  double    z          ;
  float     hdg        ;
  float     vel        ;

  uint16_t  iIndex = 0     ;
  uint16_t  jIndex = 0     ;

  uint64_t  timestamp      ;
  int32_t   latitude       ;
  int32_t   longitude      ;
  int32_t   altitude       ;
  uint16_t  speed          ;
  uint16_t  heading        ;
  uint8_t   drivedirection ;
  monitor_t monitor        ;

  monitor_t ldm          [ MAX_OBJECTS ] [ LDM_SIZE ] ;
  int       ldm_act_step [ MAX_OBJECTS ]              ;
  char      cpBuffer     [ RECV_MESSAGE_BUFFER ]      ;


  struct timespec sleep_time ;
  struct timespec ref_time   ;

  ObjectPosition tObjectPos  ;
  double         dP1Lat  = 0 ;
  double         dP1Long = 0 ;
  double         dP2Lat  = 0 ;
  double         dP2Long = 0 ;
  double         dLat        ;
  double         dLong       ;
  uint64_t       dTime       ;
  uint64_t       dTime0      ;

  char           pcTempBuffer [512];

  double         devPos = 0.0;
  uint64_t       devTim = 0;

  struct timeval tv ;
  uint64_t msSinceEpochETSI ;


//  printf ("--------------------------------------------------\n");
//  printf ("INF : SV : Supervision started.\n");
//  printf ("--------------------------------------------------\n");
//  fflush(stdout);


  /*----------------------------------------------------------------------
    -- Init
    ----------------------------------------------------------------------*/

  tObjectPos.x = 0;
  tObjectPos.y = 0;

  for ( iIndex = 0           ;
        iIndex < MAX_OBJECTS ;
        ++iIndex             )
    {
      ldm_act_step [ iIndex ] = 0 ;

      bestFitDone  [ iIndex ] = 0 ;
      align        [ iIndex ] = 0 ;

      for ( i = 0;
            i < 3;
            i++ ) {

        deviation    [ iIndex ] [i] = tObjectPos ;
        distance     [ iIndex ] [i] = 0          ;
        bestFit      [ iIndex ] [i] = -1         ;
      }
    }


  /*----------------------------------------------------------------------
    -- Open a drive file
    --    get objects; name and drive file
    ----------------------------------------------------------------------*/

  vFindObjectsInfo ( object_traj_file,
                     object_address_name,
                     &nbr_objects);

  //printf ( "DGB : SV : dir nr of files = %d\n", nbr_objects );


  /*----------------------------------------------------------------------
    -- Read the 2 (!) first lines from each trajectory file and put into buffer.
    ----------------------------------------------------------------------*/

  for ( iIndex = 0           ;
        iIndex < nbr_objects ;
        ++iIndex             )
    {

      /* printf ( "INF : SV : Open index = %d file %s \n", */
      /*          iIndex, */
      /*          object_traj_file [ iIndex ] ); */

      fp [ iIndex ]  = fopen ( object_traj_file [ iIndex ], "rb");
      if ( fp [ iIndex ] == NULL )
        {
          util_error("ERR: Failed to open trajectory file");
        }


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
         i.e. position 3 */

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

      for ( i = 0;
            i < 3;
            i++ )
        {
         // printf ( "INF : SV : i = %d , s = %s",
         //          i,
         //          bFileLine [ iIndex ] [i] );
        }

    } // for ( iIndex = 0


  //printf ( "INF : SV : Opened all trajectory files.\n" );


  /*----------------------------------------------------------------------
    -- Convert all lines to internal representation, lat & long
    ----------------------------------------------------------------------*/

  for ( iIndex = 0           ;
        iIndex < nbr_objects ;
        ++iIndex ) {

    for ( i = 0;
          i < 3;
          i++ ) {

      sscanf ( &bFileLine [ iIndex ] [i] [5],
               "%f;%lf;%lf;%lf;%f;%f;",
               &time ,
               &x    ,
               &y    ,
               &z    ,
               &hdg  ,
               &vel  );

      traj2ldm ( time  ,
                 x     ,
                 y     ,
                 z     ,
                 hdg   ,
                 vel   ,
                 &traj );

      trajs [ iIndex ] [ i ].timestamp  = traj.timestamp ;
      trajs [ iIndex ] [ i ].latitude   = traj.latitude  ;
      trajs [ iIndex ] [ i ].longitude  = traj.longitude ;

      //printf ( "DBG : SV : %d %d : t %" PRIu64 ", lat %d, lon %d \n",
      //         iIndex ,
      //         i      ,
      //         trajs [ iIndex ] [ i ].timestamp ,
      //         trajs [ iIndex ] [ i ].latitude  ,
      //         trajs [ iIndex ] [ i ].longitude );
    }

  } // for ( iIndex = 0


  /*----------------------------------------------------------------------
    -- Listen loop.
    ----------------------------------------------------------------------*/

  (void) iCommInit ( IPC_RECV ,
                     MQ_SV    ,
                     0        );

  int iExit                  = 0 ;
  int iCommand                   ;
  int doInitialAlignmentLoop = 0 ;
  int doCorrelationtLoop     = 0 ;

  while ( ! iExit ) {

    bzero(cpBuffer, RECV_MESSAGE_BUFFER);
    (void) iCommRecv ( &iCommand,
                       cpBuffer,
                       RECV_MESSAGE_BUFFER);

#ifdef DEBUG1
    //printf ( "INF : SV : Received a command: %s\n",
    //         cpBuffer );
    //fflush(stdout);
#endif

    /*--------------------------------------------------
      --  TRIG
      --------------------------------------------------*/

    if ( iCommand == COMM_STRT )
      {
        //printf ( "INF : SV : Received a START message: %s\n",
        //         cpBuffer );

        for ( iIndex = 0           ;
              iIndex < nbr_objects ;
              ++iIndex             )
          {
            time0 [ iIndex ] = timeX [ iIndex ] ;
        //    printf ( "INF : SV : index %d  time0 %" PRIu64 ".\n",
        //             iIndex,
        //             time0 [ iIndex ] );
          }

        //printf ( "INF : SV : Start correlation.\n" );
        doCorrelationtLoop = 1;
      }


    /*--------------------------------------------------
      --  ARM
      --------------------------------------------------*/

    if ( iCommand == COMM_ARMD )
      {
        //printf ( "INF : SV : Received an ARM message: %s\n",
        //         cpBuffer );
        //printf ( "INF : SV : Start initial alignment.\n" );
        doInitialAlignmentLoop = 1;
      }


    /*--------------------------------------------------
      --  MON
      --------------------------------------------------*/

    else if ( iCommand == COMM_MONI )
      {
#ifdef DEBUG1
        //printf ( "INF : SV : Recieved MONITOR message: %s\n",
        //         cpBuffer );
        //fflush(stdout);
#endif

        /* Start parsing messages */
        sscanf ( cpBuffer,
                 "MONR;%" SCNu16 ";%" SCNu64 ";%" SCNd32 ";%" SCNd32 ";%" SCNd32 ";%" SCNu16 ";%" SCNu16 ";%" SCNu8 ";",
                 &iIndex,
                 &timestamp,
                 &latitude,
                 &longitude,
                 &altitude,
                 &speed,
                 &heading,
                 &drivedirection );

        monitor.timestamp      = timestamp      ;
        monitor.latitude       = latitude       ;
        monitor.longitude      = longitude      ;
        monitor.altitude       = altitude       ;
        monitor.speed          = speed          ;
        monitor.heading        = heading        ;
        monitor.drivedirection = drivedirection ;

        /*
          #ifdef DEBUG1
          printf ( "DBG : SV : Rec MON : i %d : t %" PRIu64 ", lat %d, lon %d, a %d, s %d, h %d, d %d \n",
          iIndex                 ,
          monitor.timestamp      ,
          monitor.latitude       ,
          monitor.longitude      ,
          monitor.altitude       ,
          monitor.speed          ,
          monitor.heading        ,
          monitor.drivedirection );
          fflush(stdout);
          #endif
        */

        timeX [ iIndex  ] = timestamp;


        /*------------------------------------------------------------
          --
          ------------------------------------------------------------*/

        if ( doInitialAlignmentLoop == 1 )
          {
#ifdef DEBUG
            //printf ( "DBG : SV : Rec MON : i %d : t %" PRIu64 ", lat %d, lon %d, a %d, s %d, h %d, d %d \n",
            //         iIndex                 ,
            //         monitor.timestamp      ,
            //         monitor.latitude       ,
            //         monitor.longitude      ,
            //         monitor.altitude       ,
            //         monitor.speed          ,
            //         monitor.heading        ,
            //         monitor.drivedirection );
            //fflush(stdout);
#endif

            while ( bestFitDone [ iIndex ] == 0 )
              {
                //printf ( "DBG : SV : Align object id  %d \n",
                //         iIndex );

                /* Calculate deviation (devPos) per buffer : 0 1 2 */

                for ( i = 0 ;
                      i < 3 ;
                      i++   )
                  {
                    dP1Lat  = (double) trajs [ iIndex ] [ i ].latitude  / 10000000;
                    dP1Long = (double) trajs [ iIndex ] [ i ].longitude / 10000000;

                    dP2Lat  = (double) latitude  / 10000000;
                    dP2Long = (double) longitude / 10000000;

                    dLat  = dP1Lat  - dP2Lat;
                    dLong = dP1Long - dP2Long;

#ifdef DEBUG
                    //printf ( "DGB : SV : LatLong : iIndex = %d  %d \n",
                    //         iIndex  ,
                    //         i       );
                    //printf ( "  dP1Lat = %f, dP1Long = %f\n",
                    //         dP1Lat  ,
                    //         dP1Long );
                    //printf ( "  dP2Lat = %f, dP2Long = %f\n",
                    //         dP2Lat  ,
                    //         dP2Long );
                    //printf ( "  dLat   = %f,  dLong   = %f\n",
                    //         dLat    ,
                    //         dLong   );
#endif

                    tObjectPos.x = 0;
                    tObjectPos.y = 0;

                    UtilCalcPositionDelta ( dP1Lat  ,
                                            dP1Long ,
                                            dP2Lat  ,
                                            dP2Long ,
                                            &tObjectPos );

                    devPos = sqrt ( tObjectPos.x * tObjectPos.x +
                                    tObjectPos.y * tObjectPos.y );

                    deviation [ iIndex ] [ i ] = tObjectPos ;
                    distance  [ iIndex ] [ i ] = devPos     ;

#ifdef DEBUG
                    //printf ( "  x = %f, y = %f, devPos %f \n",
                    //         tObjectPos.x      ,
                    //         tObjectPos.y      ,
                    //         devPos            );
#endif

                  } // for ( i


                if ( ( distance  [ iIndex ] [ 0 ] >= distance  [ iIndex ] [ 1 ] ) &&
                     ( distance  [ iIndex ] [ 1 ] <= distance  [ iIndex ] [ 2 ] ) )
                  {
                    //printf ( "DBG : SV : Initial align - Best fit on 1 - index %d ok.\n", iIndex );

                    bestFitDone [ iIndex ] = 1 ;
                  } else if ( distance  [ iIndex ] [ 1 ] <  distance  [ iIndex ] [ 2 ] )
                  {
                    //printf ( "DBG : SV : Initial align - Worse on 2 - index %d ok.\n", iIndex );

                    bestFitDone [ iIndex ] = 1 ;
                  } else
                  {
                    //printf ( "DBG : SV : Initial align - Shifting in a new line from trajectory file.\n" );

                    trajs [ iIndex ] [ 0 ] = trajs [ iIndex ] [ 1 ];
                    trajs [ iIndex ] [ 1 ] = trajs [ iIndex ] [ 2 ];

                    bFileLine_p [ iIndex ] = bFileLine [ iIndex ] [ 2 ];
                    len = sizeof(bFileLine [ iIndex ] [ 2 ]);
                    bzero ( &bFileLine [ iIndex ] [ 2 ], len);
                    read = getline ( &bFileLine_p [ iIndex ] ,
                                     &len,
                                     fp [ iIndex ] );
                    //printf ( "INF : SV : i = 2 , s = %s",
                    //         bFileLine [ iIndex ] [ 2 ] );

                    sscanf ( &bFileLine [ iIndex ] [ 2 ] [ 5 ],
                             "%f;%lf;%lf;%lf;%f;%f;",
                             &time ,
                             &x    ,
                             &y    ,
                             &z    ,
                             &hdg  ,
                             &vel  );

                    traj2ldm ( time  ,
                               x     ,
                               y     ,
                               z     ,
                               hdg   ,
                               vel   ,
                               &traj );

                    trajs [ iIndex ] [ 2 ].timestamp  = traj.timestamp ;
                    trajs [ iIndex ] [ 2 ].latitude   = traj.latitude  ;
                    trajs [ iIndex ] [ 2 ].longitude  = traj.longitude ;

                    for ( i = 0;
                          i < 3;
                          i++ )
                      {
                       // printf ( "DBG : SV : trajs : %d %d : t %" PRIu64 ", lat %d, lon %d \n",
                       //          iIndex ,
                       //          i      ,
                       //          trajs [ iIndex ] [ i ].timestamp ,
                       //          trajs [ iIndex ] [ i ].latitude  ,
                       //          trajs [ iIndex ] [ i ].longitude );
                      } // for

                  } // if ... else ..

              } // while ( bestFitDone


            /* Check to see if all nbr_objects has been aligned & correlated */

            doInitialAlignmentLoop = 0;

            for ( jIndex = 0           ;
                  jIndex < nbr_objects ;
                  ++jIndex             )
              {
              //  printf ( "DBG : SV : Initial align - bestFitDone index %d ok = %d.\n",
              //           jIndex,
              //           bestFitDone [ jIndex ] );
                if ( bestFitDone [ jIndex ] == 0 ) doInitialAlignmentLoop = 1;
              }


            /*  Finished aligning all objects, just print status. */

            if ( doInitialAlignmentLoop == 0 )
              {
                gettimeofday ( &tv, NULL );
                msSinceEpochETSI =
                  (uint64_t) tv.tv_sec  * 1000 +
                  (uint64_t) tv.tv_usec / 1000 -
                  MS_FROM_1970_TO_2004_NO_LEAP_SECS +
                  DIFF_LEAP_SECONDS_UTC_ETSI * 1000;

                //printf ( "DBG : SV : Initial align - all index ok.\n" );
                //printf ( "DBG : SV : Initial align - SV  time %" PRIu64 ".\n", msSinceEpochETSI  );
                //printf ( "DBG : SV : Initial align - MON time %" PRIu64 ".\n", monitor.timestamp );

                for ( jIndex = 0           ;
                      jIndex < nbr_objects ;
                      ++jIndex             )
                  {
                  //  printf ( "DBG : SV : Initial align - index %d  1.time = %" PRIu64 " .\n",
                  //           jIndex,
                  //           trajs [ jIndex ] [ 1 ].timestamp );
                  }
              }

          } // if ( doInitialAlignmentLoop


        /*------------------------------------------------------------
          -- Supervision loop.
          ------------------------------------------------------------*/

        else if ( doCorrelationtLoop == 1 )
          {
#ifdef DEBUG1
            //printf ( "INF : SV : Corr id %d - Recieved MONITOR message: %s\n",
            //         iIndex,
            //         cpBuffer );
            //fflush(stdout);
#endif

            /*  not used for the moment .....

            ldm [ iIndex ][ ldm_act_step [ iIndex ] ] = monitor;

#ifdef DEBUG1
            printf ( "DBG : SV : i %d, s %d: t %" PRIu64 ", lat %d, lon %d, a %d, s %d, h %d, d %d \n",
                     iIndex                                                    ,
                     ldm_act_step [ iIndex ]                                   ,
                     ldm [ iIndex ] [ ldm_act_step [ iIndex ] ].timestamp      ,
                     ldm [ iIndex ] [ ldm_act_step [ iIndex ] ].latitude       ,
                     ldm [ iIndex ] [ ldm_act_step [ iIndex ] ].longitude      ,
                     ldm [ iIndex ] [ ldm_act_step [ iIndex ] ].altitude       ,
                     ldm [ iIndex ] [ ldm_act_step [ iIndex ] ].speed          ,
                     ldm [ iIndex ] [ ldm_act_step [ iIndex ] ].heading        ,
                     ldm [ iIndex ] [ ldm_act_step [ iIndex ] ].drivedirection );
            fflush(stdout);
#endif

            ldm_act_step [ iIndex ] = ++ldm_act_step [ iIndex ] % LDM_SIZE;

            not used for the moment ....*/


            gettimeofday ( &tv, NULL );
            msSinceEpochETSI =
              (uint64_t) tv.tv_sec  * 1000 +
              (uint64_t) tv.tv_usec / 1000 -
              MS_FROM_1970_TO_2004_NO_LEAP_SECS +
              DIFF_LEAP_SECONDS_UTC_ETSI * 1000;

            if ( monitor.timestamp >= msSinceEpochETSI )
              {
                dTime  = monitor.timestamp - msSinceEpochETSI ;
              } else
              {
                dTime  = msSinceEpochETSI - monitor.timestamp ;
              }

            dTime0 = monitor.timestamp - time0 [ iIndex ] ;


            bestFitDone [ iIndex ] = 0 ;

            while ( bestFitDone [ iIndex ] == 0 )
              {
                /* Calculate deviation (devPos) per buffer : 0 1 2 */

                for ( i = 0 ;
                      i < 3 ;
                      i++   )
                  {
                    dP1Lat  = (double) trajs [ iIndex ] [ i ].latitude  / 10000000;
                    dP1Long = (double) trajs [ iIndex ] [ i ].longitude / 10000000;

                    dP2Lat  = (double) monitor.latitude  / 10000000;
                    dP2Long = (double) monitor.longitude / 10000000;

                    dLat  = dP1Lat  - dP2Lat;
                    dLong = dP1Long - dP2Long;

                    tObjectPos.x = 0;
                    tObjectPos.y = 0;

                    UtilCalcPositionDelta ( dP1Lat  ,
                                            dP1Long ,
                                            dP2Lat  ,
                                            dP2Long ,
                                            &tObjectPos );

                    devPos = sqrt ( tObjectPos.x * tObjectPos.x +
                                    tObjectPos.y * tObjectPos.y );

                    deviation [ iIndex ] [ i ] = tObjectPos ;
                    distance  [ iIndex ] [ i ] = devPos     ;

#ifdef DEBUG1
                   // printf ( "DGB : SV : LatLong : iIndex = %d  %d \n",
                   //          iIndex  ,
                   //          i       );
                   // printf ( "  dP1Lat = %f, dP1Long = %f\n",
                   //          dP1Lat  ,
                   //          dP1Long );
                   // printf ( "  dP2Lat = %f, dP2Long = %f\n",
                   //          dP2Lat  ,
                   //          dP2Long );
                   // printf ( "  dLat   = %f,  dLong   = %f\n",
                   //          dLat    ,
                   //          dLong   );
                   // printf ( "  dTime  = %" PRIu64 "\n",
                   //          dTime    );
                   // printf ( "  dTime0 = %" PRIu64 "\n",
                   //          dTime0   );
                   // printf ( "  x = %f, y = %f, devPos %f \n",
                   //          tObjectPos.x      ,
                   //          tObjectPos.y      ,
                   //          devPos            );
#endif

                  } // for ( i


                if ( ( distance  [ iIndex ] [ 0 ] >= distance  [ iIndex ] [ 1 ] ) &&
                     ( distance  [ iIndex ] [ 1 ] <= distance  [ iIndex ] [ 2 ] ) )
                  {
                    // printf ( "DBG : SV : Correlation - Best fit on 1 - index %d ok.\n", iIndex );

                    bestFitDone [ iIndex ] = 1 ;

                  } else if ( distance  [ iIndex ] [ 1 ] <  distance  [ iIndex ] [ 2 ] )
                  {
                    //printf ( "DBG : SV : Correlation - Worse on 2 - index %d ok.\n", iIndex );

                    bestFitDone [ iIndex ] = 1 ;

                    // exit (-1);

                  } else
                  {
                    //printf ( "DBG : SV : Correlation - Shifting in a new line from trajectory file.\n" );

                    // exit (-1);

                    trajs [ iIndex ] [ 0 ] = trajs [ iIndex ] [ 1 ];
                    trajs [ iIndex ] [ 1 ] = trajs [ iIndex ] [ 2 ];

                    bFileLine_p [ iIndex ] = bFileLine [ iIndex ] [ 2 ];
                    len = sizeof(bFileLine [ iIndex ] [ 2 ]);
                    bzero ( &bFileLine [ iIndex ] [ 2 ], len);
                    read = getline ( &bFileLine_p [ iIndex ] ,
                                     &len,
                                     fp [ iIndex ] );
                    //printf ( "INF : SV : i = 2 , s = %s",
                    //         bFileLine [ iIndex ] [ 2 ] );

                    sscanf ( &bFileLine [ iIndex ] [ 2 ] [ 5 ],
                             "%f;%lf;%lf;%lf;%f;%f;",
                             &time ,
                             &x    ,
                             &y    ,
                             &z    ,
                             &hdg  ,
                             &vel  );

                    traj2ldm ( time  ,
                               x     ,
                               y     ,
                               z     ,
                               hdg   ,
                               vel   ,
                               &traj );

                    trajs [ iIndex ] [ 2 ].timestamp  = traj.timestamp ;
                    trajs [ iIndex ] [ 2 ].latitude   = traj.latitude  ;
                    trajs [ iIndex ] [ 2 ].longitude  = traj.longitude ;

                    for ( i = 0;
                          i < 3;
                          i++ )
                      {
                        //printf ( "DBG : SV : trajs : %d %d : t %" PRIu64 ", lat %d, lon %d \n",
                        //         iIndex ,
                        //         i      ,
                        //         trajs [ iIndex ] [ i ].timestamp ,
                        //         trajs [ iIndex ] [ i ].latitude  ,
                        //         trajs [ iIndex ] [ i ].longitude );
                      } // for

                  } // if ... else ..

              } // while ( bestFitDone


            devPos = distance  [ iIndex ] [ 1 ] ;
            devTim = trajs     [ iIndex ] [ 1 ].timestamp * 1000 ;

            if ( dTime0 >= devTim )
              {
                devTim = dTime0 - devTim ;
              } else
              {
                devTim = devTim - dTime0 ;
              }

#ifdef DEBUG1
            //printf ( "DGB : SV : Correlation Dev : iIndex = %d   devPos %f  devTim %" PRIu64 "\n",
            //         iIndex ,
            //         devPos ,
            //         devTim );
#endif

            if ( devPos > SAFETY_MARGIN_POS ||
                 devTim > SAFETY_MARGIN_TIM   )
              {
                /* printf ("INF : SV : Sending ABORT.\n"); */
                /* fflush(stdout); */
                // (void) iCommSend ( COMM_ABORT, NULL );
              }

          } // if ( doInitialAlignmentLoop == 1 ) .. else if ( doCorrelationtLoop

      } // if ( iCommand == COMM_MONI


    /*--------------------------------------------------
      --  REPLAY
      --------------------------------------------------*/

    else if ( iCommand == COMM_REPLAY )
      {

        /* printf ( "INF : SV : Received REPLAY message: %s\n", */
        /*          cpBuffer ); */

      }


    /*--------------------------------------------------
      --  EXIT
      --------------------------------------------------*/

    else if ( iCommand == COMM_EXIT )
      {

        //printf ( "INF : SV : Received EXIT message: %s\n",
        //         cpBuffer );

        iExit = 1;

      }


    /*--------------------------------------------------
      --  ??
      --------------------------------------------------*/

    else
      {
        //printf ( "INF : SV : Unhandled command.\n" );
        //printf ( "INF : SV : Received  command: %s\n",
        //         cpBuffer );
        fflush(stdout);
      }

  } // while(!iExit)


  /*--------------------------------------------------
    --  Clean upp.
    --------------------------------------------------*/

  (void) iCommClose();

  for ( iIndex = 0           ;
        iIndex < nbr_objects ;
        ++iIndex ) {

    fclose ( fp [ iIndex ] );
  }

}
