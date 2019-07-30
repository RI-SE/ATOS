#include <sys/time.h>
#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <string.h>

#include "util.h"

/*------------------------------------------------------------
  -- Defines
  ------------------------------------------------------------*/
#define RECV_MESSAGE_BUFFER 1024
#define VISUAL_SERVER_NAME  "localhost"
#define VISUAL_SERVER_PORT  53250
#define VISUAL_CONTROL_MODE 0
#define VISUAL_REPLAY_MODE 1

#define SMALL_ITEM_TEXT_BUFFER_SIZE 20
#define MODULE_NAME "VisualizationAdapter"

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/


/*------------------------------------------------------------
-- Private variables
------------------------------------------------------------*/


/*------------------------------------------------------------
-- The main function.
------------------------------------------------------------*/
void visualization_task(TimeType *GPSTime, GSDType *GSD, LOG_LEVEL logLevel)
{
    int camTimeCycle = 0;
    I32 iExit = 0;
    char busReceiveBuffer[MBUS_MAX_DATALEN];               //!< Buffer for receiving from message bus
    enum COMMAND command;

    (void)iCommInit();
    LogInit(MODULE_NAME,LOG_LEVEL_INFO);
    LogMessage(LOG_LEVEL_INFO, "Visualization running with PID: %i", getpid());

    while(!iExit)
    {
        // Handle states specific things
        state = pending_state;

        switch (state) {
        case INIT:
            break;
        case CONNECTED:
            break;
        case SENDING:
            break;
        }

        // Handle MQ messages
        bzero(busReceiveBuffer, sizeof(busReceiveBuffer));
               (void)iCommRecv(&command,busReceiveBuffer, sizeof(busReceiveBuffer), NULL);
               if (command == COMM_ABORT)
               {

               }

               if(command == COMM_EXIT)
               {
                   iExit = 1;
                   printf("Vizualisation exiting.\n");

                   (void)iCommClose();
               }
        //usleep(100000);
               switch (command)
               {
               case COMM_INIT:

                   break;
               case COMM_MONI:
                   // Ignore old style MONR data
                   break;
               case COMM_MONR:

                   break;
               case COMM_OBC_STATE:
                   break;
               case COMM_CONNECT:
                   break;
               case COMM_LOG:
                   break;
               case COMM_INV:
                   break;
               default:
                   LogMessage(LOG_LEVEL_WARNING, "Unhandled message bus command: %u", command);
       }
    }
}


