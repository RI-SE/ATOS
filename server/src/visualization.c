/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  ------------------------------------------------------------------------------
  -- File        : visualization.c
  -- Author      : Karl-Johan Ode
  -- Description : CHRONOS
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

/*------------------------------------------------------------
  -- Include files.
  ------------------------------------------------------------*/
#include <errno.h>
#include <mqueue.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>

#include "util.h"

/*------------------------------------------------------------
  -- Defines
  ------------------------------------------------------------*/
#define RECV_MESSAGE_BUFFER 1024
#define VISUAL_SERVER_NAME  "localhost"
#define VISUAL_SERVER_PORT  53250
#define VISUAL_CONTROL_MODE 0
#define VISUAL_REPLAY_MODE 1


#define SIM_CONTROL_BUFFER_SIZE_20 20
#define SIM_CONTROL_BUFFER_SIZE_128 128

#define SMALL_ITEM_TEXT_BUFFER_SIZE 20
/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
static void vConnectVisualizationChannel(int* sockfd, struct sockaddr_in* addr);
static void vDisconnectVisualizationChannel(int* sockfd);
static void vSendVisualization(int* sockfd,
                               struct sockaddr_in* addr,
                               const char* message);

void vISOtoCHRONOSmsg(char* ISOmsg,char* tarCHRONOSmsg,int MSG_size);
U32 SimulatorControlBuildObjectMonitorMessage(C8* MessageBuffer, C8 *MONRData, ObjectMonitorType *ObjectMonitorData, U8 debug);

/*------------------------------------------------------------
-- Private variables
------------------------------------------------------------*/

/*------------------------------------------------------------
  -- Public functions
  ------------------------------------------------------------*/
int main(int argc, char *argv[])
{
    C8 SendBuffer[SIM_CONTROL_BUFFER_SIZE_128];
    int visual_server;
    struct sockaddr_in visual_server_addr;
    char cpBuffer[RECV_MESSAGE_BUFFER];
    char chronosbuff[RECV_MESSAGE_BUFFER];
    ObjectMonitorType ObjectMonitorData;
    U32 LengthU32;
    printf("[Visualization] DefaultVisualizationAdapter started\n");
    fflush(stdout);

    (void)iCommInit(IPC_RECV,MQ_VA,0);

    vConnectVisualizationChannel(&visual_server,&visual_server_addr);

    /* Listen for commands */
    int iExit = 0;
    int iCommand;

    /* Execution mode*/
    int VisualExecutionMode = VISUAL_CONTROL_MODE;

    while(!iExit)
    {
        bzero(cpBuffer,RECV_MESSAGE_BUFFER);
        (void)iCommRecv(&iCommand,cpBuffer,RECV_MESSAGE_BUFFER, NULL);

        //if(iCommand == COMM_MONI_BIN)
        //{
        //    //send as chronos if i dont missremeber!!
        //
        //
        //    DEBUG_LPRINT(DEBUG_LEVEL_LOW,"INF: Recieved MONITOR message: %s\n",cpBuffer);
        //
        //
        //}

        if(iCommand == COMM_MONI)
        {
            DEBUG_LPRINT(DEBUG_LEVEL_LOW,"INF: Recieved MONITOR message: %s\n",cpBuffer);

            // Hwere we send as ASCII
            //Option 1
            //vISOtoCHRONOSmsg(cpBuffer,chronosbuff,RECV_MESSAGE_BUFFER);
            //printf("MQ monr %s\n", chronosbuff);
            //printf("Visual_server valueb %d\n", visual_server );

            //vSendVisualization(&visual_server,&visual_server_addr,chronosbuff);

            //Option 2
            LengthU32 = SimulatorControlBuildObjectMonitorMessage(SendBuffer, cpBuffer, &ObjectMonitorData, 1);



            UtilSendUDPData("Visualization", &visual_server, &visual_server_addr, SendBuffer, LengthU32, 0);

            //Option3

            //C8 data[strlen(cpBuffer)];
            //bzero(data, strlen(data));
            //strcat(data, cpBuffer);
            //UtilSendUDPData("Visualization", &visual_server, &visual_server_addr, data, sizeof(data), 0);

        }
        else if(iCommand == COMM_REPLAY)
        {
            VisualExecutionMode = VISUAL_REPLAY_MODE;
            DEBUG_LPRINT(DEBUG_LEVEL_MEDIUM,"Visualization in REPLAY mode: %s\n",cpBuffer);
        }
        else if(iCommand == COMM_CONTROL)
        {
            VisualExecutionMode = VISUAL_CONTROL_MODE;
            DEBUG_LPRINT(DEBUG_LEVEL_MEDIUM,"Visualization in CONTROL mode: %s\n", cpBuffer);
        }
        else if(iCommand == COMM_EXIT)
        {
            iExit = 1;
        }
        else if (iCommand == COMM_OBC_STATE) {

        }
        else
        {
            DEBUG_LPRINT(DEBUG_LEVEL_LOW,"Vizualization unhandled command: %d",iCommand);
        }
    }

    /* Close visualization socket */
    //vDisconnectVisualizationChannel(&visual_server);

    (void)iCommClose();
}

/*------------------------------------------------------------
  -- Private functions
  ------------------------------------------------------------*/
static void vConnectVisualizationChannel(int* sockfd, struct sockaddr_in* addr)
{
    struct hostent *server;
    char pcTempBuffer[MAX_UTIL_VARIBLE_SIZE];

    /* Setup connection to visualization */
    DEBUG_LPRINT(DEBUG_LEVEL_LOW,"%s","INF: Creating visualization socket.\n");

    *sockfd = socket ( AF_INET,
                       SOCK_DGRAM,
                       IPPROTO_UDP );

    if (*sockfd < 0)
    {
        util_error("ERR: Failed to create visualization socket");
    }

    bzero((char *)addr, sizeof(*addr));

    bzero(pcTempBuffer,MAX_UTIL_VARIBLE_SIZE);
    if(!iUtilGetParaConfFile("VisualizationServerName",pcTempBuffer))
    {
        strcat(pcTempBuffer,VISUAL_SERVER_NAME);
    }

    DEBUG_LPRINT(DEBUG_LEVEL_LOW,"[Visualization] UDP visualization sending to %s %d\n",pcTempBuffer,VISUAL_SERVER_PORT);


    server = gethostbyname(pcTempBuffer);

    if (server == NULL)
    {
        util_error("ERR: Unkonown host\n");
    }
    bcopy((char *) server->h_addr,
          (char *)&addr->sin_addr.s_addr, server->h_length);

    addr->sin_family = AF_INET;
    addr->sin_port   = htons(VISUAL_SERVER_PORT);
}

static void vDisconnectVisualizationChannel(int* sockfd)
{
    close(*sockfd);
}

static void vSendVisualization(int* sockfd, struct sockaddr_in* addr,const char* message)
{
    char buffer[1024];
    int result;

    DEBUG_LPRINT(DEBUG_LEVEL_MEDIUM,"INF: Buffer to visualization: <%s>\n",message);

    result = sendto(*sockfd,
                    message,
                    strlen (message),
                    0,
                    (const struct sockaddr *) addr,
                    sizeof(struct sockaddr_in));

    if (result < 0)
    {
        util_error("ERR: Failed to send on monitor socket");
    }
}

void vISOtoCHRONOSmsg(char* ISOmsg,char* tarCHRONOSmsg, int MSG_size)
{
    char IP_address[SMALL_ITEM_TEXT_BUFFER_SIZE];
    char ID[SMALL_ITEM_TEXT_BUFFER_SIZE];
    char Timestamp[SMALL_ITEM_TEXT_BUFFER_SIZE];
    char Latitude[SMALL_ITEM_TEXT_BUFFER_SIZE];
    char Longitude[SMALL_ITEM_TEXT_BUFFER_SIZE];
    char Altitude[SMALL_ITEM_TEXT_BUFFER_SIZE];
    char Heading[SMALL_ITEM_TEXT_BUFFER_SIZE];
    char LonSpeed[SMALL_ITEM_TEXT_BUFFER_SIZE];
    char LatSpeed[SMALL_ITEM_TEXT_BUFFER_SIZE];
    char LonAcc[SMALL_ITEM_TEXT_BUFFER_SIZE];
    char LatAcc[SMALL_ITEM_TEXT_BUFFER_SIZE];
    char DriveDirection[SMALL_ITEM_TEXT_BUFFER_SIZE];
    char StateFlag[SMALL_ITEM_TEXT_BUFFER_SIZE];
    char StatusFlag[SMALL_ITEM_TEXT_BUFFER_SIZE];

    bzero(tarCHRONOSmsg,MSG_size);

    char* item_p = strtok(ISOmsg,";");
    int item_num = 0; // The placement incoming message
    while(item_p != NULL)
    {
        // What to do with each item at the current placement
        switch (item_num) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 11:
        case 12:
            strcat(tarCHRONOSmsg,item_p); strcat(tarCHRONOSmsg,";");
            break;
        case 6:
            bzero(Heading,SMALL_ITEM_TEXT_BUFFER_SIZE);
            int mod_heading = atoi(item_p)/10;
            sprintf(Heading,"%d;",mod_heading);
            break;
        case 7:
            strcat(tarCHRONOSmsg,item_p); strcat(tarCHRONOSmsg,";");
            strcat(tarCHRONOSmsg,Heading);
            break;
        default:
            break;
        }
        item_num++;
        item_p = strtok(NULL,";");
    }

}
U32 SimulatorControlBuildObjectMonitorMessage(C8* MessageBuffer, C8 *MONRData, ObjectMonitorType *ObjectMonitorData, U8 debug)
{

  C8 *ptr;
  C8 TextBuffer[SIM_CONTROL_BUFFER_SIZE_20];
  U32 i,j;

  /*10.130.22.8;0;0;-6;14;229;256;800;0;0;0;0;4;0; */
  //printf("%s\n", MONRData);

  ptr = MONRData;
  /*Get IP*/
  bzero(TextBuffer, SIM_CONTROL_BUFFER_SIZE_20);
  strncpy(TextBuffer, ptr, (uint64_t)strchr(ptr, ';') - (uint64_t)ptr);
  ObjectMonitorData->ObjectIPU32 = SwapU32(UtilIPStringToInt(TextBuffer));

  /*Get GPSSOW*/
  ptr = strchr(ptr+1, ';');
  ptr = strchr(ptr+1, ';');
  bzero(TextBuffer, SIM_CONTROL_BUFFER_SIZE_20);
  strncpy(TextBuffer, ptr+1, (uint64_t)strchr(ptr+1, ';') - (uint64_t)ptr);
  ObjectMonitorData->GPSSOWU32 = SwapU32((U32) atoi(TextBuffer));

  //Get XPosition
  ptr = strchr(ptr+1, ';');
  bzero(TextBuffer, SIM_CONTROL_BUFFER_SIZE_20);
  strncpy(TextBuffer, ptr+1, (uint64_t)strchr(ptr+1, ';') - (uint64_t)ptr);
  ObjectMonitorData->XPositionI32 = SwapI32((I32) atoi(TextBuffer));

  //Get YPosition
  ptr = strchr(ptr+1, ';');
  bzero(TextBuffer, SIM_CONTROL_BUFFER_SIZE_20);
  strncpy(TextBuffer, ptr+1, (uint64_t)strchr(ptr+1, ';') - (uint64_t)ptr);
  ObjectMonitorData->YPositionI32 = SwapI32((I32) atoi(TextBuffer));

  //Get ZPosition
  ptr = strchr(ptr+1, ';');
  bzero(TextBuffer, SIM_CONTROL_BUFFER_SIZE_20);
  strncpy(TextBuffer, ptr+1, (uint64_t)strchr(ptr+1, ';') - (uint64_t)ptr);
  ObjectMonitorData->ZPositionI32 = SwapI32((I32) atoi(TextBuffer));

  //Get Heading
  ptr = strchr(ptr+1, ';');
  bzero(TextBuffer, SIM_CONTROL_BUFFER_SIZE_20);
  strncpy(TextBuffer, ptr+1, (uint64_t)strchr(ptr+1, ';') - (uint64_t)ptr);
  ObjectMonitorData->HeadingU16 = SwapU16((U16) atoi(TextBuffer));

  //Get Speed
  ptr = strchr(ptr+1, ';');
  bzero(TextBuffer, SIM_CONTROL_BUFFER_SIZE_20);
  strncpy(TextBuffer, ptr+1, (uint64_t)strchr(ptr+1, ';') - (uint64_t)ptr);
  ObjectMonitorData->SpeedI16 = SwapI16((I16) atoi(TextBuffer));

  //Set MessageId
  ObjectMonitorData->MessageIdU16 = SwapU16(2);

  ptr=(char *)ObjectMonitorData;
  for(i=0; i<sizeof(ObjectMonitorType); i++) *(MessageBuffer + i + 4) = *ptr++;

  *(MessageBuffer + 0) = (U8)(i >> 24);
  *(MessageBuffer + 1) = (U8)(i >> 16);
  *(MessageBuffer + 2) = (U8)(i >> 8);
  *(MessageBuffer + 3) = (U8)(i);

  if(debug)
  {
    printf("----MONR TO SIMULATOR----\n");
    printf("Heading=%d, Speed=%d, Time=%d, X=%d, Y=%d\n", SwapU16(ObjectMonitorData->HeadingU16), SwapI16(ObjectMonitorData->SpeedI16), SwapU32(ObjectMonitorData->GPSSOWU32), SwapI32(ObjectMonitorData->XPositionI32), SwapI32(ObjectMonitorData->YPositionI32));
    for(j = 0;j < sizeof(ObjectMonitorType) + 4; j ++) printf("%x ", (unsigned char)MessageBuffer[j]);
    printf("\n");

  }


  return i + 4;
}
