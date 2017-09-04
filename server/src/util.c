/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  ------------------------------------------------------------------------------
  -- File        : util.c
  -- Author      : Karl-Johan Ode, Sebastian Loh Lindholm
  -- Description : CHRONOS
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

/*------------------------------------------------------------
  -- Include files.
  ------------------------------------------------------------*/

#include <stdio.h>
#include <math.h>
#include <errno.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "util.h"

/*------------------------------------------------------------
  -- Defines
  ------------------------------------------------------------*/
#define MQ_NBR_QUEUES 4


/*------------------------------------------------------------
-- Private variables
------------------------------------------------------------*/
static mqd_t tMQRecv;
static mqd_t ptMQSend[MQ_NBR_QUEUES];
static char pcMessageQueueName[1024];

/*---------------------------------------------s---------------
  -- Public functions
  ------------------------------------------------------------*/
void util_error(char* message)
{
  perror(message);
  exit(EXIT_FAILURE);
}


int UtilAddEightBytesMessageData(unsigned char *MessageBuffer, int StartIndex, unsigned long Data)
{
  
  int i;
  for(i = 0; i < 8; i++)
  {
    *(MessageBuffer+StartIndex+i) = (char) (Data >> (7-i)*8);
    //printf("[%d]=%x\n", (StartIndex+i), *(MessageBuffer+StartIndex+i));
  }

  return (StartIndex+i);
}

int UtilAddSixBytesMessageData(unsigned char *MessageBuffer, int StartIndex, unsigned long Data)
{
  
  int i;
  for(i = 0; i < 6; i++)
  {
    *(MessageBuffer+StartIndex+i) = (char) (Data >> (5-i)*8);
    //printf("[%d]=%x\n", (StartIndex+i), *(MessageBuffer+StartIndex+i));
  }

  return (StartIndex+i);
}


int UtilAddFourBytesMessageData(unsigned char *MessageBuffer, int StartIndex, unsigned int Data)
{
  int i;
  for(i = 0; i < 4; i++)
  {
    *(MessageBuffer+StartIndex+i) = (unsigned char) (Data >> (3-i)*8);
    //printf("[%d]=%x\n", (StartIndex+i), *(MessageBuffer+StartIndex+i));
  }

  return StartIndex+i;
}

int UtilAddTwoBytesMessageData(unsigned char *MessageBuffer, int StartIndex, unsigned short Data)
{
  int i;
  for(i = 0; i < 2; i++)
  {
    *(MessageBuffer+StartIndex+i) = (unsigned char) (Data >> (1-i)*8);
    //printf("[%d]=%x\n", (StartIndex+i), *(MessageBuffer+StartIndex+i));
  }

  return StartIndex+i;
}

int UtilAddOneByteMessageData(unsigned char *MessageBuffer, int StartIndex, unsigned char Data)
{

  *(MessageBuffer+StartIndex) = Data;
  //printf("[%d]=%x\n", (StartIndex), *(MessageBuffer+StartIndex));
  return StartIndex+1;
}


int UtilAddNBytesMessageData(unsigned char *MessageBuffer, int StartIndex, int Length, unsigned char *Data)
{
  int i;
  for(i = 0; i < Length; i++)
  {
    *(MessageBuffer+StartIndex+i) = *(Data+i);
    //printf("[%d]=%x\n", (StartIndex+i), *(MessageBuffer+StartIndex+i));
  }

  return StartIndex+i;
}


int iUtilGetParaConfFile(char* pcParameter, char* pcValue) 
{
  FILE *filefd;
  int iFindResult;
  char pcTemp[512];

  iFindResult = 0;

  filefd = fopen (TEST_CONF_FILE, "rb");

  if (filefd == NULL)
  {
    return 0;
  }
  
  while(fgets(pcTemp, 512, filefd) != NULL)
  {
    if((strstr(pcTemp, pcParameter)) != NULL) 
    {
      
      /* Does contain any value? */
      if(strlen(pcTemp) > (strlen(pcParameter)+1))
      {
        /* replace new line */
        if(pcTemp[strlen(pcTemp)-1] == '\n')
        {
          pcTemp[strlen(pcTemp)-1] = 0;
        }
        strcpy(pcValue,&pcTemp[strlen(pcParameter)+1]);
      }
      iFindResult = 1;
    }
  }
  
  if(filefd) 
  {
    fclose(filefd);
  }

  return 1;
}


int UtilSetAdaptiveSyncPoint(AdaptiveSyncPoint *ASP, FILE *filefd, char debug)
{
  
  char DataBuffer[MAX_FILE_PATH];
  char RowBuffer[MAX_FILE_PATH];
  char *ptr, *ptr1;
  bzero(DataBuffer,MAX_FILE_PATH);
  bzero(RowBuffer,MAX_FILE_PATH);
  
  bzero(RowBuffer, MAX_FILE_PATH);
  UtilReadLineCntSpecChars(filefd, RowBuffer);
  ptr = strchr(RowBuffer, ';');
  strncpy(DataBuffer, RowBuffer, (uint64_t)ptr - (uint64_t)RowBuffer);
  strncpy(ASP->MasterIP, DataBuffer, strlen(DataBuffer));
  //if(USE_TEST_HOST == 1) strncpy(ASP->MasterIP, TESTHOST_IP, sizeof(TESTHOST_IP)); 

  bzero(DataBuffer, strlen(DataBuffer));
  ptr1 = ptr+1;
  ptr = strchr(ptr+2, ';');
  strncpy(DataBuffer, ptr1, (uint64_t)ptr - (uint64_t)ptr1);  
  strncpy(ASP->SlaveIP, DataBuffer, strlen(DataBuffer));
  //if(USE_TEST_HOST == 1) strncpy(ASP->SlaveIP, TESTHOST_IP, sizeof(TESTHOST_IP)); 

  bzero(DataBuffer, strlen(DataBuffer));
  ptr1 = ptr+1;
  ptr = strchr(ptr+2, ';');
  strncpy(DataBuffer, ptr1, (uint64_t)ptr - (uint64_t)ptr1);  
  ASP->MasterTrajSyncTime = atof(DataBuffer);

  bzero(DataBuffer, strlen(DataBuffer));
  ptr1 = ptr+1;
  ptr = strchr(ptr+2, ';');
  strncpy(DataBuffer, ptr1, (uint64_t)ptr - (uint64_t)ptr1);  
  ASP->SlaveTrajSyncTime = atof(DataBuffer);

  bzero(DataBuffer, strlen(DataBuffer));
  ptr1 = ptr+1;
  ptr = strchr(ptr+2, ';');
  strncpy(DataBuffer, ptr1, (uint64_t)ptr - (uint64_t)ptr1);  
  ASP->SlaveSyncStopTime = atof(DataBuffer);

  ASP->TestPort = 0;

  if(debug)
  { 
    printf("MasterIP: %s\n", ASP->MasterIP);
    printf("SlaveIP: %s\n", ASP->SlaveIP);
    printf("MasterTrajSyncTime %3.2f\n", ASP->MasterTrajSyncTime);
    printf("SlaveTrajSyncTime %3.2f\n", ASP->SlaveTrajSyncTime);
    printf("SlaveSyncStopTime %3.2f\n", ASP->SlaveSyncStopTime);
  }
 
  return 0;
}

int UtilSetTriggActions(TriggAction *TAA, FILE *filefd, char debug)
{
  
  char DataBuffer[MAX_FILE_PATH];
  char RowBuffer[MAX_FILE_PATH];
  char *ptr, *ptr1, *ptr2;
  bzero(DataBuffer,MAX_FILE_PATH);
  bzero(RowBuffer,MAX_FILE_PATH);
  
  bzero(RowBuffer, MAX_FILE_PATH);
  UtilReadLineCntSpecChars(filefd, RowBuffer);

  ptr = strchr(RowBuffer, ';');
  strncpy(DataBuffer, RowBuffer, (uint64_t)ptr - (uint64_t)RowBuffer);
  strncpy(TAA->TriggerIP, DataBuffer, strlen(DataBuffer));
 
  bzero(DataBuffer, strlen(DataBuffer));
  ptr1 = ptr+1;
  ptr = strchr(ptr+2, '[');
  strncpy(DataBuffer, ptr1, (uint64_t)ptr - (uint64_t)ptr1);  
  strncpy(TAA->TriggerType, DataBuffer, strlen(DataBuffer));
 
  bzero(DataBuffer, strlen(DataBuffer));
  ptr1 = ptr+1;
  ptr = strchr(ptr+2, ']');
  strncpy(DataBuffer, ptr1, (uint64_t)ptr - (uint64_t)ptr1);  
  strncpy(TAA->TriggerTypeVar, DataBuffer, strlen(DataBuffer));
  ptr = strchr(ptr, ';');

  bzero(DataBuffer, strlen(DataBuffer));
  ptr1 = ptr+1;
  ptr = strchr(ptr+2, '[');
  strncpy(DataBuffer, ptr1, (uint64_t)ptr - (uint64_t)ptr1);  
  strncpy(TAA->ActionType, DataBuffer, strlen(DataBuffer));

  bzero(DataBuffer, strlen(DataBuffer));
  ptr1 = ptr+1;
  ptr = strchr(ptr+2, ']');
  strncpy(DataBuffer, ptr1, (uint64_t)ptr - (uint64_t)ptr1);  
  strncpy(TAA->ActionTypeVar, DataBuffer, strlen(DataBuffer));
  ptr = strchr(ptr, ';');

  bzero(DataBuffer, strlen(DataBuffer));
  ptr1 = ptr+1;
  ptr = strchr(ptr+2, ';');
  strncpy(DataBuffer, ptr1, (uint64_t)ptr - (uint64_t)ptr1);  
  
  if(strstr(DataBuffer,"SEND_START") != NULL) TAA->Action = TAA_SEND_START;
  else if(strstr(DataBuffer,"TEST_SIGNAL") != NULL) TAA->Action = TAA_TEST_SIGNAL;

  if(debug)
  { 
    printf("TriggerIP: %s\n", TAA->TriggerIP);
    printf("TriggerType: %s\n", TAA->TriggerType);
    printf("TriggerTypeVar: %s\n", TAA->TriggerTypeVar);
    printf("ActionType: %s\n", TAA->ActionType);
    printf("ActionTypeVar: %s\n", TAA->ActionTypeVar);
    printf("Action: %d\n", TAA->Action);
  }

  return 0;
}



void UtilSetObjectPositionIP(ObjectPosition *OP, char *IP) { strncpy(OP->IP, IP, strlen(IP));}

int UtilSetMasterObject(ObjectPosition *OP, char *Filename, char debug)
{
  
  FILE *filefd;
  char FilenameBuffer[MAX_FILE_PATH];
  char DataBuffer[20];
  double Time;
  bzero(FilenameBuffer,MAX_FILE_PATH);
  bzero(DataBuffer,20);
  strcat(FilenameBuffer,Filename);
  strcat(FilenameBuffer, MASTER_FILE_EXTENSION);
  filefd = fopen (FilenameBuffer, "r");
   
  if(filefd)
  {     
    UtilReadLineCntSpecChars(filefd, DataBuffer);
    Time = atof(DataBuffer);
    OP->Type = 'm';
    OP->SyncTime = Time;
    OP->SyncStopTime = 0;
    fclose(filefd);

    if(debug) printf("Master object set: %s, SyncTime: %3.4f\n", FilenameBuffer, Time);

  } else {
    OP->Type = 'u';
    OP->SyncTime = 0;
  }    


  return 0;
}


int UtilSetSlaveObject(ObjectPosition *OP, char *Filename, char debug)
{
  FILE *filefd;
  char FilenameBuffer[MAX_FILE_PATH];
  char DataBuffer[20];
  double Time;
  bzero(FilenameBuffer,MAX_FILE_PATH);
  bzero(DataBuffer,20);
  strcat(FilenameBuffer,Filename);
  strcat(FilenameBuffer, SLAVE_FILE_EXTENSION);
  filefd = fopen (FilenameBuffer, "r");
  
  if(filefd)
  {     
    UtilReadLineCntSpecChars(filefd, DataBuffer);
    Time = atof(DataBuffer);
    OP->Type = 's';
    OP->SyncTime = Time;
    UtilReadLineCntSpecChars(filefd, DataBuffer);
    Time = atof(DataBuffer);
    OP->SyncStopTime = Time;
    fclose(filefd);
    if(debug) printf("Slave object set: %s, SyncTime: %3.4f\n", FilenameBuffer, Time);
  }


  return 0;
}


double UtilCalcPositionDelta(double P1Lat, double P1Long, double P2Lat, double P2Long, ObjectPosition *OP)
{

	double f, d, P1LatRad, P1LongRad, P2LatRad, P2LongRad, U1, U2, L, lambda, sins, coss, sigma, sinalpha, cosalpha2, cossm, C, lambdaprim, u2, A, B, dsigma, s, alpha1, alpha2, cosU2, sinlambda, cosU1, sinU1, sinU2, coslambda;

 	OP->Latitude = P2Lat;
	OP->Longitude = P2Long;	
	P1LatRad = UtilDegToRad(P1Lat);
	P1LongRad = UtilDegToRad(P1Long);
	P2LatRad = UtilDegToRad(P2Lat);
	P2LongRad = UtilDegToRad(P2Long);

	f = 1/k;
	U1 = atan((1-f)*tan(P1LatRad));
	U2 = atan((1-f)*tan(P2LatRad));
	L = P2LongRad - P1LongRad;
	lambda = L;
	lambdaprim = lambda;	
	//printf("Lambdadiff: %1.15f\n", fabs(lambda-lambdaprim));
	
	int i = ORIGO_DISTANCE_CALC_ITERATIONS;
	OP->CalcIterations = 0;	
	do
	{
		sins = sqrt( pow((cos(U2)*sin(lambda)),2) + pow((cos(U1)*sin(U2) - sin(U1)*cos(U2)*cos(lambda)),2) );
		if (sins==0) return 0; //co-incident points
		coss = sin(U1)*sin(U2) + cos(U1)*cos(U2)*cos(lambda);
		sigma = atan(sins/coss);	
		sinalpha = (cos(U1)*cos(U2)*sin(lambda))/sins;
		cosalpha2 = 1 - pow(sinalpha,2);
		cossm = coss - (2*sin(U1)*sin(U2) / cosalpha2);
		C = (f/16) * cosalpha2 * ( 4 + f*(4 - 3 * cosalpha2) );
		lambdaprim = lambda;		
		lambda = L + (1-C)*f*sinalpha*(sigma+C*sins*(cossm + C*coss*(-1+ 2*pow(cossm,2))));
		OP->CalcIterations ++;
		//printf("Lambdadiff: %1.15f\n", fabs(lambda-lambdaprim));
	
	} while(fabs(lambda-lambdaprim) > l  && --i > 0);

	if (i == 0)
	{
		//printf("Failed to converge!\n");
		OP->OrigoDistance = -1;
	}
	else
	{
		u2 = cosalpha2*((pow(a,2) - pow(b,2))/pow(b,2));
		A = 1 +(u2/16384)*(4096+u2*(-768+u2*(320-175*u2)));
		B = (u2/1024)*(256 + u2*(-128*u2*(74-47*u2)));
		dsigma = B*sins*(cossm+0.25*B*(coss*(-1+2*pow(cossm,2)) - (1/6)*B*cossm*(-3+4*pow(sins,2))*(-3+4*pow(cossm,2))));	
		s = b*A*(sigma-dsigma);
    OP->DeltaOrigoDistance = s - OP->OrigoDistance;
		OP->OrigoDistance = s;

		cosU2 = cos(U2);
		sinU2 = sin(U2); 
		cosU1 = cos(U1);
		sinU1 = sin(U1); 
		sinlambda = sin(lambda);
		coslambda = cos(lambda);

		OP->ForwardAzimuth1 = atan2(cosU2*sinlambda,(cosU1*sinU2-sinU1*cosU2*coslambda));
		OP->ForwardAzimuth2 = atan2(cosU1*sinlambda,(sinU1*cosU2*-1+cosU1*sinU2*coslambda));
    //OP->x = s*sin((OP->ForwardAzimuth1));
    //OP->y = s*cos((OP->ForwardAzimuth1));
    OP->x = s*sin((OP->ForwardAzimuth2));
    OP->y = s*cos((OP->ForwardAzimuth2));
  
	}
	return s;
}


double UtilDegToRad(double Deg){return (PI*Deg/180);}
double UtilRadToDeg(double Rad){return (180*Rad/PI);}


int UtilPopulateSpaceTimeArr(ObjectPosition *OP, char* TrajFile)
{

  FILE *Trajfd;
  int Rows, j = 0;
  char TrajRow[TRAJECTORY_LINE_LENGTH];

  Trajfd = fopen (TrajFile, "r");
  //printf("%s\n",TrajFile);
  if(Trajfd)
  {
    Rows = OP->TrajectoryPositionCount;
    printf("Rows = %d\n", Rows);
    double x, y, z;
    float t;
    char ValueStr[NUMBER_CHAR_LENGTH];
    char* src1; char* src2; 
    do
    {
      bzero(TrajRow,TRAJECTORY_LINE_LENGTH);
      if( UtilReadLineCntSpecChars(Trajfd, TrajRow) >= 10)
      { 
        bzero(ValueStr, NUMBER_CHAR_LENGTH);
        src1 = strchr(TrajRow, ';');
        src2 = strchr(src1+1, ';');
        strncpy(ValueStr, src1+1, (uint64_t)src2 - (uint64_t)src1);
        t = atof(ValueStr);
        //printf("%d :t = %3.3f\n", j, t);
        bzero(ValueStr, NUMBER_CHAR_LENGTH);
        src1 = strchr(src2, ';');
        src2 = strchr(src1+1, ';');
        strncpy(ValueStr, src1+1, (uint64_t)src2 - (uint64_t)src1);
        x = atof(ValueStr);
        
        bzero(ValueStr, NUMBER_CHAR_LENGTH);
        src1 = strchr(src2, ';');
        src2 = strchr(src1+1, ';');
        strncpy(ValueStr, src1+1, (uint64_t)src2 - (uint64_t)src1);
        y = atof(ValueStr);
        
        /*
        bzero(ValueStr, NUMBER_CHAR_LENGTH);
        src1 = strchr(src2, ';');
        src2 = strchr(src1+1, ';');
        strncpy(ValueStr, src1+1, (uint64_t)src2 - (uint64_t)src1);
        z = atof(ValueStr);
        */

        OP->SpaceArr[j]= (float)sqrt(pow(x,2) + pow(y,2));
        OP->TimeArr[j]= (float)t;

        OP->SpaceTimeArr[j].Index = j;
        OP->SpaceTimeArr[j].Time = (float)t;
        OP->SpaceTimeArr[j].OrigoDistance = (float)sqrt(pow(x,2) + pow(y,2) + pow(z,2));
        //OP->SpaceTimeArr[j].Bearing = tan(y/x);
        //OP->SpaceTimeArr[j].x = x;
        //OP->SpaceTimeArr[j].y = y;
        //printf("t = %5.3f\n", OP->TimeArr[j]);
        //printf("t = %5.3f\n", OP->TimeArr[j]);
        j ++;
      }
      

    } while (--Rows >= 0 /*j < 10*/);

    UtilSortSpaceTimeAscending(OP);
    
    //for(int g=0; g < OP->TrajectoryPositionCount; g ++)
    //{
    //  printf("OrigoDistance=%4.3f, Time=%4.3f, Index=%d\n", OP->SpaceTimeArr[g].OrigoDistance, OP->SpaceTimeArr[g].Time, OP->SpaceTimeArr[g].Index);
    //}
      

    fclose(Trajfd);
  } 
  else
  { 
    printf("Failed to open file:%s\n",  TrajFile);
  }

  return 0;
}

int UtilSetSyncPoint(ObjectPosition *OP, double x, double y, double z, double Time)
{

  int i = 0;
  int Gate1Reached = 0, Gate2Reached = 0;
  
  if(Time == 0)
  {
    float R = (float)sqrt(pow(x,2) + pow(y,2));
    while(i < (OP->TrajectoryPositionCount-1) && Gate1Reached == 0)
    {
        if( OP->SpaceArr[i] == R)
        {
          Gate1Reached = 1;
          OP->SyncIndex = i;
          //printf("Sync point found=%4.3f, Time=%4.3f, Index=%d\n", OP->SpaceArr[i], OP->TimeArr[i], i);
         } 
        else 
        {
          OP->SyncIndex = -1;
        }
        i ++; 
    }
  } else {

    while(i < (OP->TrajectoryPositionCount-1) && Gate1Reached == 0)
    {
        //printf("%4.3f, %4.3f\n", OP->TimeArr[i], OP->TimeArr[i+1]);

        if(Time >= OP->TimeArr[i]  &&  Time <= OP->TimeArr[i+1] )
        {
          Gate1Reached = 1;
          OP->SyncIndex = i;
         // printf("Sync point found=%4.3f, Time=%4.3f, Index=%d\n", OP->SpaceArr[i], OP->TimeArr[i], i);
         } 
        else 
        {
          OP->SyncIndex = -1;
        }
        i ++; 
    }

  }

}

float UtilCalculateTimeToSync(ObjectPosition *OP)
{
  
  float t = OP->TimeArr[OP->SyncIndex] - OP->TimeArr[OP->BestFoundTrajectoryIndex];
  OP->TimeToSyncPoint = t;
  return t;
}



int UtilSortSpaceTimeAscending(ObjectPosition *OP)
{
    int i, j, index;
    float r,t;
    for (i = 0; i < OP->TrajectoryPositionCount; ++i)
    {
        for (j = i + 1; j < OP->TrajectoryPositionCount; ++j)
        {
            if (OP->SpaceTimeArr[i].OrigoDistance > OP->SpaceTimeArr[j].OrigoDistance)
            {
                index = OP->SpaceTimeArr[i].Index;
                r = OP->SpaceTimeArr[i].OrigoDistance;
                t = OP->SpaceTimeArr[i].Time;
                OP->SpaceTimeArr[i].Index = OP->SpaceTimeArr[j].Index;
                OP->SpaceTimeArr[i].OrigoDistance = OP->SpaceTimeArr[j].OrigoDistance;
                OP->SpaceTimeArr[i].Time = OP->SpaceTimeArr[j].Time;
                OP->SpaceTimeArr[j].Index = index;
                OP->SpaceTimeArr[j].OrigoDistance = r;
                OP->SpaceTimeArr[j].Time = t;
            }
        }
    }
}


int UtilFindCurrentTrajectoryPosition(ObjectPosition *OP, int StartIndex, double CurrentTime, double MaxTrajDiff, double MaxTimeDiff, char debug)
{

  int i = StartIndex, j=0;
  int Gate1Reached = 0, Gate2Reached = 0, SampledSpaceIndex[SYNC_POINT_BUFFER];
  double Diff;
  
  if(i == -1) i = 0;
  OP->BestFoundTrajectoryIndex = 0;
  if(debug) printf("OPOrigoDistance=%4.3f, x=%4.3f, y=%4.3f\n", OP->OrigoDistance, OP->x, OP->y);      
  
  while(i < (OP->TrajectoryPositionCount-1) && Gate2Reached == 0)
  {
      
      Diff = fabs(OP->SpaceTimeArr[i].OrigoDistance - OP->OrigoDistance);
     //printf("%4.3f, %4.3f, %4.3f\n ", Diff,OP->SpaceTimeArr[i].OrigoDistance,OP->OrigoDistance);
     
      if( Diff < MaxTrajDiff  && Gate1Reached == 0)
      {
        Gate1Reached = 1;
      }

      if(Diff > MaxTrajDiff  && Gate1Reached == 1)
      {
        Gate2Reached = 1;
      }

      if(Gate1Reached == 1)
      {
          if(j < SYNC_POINT_BUFFER-1 && OP->SpaceTimeArr[i].Index <= OP->SyncIndex && OP->SpaceTimeArr[i].Index > OP->BestFoundTrajectoryIndex)
          {
            SampledSpaceIndex[j] = i;
            j++;
            if(debug) printf("i=%d, j=%d ,ArrOrigoDistance=%4.3f, Diff=%4.3f, ArrTime=%4.3f, Index=%d, CurrentTime=%4.3f \n",i , j, OP->SpaceTimeArr[i].OrigoDistance, Diff, OP->SpaceTimeArr[i].Time, OP->SpaceTimeArr[i].Index, CurrentTime);      
          } 
          else if(j >= SYNC_POINT_BUFFER)
          {
            printf("Sync point buffer overrun j=%d\n", j);
          }
      }

   i ++; 
  }

  if(j == 0)  OP->BestFoundTrajectoryIndex = TRAJ_POSITION_NOT_FOUND; //No trajectory position found

  int PositionFound = -1, kc = 0;
  if(OP->BestFoundTrajectoryIndex > TRAJ_POSITION_NOT_FOUND)
  {
    i = 0;
    int SampledTimeIndex[SYNC_POINT_BUFFER];
    while(i < j)
    {
      Diff = fabs(OP->SpaceTimeArr[SampledSpaceIndex[i]].Time - CurrentTime);
      if(debug) printf("%4.3f, ", Diff);
      if(Diff < MaxTimeDiff)
      { 
        SampledTimeIndex[kc] = SampledSpaceIndex[i];
        kc ++;
      }
      i ++;
    }
    
    if(debug) printf("\n"); 

    i = 0;
    int Init = 1;
    double PrevDiff = 0;
    while(i < kc)
    {
      if(Init == 1) PositionFound = SampledTimeIndex[i];
      Diff = fabs(OP->SpaceTimeArr[SampledTimeIndex[i]].OrigoDistance - OP->OrigoDistance); //+ fabs(OP->SpaceTimeArr[SampledSpaceIndex[i]].Time - CurrentTime);
      if(debug) printf("%4.3f, ", Diff);
      if(Diff < PrevDiff && Init == 0)
      {
       PositionFound = SampledTimeIndex[i];
      }
      Init = 0;
      PrevDiff = Diff;
      i ++;
    }
     if(debug) printf("\n"); 
    if(PositionFound > TRAJ_POSITION_NOT_FOUND)
    {
      OP->BestFoundTrajectoryIndex = OP->SpaceTimeArr[PositionFound].Index;
      OP->SpaceTimeFoundIndex = PositionFound;
      /*
      printf("BestFoundTrajectoryIndex=%d\n", OP->BestFoundTrajectoryIndex);
      printf("Current origo distance=%4.3f m\n", OP->OrigoDistance);
      printf("Current time=%4.3f s\n", CurrentTime);
      printf("Matched origo distance=%4.3f m\n", OP->SpaceTimeArr[PositionFound].OrigoDistance);
      printf("Distance error=%4.3f m\n", OP->OrigoDistance - OP->SpaceTimeArr[PositionFound].OrigoDistance);
      printf("Expected time=%4.3f s (index=%d)\n", OP->SpaceTimeArr[PositionFound].Time, OP->SpaceTimeArr[PositionFound].Index);
      printf("Time error=%4.3f s\n", CurrentTime - OP->SpaceTimeArr[PositionFound].Time);
      */
    }
    else
    {
      OP->BestFoundTrajectoryIndex = TRAJ_MASTER_LATE;
      //printf("Not in time\n");
    }
  }
  return PositionFound;
} 

int UtilCountFileRows(FILE *fd)
{
  int c = 0;
  int rows = 0;

  while(c != EOF)
  {
    
    c = fgetc(fd);
    //printf("%x-", c);
    if(c == '\n') rows++;
  }

  return rows;
}

int UtilReadLineCntSpecChars(FILE *fd, char *Buffer)
{
  int c = 0;
  int d = 0;
  int SpecChars = 0;
  int comment = 0;

  while( (c != EOF) && (c != '\n') )
  {
    c = fgetc(fd);
    //printf("%x-", c);
    if(c != '\n')
    {
      if(c == '/')
      {
       comment++;
       d = c;
      } 

      if(comment == 0)
      {
        *Buffer = (char)c;
        Buffer++;
        if(c == ';' || c == ':') SpecChars++;
      }
      else if (comment == 1 && c != '/')
      {
        *Buffer = (char)d;
        Buffer++;
        if(d == ';' || d == ':') SpecChars++;
        *Buffer = (char)c;
        Buffer++;
        if(c == ';' || c == ':') SpecChars++;
        comment = 0;
      }
      else if(comment == 2 && c == '\n') 
      {
        c = 1;
        comment = 0;
      } 
      else if(comment == 2 && c != '\n')
      {
        //just continue
      }
      
    } 
  }
  return SpecChars;
}


char* UtilSearchTextFile(char *Filename, char *Text1, char *Text2, char *Result)
{

  FILE *fd;
  
  char RowBuffer[MAX_ROW_SIZE];
  char DataBuffer[MAX_ROW_SIZE];
  char *PtrText1;
  char *PtrText2;
  int Length;
  int Found = 0;
  
  fd = fopen (Filename, "r");
  int RowCount = UtilCountFileRows(fd);
  fclose(fd);
  
  fd = fopen (Filename, "r");
  if(fd > 0)
  {
     do
    {
      bzero(RowBuffer, MAX_ROW_SIZE);
      UtilReadLineCntSpecChars(fd, RowBuffer);
      bzero(DataBuffer, MAX_ROW_SIZE);
      PtrText1 = strstr(RowBuffer, (const char *)Text1); 
      if(PtrText1 != NULL)
      {
        if(strlen(Text2) > 0)
        {
          PtrText2 = strstr((const char *)(PtrText1+1), (const char *)Text2);
          if(PtrText2 != NULL)
          {
            strncpy(Result, PtrText1+strlen(Text1), strlen(RowBuffer) - strlen(Text1) - strlen(Text2));
          }
        }
        else
        {
          strncpy(Result, PtrText1+strlen(Text1), strlen(RowBuffer) - strlen(Text1));
        }
        Found = 1;
      }
      RowCount--;
 
    } while(Found == 0 && RowCount >= 0);

    fclose(fd);
  }

  //printf("String found: %s\n", Result);
  return Result;

} 


int iUtilGetIntParaConfFile(char* pcParameter, int* iValue)
{
  int iResult;
  char pcValue[512];

  bzero(pcValue,512);
  iResult = iUtilGetParaConfFile(pcParameter,pcValue);

  if (iResult > 0)
  {
    *iValue = atoi(pcValue);
  }

  return iResult;
}

int iCommInit(const unsigned int uiMode, const char* name, const int iNonBlocking)
{
  struct mq_attr attr;
  int iResult;
  int iOFlag;
  unsigned int uiIndex;

  attr.mq_maxmsg = MQ_MAX_MSG;
  attr.mq_msgsize = MQ_MAX_MESSAGE_LENGTH;
  attr.mq_flags = 0;
  attr.mq_curmsgs = 0;

  tMQRecv = 0;

  for(uiIndex=0;uiIndex < MQ_NBR_QUEUES;++uiIndex)
  {
    ptMQSend[uiIndex] = 0;
  }

  strcpy(pcMessageQueueName,name);

  if(uiMode & IPC_RECV)
  {
    iOFlag = O_RDONLY | O_CREAT;
    if(iNonBlocking)
    {
      iOFlag |= O_NONBLOCK;
    }

    tMQRecv = mq_open(name, iOFlag, MQ_PERMISSION, &attr);
    if(tMQRecv < 0)
    {
      util_error("ERR: Failed to open receiving message queue");
    }
  }

  if(uiMode & IPC_SEND)
  {
    uiIndex = 0;

    if(strcmp(name,MQ_LG))
    {
      ptMQSend[uiIndex] = mq_open(MQ_LG, O_WRONLY | O_NONBLOCK | O_CREAT, MQ_PERMISSION, &attr);
      if(ptMQSend[uiIndex] < 0)
      {
        util_error("ERR: Failed to open MQ_LG message queue");
      }
      ++uiIndex;
    }
 
    if(strcmp(name,MQ_OC))
    {
      ptMQSend[uiIndex] = mq_open(MQ_OC, O_WRONLY | O_NONBLOCK | O_CREAT, MQ_PERMISSION, &attr);
      if(ptMQSend[uiIndex] < 0)
      {
        util_error("ERR: Failed to open MQ_OC message queue");
      }
      ++uiIndex;
    }

    if(strcmp(name,MQ_SV))
    {
      ptMQSend[uiIndex] = mq_open(MQ_SV, O_WRONLY | O_NONBLOCK | O_CREAT, MQ_PERMISSION, &attr);
      if(ptMQSend[uiIndex] < 0)
      {
        util_error("ERR: Failed to open MQ_SV message queue");
      }
      ++uiIndex;
    }

    if(strcmp(name,MQ_VA))
    {
      ptMQSend[uiIndex] = mq_open(MQ_VA, O_WRONLY | O_NONBLOCK | O_CREAT, MQ_PERMISSION, &attr);
      if(ptMQSend[uiIndex] < 0)
      {
        util_error("ERR: Failed to open MQ_VA message queue");
      }
      ++uiIndex;
    }
  }

  return 1;
}

int iCommClose()
{
  int iIndex = 0;
  int iResult;

  if(tMQRecv != 0 && pcMessageQueueName != NULL)
  {
    iResult = mq_unlink(pcMessageQueueName);
    if(iResult < 0)
    {
      return 0;
    }
    iResult = mq_close(tMQRecv);
    if(iResult < 0)
    {
      return 0;
    }
  }

  for(iIndex = 0; iIndex < MQ_NBR_QUEUES; ++iIndex)
  {
    if(ptMQSend[iIndex] != 0)
    {
      iResult = mq_close(ptMQSend[iIndex]);
      if(iResult < 0)
      {
        return 0;
      }
    }
  }

  return 1;
}

int iCommRecv(int* iCommand, char* cpData, const int iMessageSize)
{
  int iResult;
  char cpMessage[MQ_MAX_MESSAGE_LENGTH];
  unsigned int prio;
  
  bzero(cpMessage,MQ_MAX_MESSAGE_LENGTH);

  iResult = mq_receive(tMQRecv, cpMessage, MQ_MAX_MESSAGE_LENGTH, &prio);
    
  if(iResult < 0 && errno != EAGAIN)
  {
    util_error ("ERR: Error when recieveing");
  }
  else if((iResult >= 0))
  {
    *iCommand = cpMessage[0];
    if((strlen(cpMessage) > 1) && (cpData != NULL))
    {
      if(iMessageSize <  iResult )
      {
        iResult = iMessageSize;
      }
      (void)strncat(cpData,&cpMessage[1],iResult);
    }
  }
  else
  {
    *iCommand = COMM_INV;
    iResult = 0;
  }

  return iResult;
}

int iCommSend(const int iCommand,const char* cpData)
{
  int iResult;
  unsigned int uiMessagePrio = 0;
  int iIndex = 0;
  char cpMessage[MQ_MAX_MESSAGE_LENGTH];
  
  bzero(cpMessage,MQ_MAX_MESSAGE_LENGTH);
  
  if(iCommand == COMM_STRT)
    {
      uiMessagePrio = 100;
      cpMessage[0] = (char)COMM_STRT;
    }
  else if(iCommand == COMM_ARMD)
    {
      uiMessagePrio = 110;
      cpMessage[0] = (char)COMM_ARMD;
    }
  else if(iCommand == COMM_STOP)
    {
      uiMessagePrio = 120;
      cpMessage[0] = (char)COMM_STOP;
    }
  else if(iCommand == COMM_MONI)
    {
      uiMessagePrio = 80;
      cpMessage[0] = (char)COMM_MONI;
    }
  else if(iCommand == COMM_EXIT)
    {
      uiMessagePrio = 140;
      cpMessage[0] = (char)COMM_EXIT;
    }
  else if (iCommand == COMM_REPLAY)
    {
      uiMessagePrio = 160;
      cpMessage[0] = (char)COMM_REPLAY;
    }
  else if (iCommand == COMM_CONTROL)
    {
      uiMessagePrio = 180;
      cpMessage[0] = (char)COMM_CONTROL;
    }
  else if (iCommand == COMM_ABORT)
    {
    uiMessagePrio = 60;
    cpMessage[0] = (char)COMM_ABORT;
  }
  else
    {
      util_error("ERR: Unknown command");
    }
  
  if(cpData != NULL)
  {
    (void)strncat(&cpMessage[1],cpData,strlen(cpData));
	//printf("Util message data: %s\n", cpData);
  }

  for(iIndex = 0; iIndex < MQ_NBR_QUEUES; ++iIndex)
  {
    if(ptMQSend[iIndex] != 0)
    {
      iResult = mq_send(ptMQSend[iIndex],cpMessage,strlen(cpMessage),uiMessagePrio);
      if(iResult < 0)
      {
        return 0;
      }
    }
  }
  return 1;
}

/*------------------------------------------------------------
  -- Function traj2ldm
  --  converts a traj file format to a ldm:monitor_t
  ------------------------------------------------------------*/

void traj2ldm ( float      time ,
                double     x    ,
                double     y    ,
                double     z    ,
                float      hdg  ,
                float      vel  ,
                monitor_t* ldm  )
{

  char   pcTempBuffer [512];

  double earth_radius = a;

  double lat_origin = 0.0;
  double lon_origin = 0.0;
  double alt_origin = 0.0;
  double lat        = 0.0;
  double lon        = 0.0;
  double alt        = 0.0;

  ldm->timestamp = (uint64_t) (time * 100);
  ldm->speed     = (uint16_t) (vel * 100);
  ldm->heading   = (uint16_t) (hdg * 100);

  bzero(pcTempBuffer,512);
  iUtilGetParaConfFile("OrigoLatidude=", pcTempBuffer);
  sscanf(pcTempBuffer, "%lf", &lat_origin);
  
  bzero(pcTempBuffer,512);
  iUtilGetParaConfFile("OrigoLongitude=", pcTempBuffer);
  sscanf(pcTempBuffer, "%lf", &lon_origin);

  bzero(pcTempBuffer,512);
  iUtilGetParaConfFile("OrigoAltitude=", pcTempBuffer);
  sscanf(pcTempBuffer, "%lf", &alt_origin);

  lat = ((y * 180)/(PI * earth_radius)) + lat_origin;
  lon = ((x * 180)/(PI * earth_radius)) * (1 / (cos((PI / 180) * (0.5 * (lat_origin + lat))))) + lon_origin;
  alt = z + alt_origin;

  ldm->latitude  = (uint32_t) (lat * 10000000);
  ldm->longitude = (uint32_t) (lon * 10000000);
  ldm->altitude  = (uint32_t) (alt * 100);

}

