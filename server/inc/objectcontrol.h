/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  ------------------------------------------------------------------------------
  -- File        : objectcontrol.h
  -- Author      : Sebastian Loh Lindholm
  -- Description : CHRONOS
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

#ifndef __OBJECTCONTROL_H_INCLUDED__
#define __OBJECTCONTROL_H_INCLUDED__

#include <stdio.h>
#include <inttypes.h>
#include "util.h"

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
void objectcontrol_task(TimeType *GPSTime, GSDType *GSD);

I32 ObjectControlBuildOSEMMessage(C8* MessageBuffer, OSEMType *OSEMData, TimeType *GPSTime, C8 *Latitude, C8 *Longitude, C8 *Altitude, C8 *Heading, U8 debug);
I32 ObjectControlBuildSTRTMessage(C8* MessageBuffer, STRTType *STRTData, TimeType *GPSTime, U32 ScenarioStartTime, U32 DelayStart, U32 *OutgoingStartTime, U8 debug);
I32 ObjectControlBuildOSTMMessage(C8* MessageBuffer, OSTMType *OSTMData, C8 CommandOption, U8 debug);
I32 ObjectControlBuildHEABMessage(C8* MessageBuffer, HEABType *HEABData, TimeType *GPSTime, U8 CCStatus, U8 debug);
int ObjectControlBuildLLCMMessage(char* MessageBuffer, unsigned short Speed, unsigned short Curvature, unsigned char Mode, char debug);
int ObjectControlBuildSYPMMessage(char* MessageBuffer, unsigned int SyncPoint, unsigned int StopTime, char debug);
int ObjectControlBuildMTSPMessage(char* MessageBuffer, unsigned long SyncTimestamp, char debug);
I32 ObjectControlBuildDOTMMessageHeader(C8* MessageBuffer, I32 RowCount, HeaderType *HeaderData, U8 debug);
I32 ObjectControlBuildDOTMMessage(C8* MessageBuffer, FILE *fd, I32 RowCount, DOTMType *DOTMType, U8 debug);
I32 ObjectControlBuildVOILMessage(C8* MessageBuffer, VOILType *VOILData, C8* SimData, U8 debug);
I32 ObjectControlSendDOTMMEssage(C8* Filename, I32 *Socket, I32 RowCount, C8 *IP, U32 Port, DOTMType *DOTMData, U8 debug);
//int ObjectControlMONRToASCII(MONRType *MONRData, int Idn, char *Id, char *Timestamp, char *Latitude, char *Longitude, char *Altitude, char *Speed ,char *Heading, char *DriveDirection, char *StatusFlag, char debug);
int ObjectControlMONRToASCII(MONRType *MONRData, GeoPosition *OriginPosition, int Idn, char *Id, char *Timestamp, char *Latitude, char *Longitude, char *Altitude, char *Speed, char *LateralSpeed, char *LongitudinalAcc, char *LateralAcc, char *Heading, char *DriveDirection, char *StatusFlag, char *StateFlag, char debug);
#endif //__OBJECTCONTROL_H_INCLUDED__