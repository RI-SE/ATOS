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
#include "logging.h"

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
void objectcontrol_task(TimeType *GPSTime, GSDType *GSD, LOG_LEVEL logLevel);

I32 ObjectControlBuildOSEMMessage(C8* MessageBuffer, OSEMType *OSEMData, TimeType *GPSTime, C8 *Latitude, C8 *Longitude, C8 *Altitude, U8 debug);
I32 ObjectControlBuildOSTMMessage(C8* MessageBuffer, OSTMType *OSTMData, C8 CommandOption, U8 debug);
I32 ObjectControlBuildHEABMessage(C8* MessageBuffer, HEABType *HEABData, TimeType *GPSTime, U8 CCStatus, U8 debug);
int ObjectControlBuildLLCMMessage(char* MessageBuffer, unsigned short Speed, unsigned short Curvature, unsigned char Mode, char debug);
I32 ObjectControlBuildSYPMMessage(C8* MessageBuffer, SYPMType *SYPMData, U32 SyncPoint, U32 StopTime, U8 debug);
I32 ObjectControlBuildMTSPMessage(C8* MessageBuffer, MTSPType *MTSPData, U32 SyncTimestamp, U8 debug);
I32 ObjectControlBuildTRAJMessageHeader(C8* MessageBuffer, I32 *RowCount, HeaderType *HeaderData, TRAJInfoType *TRAJInfoData, C8 *TrajFileHeader, U8 debug);
I32 ObjectControlBuildTRAJMessage(C8* MessageBuffer, FILE *fd, I32 RowCount, DOTMType *DOTMType, U8 debug);
I32 ObjectControlBuildVOILMessage(C8* MessageBuffer, VOILType *VOILData, C8* SimData, U8 debug);
I32 ObjectControlSendTRAJMessage(C8* Filename, I32 *Socket, I32 RowCount, C8 *IP, U32 Port, DOTMType *DOTMData, U8 debug);
I32 ObjectControlMONRToASCII(MONRType *MONRData, GeoPosition *OriginPosition, I32 Idn, C8 *Id, C8 *Timestamp, C8 *XPosition, C8 *YPosition, C8 *ZPosition, C8 *LongitudinalSpeed, C8 *LateralSpeed, C8 *LongitudinalAcc, C8 *LateralAcc, C8 *Heading, C8 *DriveDirection, C8 *ObjectState, C8 *ReadyToArm, C8* ErrorStatus, C8 debug);
int ObjectControlOSEMtoASCII(OSEMType *OSEMData,char *GPSWeek, char *GPSLatitude, char *GPSLongitude, char *GPSAltitude);  
#endif //__OBJECTCONTROL_H_INCLUDED__
