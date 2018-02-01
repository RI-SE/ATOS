/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  ------------------------------------------------------------------------------
  -- File        : objectcontrol.h
  -- Author      : Karl-Johan Ode
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
void objectcontrol_task();

int ObjectControlBuildOSEMMessage(char* MessageBuffer, OSEMType *OSEMData, char *Latitude, char *Longitude, char *Altitude, char *Heading, char debug);
int ObjectControlBuildSTRTMessage(char* MessageBuffer, unsigned char CommandOption, unsigned long TimeStamp, char debug);
int ObjectControlBuildAROMMessage(char* MessageBuffer, unsigned char CommandOption, char debug);
int ObjectControlBuildHEABMessage(char* MessageBuffer, unsigned char CommandOption, char debug);
int ObjectControlBuildLLCMMessage(char* MessageBuffer, unsigned short Speed, unsigned short Curvature, unsigned char Mode, char debug);
int ObjectControlBuildSYPMMessage(char* MessageBuffer, unsigned int SyncPoint, unsigned int StopTime, char debug);
int ObjectControlBuildMTSPMessage(char* MessageBuffer, unsigned long SyncTimestamp, char debug);
int ObjectControlBuildDOPMMessageHeader(char* MessageBuffer, int RowCount, char debug);
int ObjectControlBuildDOPMMessage(char* MessageBuffer, FILE *fd, int RowCount, char debug);
int ObjectControlSendDOPMMEssage(char* Filename, int *Socket, int RowCount, char *IP, uint32_t Port,char debug);
int ObjectControlMONRToASCII(unsigned char *MonrData, int Idn, char *Id, char *Timestamp, char *Latitude, char *Longitude, char *Altitude, char *Speed ,char *Heading, char *DriveDirection, char *StatusFlag, char debug);

#endif //__OBJECTCONTROL_H_INCLUDED__