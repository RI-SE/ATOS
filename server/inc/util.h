/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  ------------------------------------------------------------------------------
  -- File        : util.h
  -- Author      : Karl-Johan Ode
  -- Description : CHRONOS
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

#ifndef __UTIL_H_INCLUDED__
#define __UTIL_H_INCLUDED__

/*------------------------------------------------------------
  -- Include files.
  ------------------------------------------------------------*/
#include <inttypes.h>
#include <mqueue.h>

/*------------------------------------------------------------
  -- Defines
  ------------------------------------------------------------*/
#define MQ_LG     "/TEServer-LG"
#define MQ_SV     "/TEServer-SV"
#define MQ_OC     "/TEServer-OC"
#define MQ_VA     "/TEServer-VA"
#define MQ_SC     "/DUMMY"

#define MQ_MAX_MESSAGE_LENGTH 4096
#define MQ_MAX_MSG            10
#define MQ_PERMISSION         0660

#define IPC_RECV       0x01
#define IPC_SEND       0x02
#define IPC_RECV_SEND  0x03

#define COMM_TRIG 1
#define COMM_STOP 2
#define COMM_MONI 3
#define COMM_EXIT 4

#define SAFETY_CHANNEL_PORT 53240
#define CONTROL_CHANNEL_PORT 53241

#define MAX_OBJECTS 10
#define MAX_FILE_PATH 256

//#define DEBUG


/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
void util_error(char* message);
void vUtilFindObjectsInfo(char object_traj_file[MAX_OBJECTS][MAX_FILE_PATH], 
  char object_address_name[MAX_OBJECTS][MAX_FILE_PATH],
  uint32_t object_port[MAX_OBJECTS],
  int* nbr_objects,
  const uint32_t default_port);
int iUtilGetParaConfFile(char* pcParameter, char* pcValue);

int iCommInit(const unsigned int, const char*, const int);
int iCommClose();
int iCommRecv(int*, char*, const int);
int iCommSend(const int,const char*);

#endif //__UTIL_H_INCLUDED__
