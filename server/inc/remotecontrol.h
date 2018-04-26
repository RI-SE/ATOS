/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  ------------------------------------------------------------------------------
  -- File        : remotecontrol.h
  -- Author      : Sebastian Loh Lindholm
  -- Description : CHRONOS
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

#ifndef __REMOTECONTROL_H_INCLUDED__
#define __REMOTECONTROL_H_INCLUDED__

#include <stdio.h>
#include <inttypes.h>
#include "util.h"

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
void remotecontrol_task();
void RemoteControlConnectServer(int* sockfd, const char* name, const uint32_t port);
U32 RemoteControlSignIn(I32 ServerSocketI32, C8 *TablenameC8, C8 *UsernameC8, C8 *PasswordC8, ServiceSessionType *SSData, U8 Debug);
U32 RemoteControlSendServerStatus(I32 ServerSocketI32, ServiceSessionType *SessionData, U32 StatusU32, U8 Debug);



#endif 