/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  ------------------------------------------------------------------------------
  -- File        : systemcontrol.h
  -- Author      : Karl-Johan Ode
  -- Description : CHRONOS
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

#ifndef __SYSTEMCONTROL_H_INCLUDED__
#define __SYSTEMCONTROL_H_INCLUDED__

#include "util.h"
#include "logging.h"

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
void systemcontrol_task(TimeType *GPSTime,  GSDType *GSD, LOG_LEVEL logLevel);

#endif //__SYSTEMCONTROL_H_INCLUDED__
