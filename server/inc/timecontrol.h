/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2018 MAESTRO project
  ------------------------------------------------------------------------------
  -- File        : timecontrol.h
  -- Author      : Sebastian Loh Lindholm
  -- Description : MAESTRO
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

#ifndef __TIMECONTROL_H_INCLUDED__
#define __TIMECONTROL_H_INCLUDED__

#include "util.h"
#include "logging.h"

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
void timecontrol_task(TimeType *GPSTime,  GSDType *GSD, LOG_LEVEL logLevel);
U16 TimeControlGetMillisecond(TimeType *GPSTime);


#endif 
