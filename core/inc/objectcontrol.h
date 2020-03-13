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

#endif //__OBJECTCONTROL_H_INCLUDED__
