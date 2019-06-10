/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  ------------------------------------------------------------------------------
  -- File        : supervision.h
  -- Author      : Karl-Johan Ode
  -- Description : CHRONOS
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

#ifndef __SUPERVISION_H_INCLUDED__
#define __SUPERVISION_H_INCLUDED__

#include "util.h"
#include "logging.h"


/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
void supervision_task(TimeType *GPSTime, GSDType *GSD, LOG_LEVEL logLevel);
#endif //__SUPERVISION_H_INCLUDED__
