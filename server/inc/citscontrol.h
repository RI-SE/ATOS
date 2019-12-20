/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2018 CHRONOS II project
  ------------------------------------------------------------------------------
  -- File        : citscontrol.h
  -- Author      : Sebastian Loh Lindholm
  -- Description : CHRONOS II
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

#ifndef __CITSCONTROL_H_INCLUDED__
#define __CITSCONTROL_H_INCLUDED__

#include "util.h"
#include "logging.h"

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
void citscontrol_task(TimeType *GPSTime, GSDType *GSD, LOG_LEVEL logLevel);


#endif 
