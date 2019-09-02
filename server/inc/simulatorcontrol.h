/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2018 CHRONOS II project
  ------------------------------------------------------------------------------
  -- File        : simulatorcontrol.h
  -- Author      : Sebastian Loh Lindholm
  -- Description : CHRONOS II
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

#ifndef __SIMULATORCONTROL_H_INCLUDED__
#define __SIMULATORCONTROL_H_INCLUDED__

#include "logging.h"


/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
void simulatorcontrol_task(TimeType *GPSTime, GSDType *GSD, LOG_LEVEL logLevel);


#endif 