/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2018 CHRONOS II project
  ------------------------------------------------------------------------------
  -- File        : citscontrol.h
  -- Author      : Sebastian Loh Lindholm
  -- Description : CHRONOS II
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

#ifndef __SUPERVISORCONTROL_H_INCLUDED__
#define __SUPERVISORCONTROL_H_INCLUDED__


#include "util.h"

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
void supervisorcontrol_task(TimeType *GPSTime, GSDType *GSD);


#endif 
