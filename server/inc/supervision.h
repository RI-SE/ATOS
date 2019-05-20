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


#include <stdio.h>
#include "util.h"

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
void supervision_task(TimeType *GPSTime, GSDType *GSD);

#endif //__SUPERVISION_H_INCLUDED__
