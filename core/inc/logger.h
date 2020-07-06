/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  ------------------------------------------------------------------------------
  -- File        : logger.h
  -- Author      : Karl-Johan Ode
  -- Description : CHRONOS
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

#ifndef __LOGGER_H_INCLUDED__
#define __LOGGER_H_INCLUDED__

#ifdef __cplusplus
extern "C" {
#endif

#include "util.h"
#include "logging.h"

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/

void logger_task(TimeType *GPSTime, GSDType *GSD, LOG_LEVEL logLevel);

#ifdef __cplusplus
}
#endif

#endif //__LOGGER_H_INCLUDED__
