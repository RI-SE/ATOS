/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2020 AstaZero
  ------------------------------------------------------------------------------
  -- File        : journalcontrol.h
  -- Author      : Lukas Wikander
  -- Description :
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

#ifndef __JOURNALCONTROL_H_INCLUDED__
#define __JOURNALCONTROL_H_INCLUDED__

#ifdef __cplusplus
extern "C" {
#endif

#include "util.h"
#include "logging.h"

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/

void journalcontrol_task(TimeType *GPSTime, GSDType *GSD, LOG_LEVEL logLevel);

#ifdef __cplusplus
}
#endif

#endif //__LOGGER_H_INCLUDED__
