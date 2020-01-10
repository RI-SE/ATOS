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


typedef struct
{
  U8 SimulatorModeU8;
} SMGDType;



/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
int simulatorcontrol_task(TimeType *GPSTime, GSDType *GSD);


#endif 