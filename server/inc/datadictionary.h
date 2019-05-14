/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2018 CHRONOS II project
  ------------------------------------------------------------------------------
  -- File        : datadictionary.h
  -- Author      : Sebastian Loh Lindholm
  -- Description : CHRONOS II
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

#ifndef __DATADICTIONARY_H_INCLUDED__
#define __DATADICTIONARY_H_INCLUDED__

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
void DataDictionaryInitExternalSupervisorIPU32(void);
void DataDictionarySetExternalSupervisorIPU32(U32 IP);
U32 DataDictionaryGetExternalSupervisorIPU32(void);

void DataDictionaryConstructor(void);

#endif 