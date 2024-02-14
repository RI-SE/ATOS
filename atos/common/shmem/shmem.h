/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef SHMEM_H
#define SHMEM_H
#ifdef __cplusplus
extern "C" {
#endif
/*! ------------------------------------------------------------------------------
 *  -- File			: sharedmemory.h
 *  -- Author		: Lukas Wikander
 *  -- Description	: This file provides function for handling shared memory in a
 *					  thread safe manner.
 *  -- Reference	: manpage shm_overview, sem_overview
 *  ------------------------------------------------------------------------------
 */

#include <stdio.h>

volatile void* createSharedMemory(const char* memoryName, const unsigned int initialElements, const size_t elementSize, int* wasCreated);
volatile void* claimSharedMemory(volatile void* addr);
volatile void* releaseSharedMemory(volatile void* addr);
volatile void* resizeSharedMemory(volatile void* addr, const unsigned int newNumberOfElements);
void destroySharedMemory(volatile void* addr);
void closeSharedMemory(volatile void* addr);
int getNumberOfMemoryElements(volatile void* addr);
ssize_t getMemorySize(volatile void* addr);
ssize_t getElementSize(volatile void* addr);



#ifdef __cplusplus
}
#endif
#endif
