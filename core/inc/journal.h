#ifndef __JOURNAL_H
#define __JOURNAL_H

#define JOURNAL_FILE_ENDING ".jnl"

#include "positioning.h"

#ifdef __cplusplus
extern "C" {
#endif

int JournalInit(const char* name);
int JournalRecordString(const char* format, ...);
int JournalRecordMonitorData(const ObjectMonitorType* data);

#ifdef __cplusplus
}
#endif
#endif
