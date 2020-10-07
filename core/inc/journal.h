#ifndef __JOURNAL_H
#define __JOURNAL_H

#define JOURNAL_FILE_ENDING ".jnl"

#include "positioning.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	JOURNAL_RECORD_MONITOR_DATA,
	JOURNAL_RECORD_EVENT,
	JOURNAL_RECORD_STRING
} JournalRecordType;

int JournalInit(const char* name);
int JournalRecordData(JournalRecordType type, const char* format, ...);
int JournalRecordMonitorData(const ObjectDataType* data);



#ifdef __cplusplus
}
#endif
#endif
