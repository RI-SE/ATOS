/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef __JOURNAL_HPP
#define __JOURNAL_HPP

#define JOURNAL_FILE_ENDING ".jnl"

#include "util.h"
#include <rclcpp/logging.hpp>

typedef enum {
	JOURNAL_RECORD_MONITOR_DATA,
	JOURNAL_RECORD_EVENT,
	JOURNAL_RECORD_STRING
} JournalRecordType;

int JournalInit(const char* name, rclcpp::Logger logger);
int JournalRecordData(JournalRecordType type, const char* format, ...);
int JournalRecordMonitorData(const ObjectDataType* data);

#endif
