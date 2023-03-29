/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#pragma once



#include <stdint.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>

#ifdef __cplusplus
#include <ratio>
#include <chrono>
namespace std::chrono {
	using quartermilliseconds = std::chrono::duration<int64_t, std::ratio<1,4000>>;
	using weeks = std::chrono::duration<uint16_t, std::ratio<7*24*60*60,1>>;

	template<typename Duration>
	struct timeval to_timeval(Duration&& d) {
		std::chrono::seconds const sec = std::chrono::duration_cast<std::chrono::seconds>(d);
		struct timeval tv;
		tv.tv_sec  = sec.count();
		tv.tv_usec = std::chrono::duration_cast<std::chrono::microseconds>(d - sec).count();
		return tv;
	}

	template<typename Duration>
	void from_timeval(struct timeval & tv, Duration& d) {
		// TODO
		//const auto sec = std::chrono::seconds(tv.tv_sec);
		//const auto usec = std::chrono::microseconds(tv.tv_usec);
		//d = sec + usec;
	}
}
extern "C"{
#endif

/* 34 years between 1970 and 2004, 8 days for leap year between 1970 and 2004      */
/* Calculation: 34 * 365 * 24 * 3600 * 1000 + 8 * 24 * 3600 * 1000 = 1072915200000 */
#define MS_FROM_1970_TO_2004_NO_LEAP_SECS ((uint64_t)(1072915200000))

/* Difference of leap seconds between UTC and ETSI */
#define DIFF_LEAP_SECONDS_UTC_ETSI 5

// Between 1970 01 01 and 1980 01 06 there is 365*10 days, plus 2 for 2 leap years and plus 5 for the remaining days
// in total we have MStime= (3650 + 2 + 5) * 24 * 3600 * 1000 = 315964800000
#define MS_TIME_DIFF_UTC_GPS ((uint64_t)(315964800000))
// Difference is 18 leap seconds between utc and gps
#define MS_LEAP_SEC_DIFF_UTC_GPS (18000)


// 7 * 24 * 3600 * 1000 * 4
#define WEEK_TIME_QMS 2419200000
// 7 * 24 * 3600 * 1000
#define WEEK_TIME_MS 604800000
// 24 * 3600 * 1000
#define DAY_TIME_MS 86400000
// 3600 * 1000
#define HOUR_TIME_MS 3600000
// 60 * 1000
#define MINUTE_TIME_MS 60000



/* The time functions all use the same basic format for handling time - UTC seconds and microseconds
 * as specified by the timeval struct. This way, a single time variable can be extracted or created
 * through various formats but only one variable is needed to keep track of it. The functions below
 * can then express the timeval in whatever format is necessary.
 *
 * Some of the functions return a pointer to the input struct to reduce the amount of boilerplate code
 * necessary to use them.
 */

// Macro to compare a time to zero (useful for when a timestamp is in relative time)
#define timerpos(tmr) 						      \
  (((tmr)->tv_sec == 0) ? 					      \
   ((tmr)->tv_usec > 0) : 					      \
   ((tmr)->tv_sec > 0))

/* GetGPSmsOfMinute:
 *  Returns the number of milliseconds on top of GPS minute.
 */
uint64_t TimeGetGPSmsOfMinute(struct timeval* time); 

/* GetAsGPSminutes:
 *  Returns the number of minutes elapsed between 00:00:00:0000 January 6th 1980 and the time specified
 *  by the input time struct.
 */
int64_t TimeGetAsGPSminutes(const struct timeval* time); 

/* getAsGPSms:
 *  Returns the number of milliseconds between 00:00:00:0000 January 6th 1980 and the time specified
 *  by the input time struct, i.e. the absolute time in milliseconds of the input in GPST.
 */
int64_t TimeGetAsGPSms(const struct timeval* time);

/* getAsETSIms:
 *  Returns the number of milliseconds between 00:00:00:0000 January 1st 2004 and the time specified
 *  by the input time struct, i.e. the absolute time in milliseconds of the input in ETSI time.
 */
int64_t TimeGetAsETSIms(const struct timeval *time);

/* getAsGPSweek:
 *  Returns the number of weeks between 00:00:00:0000 January 6th 1980 and the time specified by the
 *  input time struct, i.e. the current GPS week (for reference, the first GPS week of 2018 was #1982).
 */
uint16_t TimeGetAsGPSweek(const struct timeval *time);

/* getAsGPSqmsOfWeek:
 *  Returns the number of quarter milliseconds between the start of the current GPS week and the time
 *  specified by the input time struct.
 */
uint32_t TimeGetAsGPSqmsOfWeek(const struct timeval *time);

/* TimeGetAsGPSSecondOfWeek:
 *  Returns the number of seconds between the start of the current GPS week and the time
 *  specified by the input time struct.
 */
uint32_t TimeGetAsGPSSecondOfWeek(const struct timeval *time);

/* TimeGetAsGPSSecondOfDay:
 *  Returns the number of seconds between the start of the current day and the time
 *  specified by the input time struct.
 */
uint32_t TimeGetAsGPSSecondOfDay(const struct timeval *time);

/* getAsUTCms:
 *  Returns the number of milliseconds between 00:00:00:0000 January 1st 1970 and the time specified
 *  by the input time struct, including leap seconds.
 */
int64_t TimeGetAsUTCms(const struct timeval *time);

/* setToCurrentSystemTime:
 *  Returns a pointer to the input time struct after having inserted the current time into the struct.
 */
struct timeval * TimeSetToCurrentSystemTime(struct timeval *time);

/* setToUTCms:
 *  Returns a pointer to the input time struct after having recalculated the input UTC timestamp and
 *  inserted it into the struct.
 */
struct timeval * TimeSetToUTCms(struct timeval *time, int64_t UTCms);

/* setToGPStime:
 *  Returns a pointer to the input time struct after having recalculated the input GPS time data and
 *  inserted it into the struct.
 */
struct timeval * TimeSetToGPStime(struct timeval *time, uint16_t GPSweek, uint32_t GPSqmsOfWeek);

/* setToGPSms:
 *  Returns a pointer to the input time struct after having recalculated the input GPS timestamp and
 *  inserted it into the struct.
 */
struct timeval * TimeSetToGPSms(struct timeval *time, int64_t GPSms);

/* getAsDateTime:
 *  Creates a timestamp string on the format specified and places it in the input
 *  buffer and returns a pointer to the same buffer. The format follows that of strftime e.g. %Y%m%d%H%M%S
 */
char * TimeGetAsDateTime(const struct timeval *time, const char *format, char *buffer, unsigned int bufferLength);

/* getTimeDifferenceMS:
 *  Basically a wrapper and millisecond converter around timersub().
 */
int64_t TimeGetTimeDifferenceMS(const struct timeval * stopTime, const struct timeval * startTime);



#ifdef __cplusplus
}
#endif
