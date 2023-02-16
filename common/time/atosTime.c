
/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "atosTime.h"
#include <stdio.h>
#include <stdlib.h>

#define MILLISECOND_FORMAT_SPECIFIER "%q"

// *** Time functions ***

uint64_t TimeGetGPSmsOfMinute(struct timeval* time)
{
    int64_t GPSms = TimeGetAsGPSms(time);
    uint64_t gpsSeconds = GPSms/1000;
    uint64_t gpsMilliSeconds = GPSms%1000;
    
    return (uint64_t)((gpsSeconds%60)*1000 + gpsMilliSeconds);
}

int64_t TimeGetAsGPSminutes(const struct timeval* time)
{
    return TimeGetAsGPSms(time) / 1000 / 60;
}

int64_t TimeGetAsGPSms(const struct timeval *time)
{
    return TimeGetAsUTCms(time) - MS_TIME_DIFF_UTC_GPS + MS_LEAP_SEC_DIFF_UTC_GPS;
}

int64_t TimeGetAsETSIms(const struct timeval *time)
{
    return TimeGetAsUTCms(time) - MS_FROM_1970_TO_2004_NO_LEAP_SECS + DIFF_LEAP_SECONDS_UTC_ETSI*1000;
}

uint32_t TimeGetAsGPSqmsOfWeek(const struct timeval *time)
{
    // Avoiding getUTCms(time) here to preserve (potential) precision from microseconds
    uint64_t UTCqms = (uint64_t)(time->tv_sec*4000 + time->tv_usec/250);
    uint64_t GPSqms = UTCqms - 4*MS_TIME_DIFF_UTC_GPS + 4*MS_LEAP_SEC_DIFF_UTC_GPS;

    return (uint32_t)(GPSqms % WEEK_TIME_QMS);
}

uint32_t TimeGetAsGPSSecondOfWeek(const struct timeval *time)
{
    return TimeGetAsGPSqmsOfWeek(time) / 4 / 1000;
}

uint32_t TimeGetAsGPSSecondOfDay(const struct timeval *time)
{
    return (TimeGetAsGPSms(time) % DAY_TIME_MS) / 1000;
}

uint16_t TimeGetAsGPSweek(const struct timeval *time)
{
    return (uint16_t)(TimeGetAsGPSms(time) / WEEK_TIME_MS);
}

int64_t TimeGetAsUTCms(const struct timeval *time)
{
    return time->tv_sec*1000 + time->tv_usec/1000;
}

struct timeval * TimeSetToCurrentSystemTime(struct timeval *time)
{
	// Get system time
	struct timespec currentTime;
	clock_gettime(CLOCK_REALTIME, &currentTime);
	time->tv_sec = currentTime.tv_sec;
	time->tv_usec = currentTime.tv_nsec / 1000;

    return time;
}

struct timeval * TimeSetToUTCms(struct timeval *time, int64_t UTCms)
{
    time->tv_sec = (time_t)(UTCms / 1000);
    //time->tv_usec = (__suseconds_t)(1000 * (UTCms % 1000));
	time->tv_usec = (suseconds_t)(1000 * UTCms - 1000000 * time->tv_sec);

    return time;
}

struct timeval * TimeSetToGPSms(struct timeval *time, int64_t GPSms)
{
	int64_t UTCms = GPSms + MS_TIME_DIFF_UTC_GPS - MS_LEAP_SEC_DIFF_UTC_GPS;

    return TimeSetToUTCms(time,UTCms);
}

struct timeval * TimeSetToGPStime(struct timeval *time, uint16_t GPSweek, uint32_t GPSqmsOfWeek)
{
    // Conserve precision of quarter milliseconds when building time struct
    uint64_t GPSqms = (uint64_t)(GPSweek) * WEEK_TIME_QMS + (uint64_t)(GPSqmsOfWeek);
    uint64_t UTCqms = GPSqms + 4*MS_TIME_DIFF_UTC_GPS - 4*MS_LEAP_SEC_DIFF_UTC_GPS;
    time->tv_sec = (time_t)(UTCqms / 4000);
    //time->tv_usec = (__suseconds_t)((UTCqms % 4000) * 250);
    time->tv_usec = (suseconds_t)((UTCqms % 4000) * 250);
    return time;
}

char * TimeGetAsDateTime(const struct timeval *time, const char *format, char *buffer, unsigned int bufferLength)
{
    unsigned long fmtLen = 0;
    char *msFmt;
    char msString[5];
    char* fmtCopy;

    if (fmtLen > bufferLength) return NULL;

    // Use a copy of the format string to modify
    fmtCopy = malloc(fmtLen+1);
    strcpy(fmtCopy,format);

    // Find possible millisecond format specifier
    msFmt = strstr(fmtCopy, MILLISECOND_FORMAT_SPECIFIER);
    if (msFmt != NULL)
        memset(msFmt, 0, strlen(MILLISECOND_FORMAT_SPECIFIER));

    time_t seconds = time->tv_sec;
    struct tm *tmstruct = localtime(&seconds);
    strftime(buffer, bufferLength, fmtCopy, tmstruct);

    // If there was a millisecond format specifier, print milliseconds
    if (msFmt != NULL)
    {
        sprintf(msString, "%ld", time->tv_usec/1000);
        strcat(buffer, msString);

        // If there is a remainder of the format string, print it
        if (*(msFmt + strlen(MILLISECOND_FORMAT_SPECIFIER)) != '\0')
        {
            strftime(buffer+strlen(buffer), bufferLength-strlen(buffer),
                     msFmt + strlen(MILLISECOND_FORMAT_SPECIFIER), tmstruct);
        }
    }

    free(fmtCopy);
    return buffer;
}

int64_t TimeGetTimeDifferenceMS(const struct timeval * stopTime, const struct timeval * startTime)
{
    struct timeval diff;
    timersub(stopTime, startTime, &diff);
    return diff.tv_sec*1000 + diff.tv_usec/1000;
}
