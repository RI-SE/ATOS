#include "timeconversions.h"
#include <errno.h>
#include <stddef.h>

#include "defines.h"

/*!
 * \brief setToGPStime Sets an Epoch timestamp according to supplied GPS time data
 * \param time Output epoch timestamp
 * \param GPSWeek GPS week
 * \param GPSqmsOfWeek GPS quarter millisecond of GPSWeek
 * \return 0 on success, -1 otherwise
 */
int8_t setToGPStime(struct timeval * time, const uint16_t GPSWeek, const uint32_t GPSqmsOfWeek) {
	if (time == NULL) {
		errno = EINVAL;
		return -1;
	}
	// Conserve precision of quarter milliseconds when building time struct
	uint64_t GPSqms = (uint64_t) (GPSWeek) * WEEK_TIME_QMS + (uint64_t) (GPSqmsOfWeek);
	uint64_t UTCqms = GPSqms + 4 * MS_TIME_DIFF_UTC_GPS - 4 * MS_LEAP_SEC_DIFF_UTC_GPS;

	time->tv_sec = (time_t) (UTCqms / 4000);
	time->tv_usec = (time_t) ((UTCqms % 4000) * 250);
	return 0;
}

/*!
 * \brief getAsGPSms Converts a timestamp into GPS milliseconds
 * \param time Epoch timestamp
 * \return GPS milliseconds, or 0 in case of an error
 */
uint64_t getAsGPSms(const struct timeval * time) {
	if (time == NULL) {
		errno = EINVAL;
		return 0;
	}
	if (time->tv_sec < 0) {
		errno = ERANGE;
		return 0;
	}
	return (uint64_t) (time->tv_sec * 1000 + time->tv_usec / 1000)
		- MS_TIME_DIFF_UTC_GPS;
}


/*!
 * \brief getAsGPSweek Converts a timestamp into the corresponding GPS week
 * \param time Epoch timestamp
 * \return Time represented as number of weeks, or -1 if error
 */
int32_t getAsGPSWeek(const struct timeval * time) {
	uint64_t GPSms;

	if ((GPSms = getAsGPSms(time)) == 0) {
		return -1;
	}
	return (uint16_t) (GPSms / WEEK_TIME_MS);
}

/*!
 * \brief getAsGPSQuarterMillisecondOfWeek Converts a timestamp into quarter
 *			milliseconds of the GPS week it represents.
 * \param time Epoch timestamp
 * \return Time represented as quarter milliseconds of week, or -1 if error
 */
int64_t getAsGPSQuarterMillisecondOfWeek(const struct timeval * time) {
	if (time == NULL) {
		errno = EINVAL;
		return -1;
	}
	uint64_t UTCqms = (uint64_t)(((int64_t)time->tv_sec)*4000 + ((int64_t)time->tv_usec)/250);
	uint64_t GPSqms = UTCqms - (uint64_t)(4*MS_TIME_DIFF_UTC_GPS - 4*MS_LEAP_SEC_DIFF_UTC_GPS);

	return (int64_t) (GPSqms % WEEK_TIME_QMS);
}
