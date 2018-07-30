/*
    Copyright 2016 Benjamin Vedder	benjamin@vedder.se

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef BUFFER_H_
#define BUFFER_H_

#include <stdint.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>

namespace utility {

/* 34 years between 1970 and 2004, 8 days for leap year between 1970 and 2004      */
/* Calculation: 34 * 365 * 24 * 3600 * 1000 + 8 * 24 * 3600 * 1000 = 1072915200000 */
#define MS_FROM_1970_TO_2004_NO_LEAP_SECS 1072915200000

/* Difference of leap seconds between UTC and ETSI */
#define DIFF_LEAP_SECONDS_UTC_ETSI 5

// Between 1970 01 01 and 1980 01 06 there is 365*10 days, plus 2 for 2 leap years and plus 5 for the remaining days
// in total we have MStime= (3650 + 2 + 5) * 24 * 3600 * 1000 = 315964800000
#define MS_TIME_DIFF_UTC_GPS 315964800000
// Difference is 18 leap seconds between utc and gps
#define MS_LEAP_SEC_DIFF_UTC_GPS 18000

// 7 * 24 * 3600 * 1000
#define WEEK_TIME_MS 604800000
// 24 * 3600 * 1000
#define DAY_TIME_MS 86400000
// 3600 * 1000
#define HOUR_TIME_MS 3600000
// 60 * 1000
#define MINUTE_TIME_MS 60000

// GPS TIME FUNCTIONS
uint64_t getGPSmsFromUTCms(uint64_t UTCms);
uint64_t getUTCmsFromGPSms(uint64_t GPSms);
uint64_t getMSfromGPStime(uint16_t GPSweek,uint32_t GPSquarterMSofWeek);
void getGPStimeFromMS(uint64_t GPSms, uint16_t &GPSweek, uint32_t &GPSquarterMSofWeek);
// ETSI FUNCTIONS
uint64_t getCurrentETSItimeMS();
uint64_t getETSItimeFromUTCtimeMS(uint64_t UTCtime);
uint64_t getCurrentUTCtimeMS();
void getDateTimeFromUTCtime(int64_t utc_ms, char *buffer, int size_t);
void getDateTimeFromETSItime(uint64_t etsi_time_ms, char *buffer, int size_t);

void buffer_append_int64(uint8_t* buffer, int64_t number, int32_t *index);
void buffer_append_uint64(uint8_t *buffer, uint64_t number, int32_t *index);
void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index);
void buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index);
void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index);
void buffer_append_uint16(uint8_t* buffer, uint16_t number, int32_t *index);
void buffer_append_double16(uint8_t* buffer, double number, double scale, int32_t *index);
void buffer_append_double32(uint8_t* buffer, double number, double scale, int32_t *index);
void buffer_append_double64(uint8_t* buffer, double number, double scale, int32_t *index);
void buffer_append_double32_auto(uint8_t* buffer, double number, int32_t *index);
int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index);
uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index);
int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index);
uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index);
uint64_t buffer_get_uint64(const uint8_t *buffer, int32_t *index);
int64_t buffer_get_int64(const uint8_t *buffer, int32_t *index);
double buffer_get_double16(const uint8_t *buffer, double scale, int32_t *index);
double buffer_get_double32(const uint8_t *buffer, double scale, int32_t *index);
double buffer_get_double64(const uint8_t *buffer, double scale, int32_t *index);
double buffer_get_double32_auto(const uint8_t *buffer, int32_t *index);
double map(double x, double in_min, double in_max, double out_min, double out_max);
void llhToXyz(double lat, double lon, double height, double *x, double *y, double *z);
void xyzToLlh(double x, double y, double z, double *lat, double *lon, double *height);
void createEnuMatrix(double lat, double lon, double *enuMat);
void llhToEnu(const double *iLlh, const double *llh, double *xyz);
void enuToLlh(const double *iLlh, const double *xyz, double *llh);
double logn(double base, double number);
double twoNorm(double *vector, int vector_len);

}

#endif /* BUFFER_H_ */
