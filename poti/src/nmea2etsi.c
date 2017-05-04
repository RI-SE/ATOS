/*****************************************************/
/* nmea2etsi.c NMEA to ETSI CDD conversion functions */
/* Author: Henrik Eriksson                           */
/* SP Technical Research Institute of Sweden         */
/*****************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include "nmea2etsi.h"

Latitude ConvertLatitudeNMEAtoETSICDD(char *lat, char *dir, char *stat){
  int    latitude     = 900000001; //Init latitude to: latitude unavailable
  char  *dummy_ptr    = NULL;      //Dummy pointer for strtod function
  double deg          = 0.0;       //ddmm.mmmm format for latitude
  int    deg_whole    = 0;         //Whole part of dd.ddddd format for latitude
  double deg_fraction = 0.0;       //Fractional part of dd.ddddd format for latitude

  if(!strcmp(stat,"A")){
    //Convert latitude string to ddmm.mmmm floating-point format
    deg = strtod(lat, &dummy_ptr);
    
    //Get whole part of dd.dddddd format
    deg_whole = (int) deg / 100;
    
    //Get fractional part of dd.ddddd format
    deg_fraction = deg - (100 * deg_whole);
    deg_fraction /= 60;
    
    //Put on ETSI CDD dd.ddddd format
    latitude = (deg_whole + deg_fraction) * 10000000;
    
    //Change sign if southern hemisphere
    if ((!strcmp(dir, "s")) || (!strcmp(dir, "S"))){
      latitude *= -1;}
  }
  return latitude;
}

Longitude ConvertLongitudeNMEAtoETSICDD(char *longit, char *dir, char *stat){
  int    longitude    = 1800000001; //Init longitude to: longitude unavailable
  char  *dummy_ptr    = NULL;       //Dummy pointer for strtod function
  double deg          = 0.0;        //ddmm.mmmm format for longitude
  int    deg_whole    = 0;          //Whole part of dd.ddddd format for longitude
  double deg_fraction = 0.0;        //Fractional part of dd.ddddd format for longitude

  if(!strcmp(stat,"A")){
    //Convert longitude string to ddmm.mmmm floating-point format
    deg = strtod(longit, &dummy_ptr);
    
    //Get whole part of dd.dddddd format
    deg_whole = (int) deg / 100;
    
    //Get fractional part of dd.ddddd format
    deg_fraction = deg - (100 * deg_whole);
    deg_fraction /= 60;
    
    //Put on ETSI CDD dd.ddddd format
    longitude = (deg_whole + deg_fraction) * 10000000;
    
    //Change sign if western hemisphere
    if ((!strcmp(dir, "w")) || (!strcmp(dir, "W"))){
      longitude *= -1;}
  }
  return longitude;
}

SpeedValue ConvertSpeedValueNMEAtoETSICDD(char *spd, char *stat){
  SpeedValue speed = SpeedValue_unavailable; //Init speed to: speed unavailable
  char *dummy_ptr  = NULL;                   //Dummy pointer for strtod function

  if(!strcmp(stat,"A"))
    speed = (SpeedValue) (0.5144444 * 100 * strtod(spd, &dummy_ptr));

  return speed;
}

HeadingValue ConvertHeadingValueNMEAtoETSICDD(char *hdg, char *stat) {
  HeadingValue heading = HeadingValue_unavailable; //Initialize to unavailable value
  char *dummy_ptr = NULL;                          //Dummy pointer for strtod function

  if(!strcmp(stat,"A"))
    heading = (HeadingValue) (10 * strtod(hdg, &dummy_ptr)); 

    return heading;
}

AltitudeValue ConvertAltitudeValueNMEAtoETSICDD(char *alt, char *stat) {
  AltitudeValue altitude = AltitudeValue_unavailable; //Initialize to unavailable value
  char *dummy_ptr = NULL;                             //Dummy pointer for strtod function

  if(!strcmp(stat,"A"))
    altitude = (AltitudeValue) (100 * strtod(alt, &dummy_ptr));
      
  //Set to max and min values if height is outside these bounds 
  if (altitude > 800000)
    altitude = 800000;
  if (altitude < -100000)
    altitude = -100000;
  
  return altitude;
}

TimestampIts ConvertTimestapItsNMEAtoETSICDD(char *time, char *date, char *stat){
  int64_t data           = 0;
  char   *ptr1           = NULL;
  char    year[3]        = "yy";
  char    month[3]       = "mm";
  char    day[3]         = "dd";
  char    hour[3]        = "hh";
  char    minute[3]      = "mm";
  char    second[3]      = "ss";
  char    centisecond[4] = "ss";

  if(!strcmp(stat,"A")){
    //Mask out string parts for year, month, day, hour, minute, second, and centisecond, respectively 
    strncpy(day, date, 2);
    strncpy(month, date+2, 2);
    strncpy(year, date+4, 2);
    strncpy(hour, time, 2);
    strncpy(minute, time+2, 2);
    strncpy(second, time+4, 2);
    strncpy(centisecond, time+7, 2);

    //Add the number of miiliseconds from 2004 to present year
    data = (int64_t) (((int64_t) strtol(year, &ptr1, 10) - 4) * 365 * 24 * 3600 * 1000);

    //Add the number of milliseconds from January to present month
    switch((char) strtol(month, &ptr1, 10)){
    case 1  :
      data += 0;
      break; 
    case 2  :
      data += (int64_t) 31 * 24 * 3600 * 1000;
      break; 
    case 3  :
      data += (int64_t) 59 * 24 * 3600 * 1000;
      break; 
    case 4  :
      data += (int64_t) 90 * 24 * 3600 * 1000;
      break; 
    case 5  :
      data += (int64_t) 120 * 24 * 3600 * 1000;
      break; 
    case 6  :
      data += (int64_t) 151 * 24 * 3600 * 1000;
      break; 
    case 7  :
      data += (int64_t) 181 * 24 * 3600 * 1000;
      break; 
    case 8  :
       data += (int64_t) 212 * 24 * 3600 * 1000;
       break; 
    case 9  :
      data += (int64_t) 243 * 24 * 3600 * 1000;
      break; 
    case 10  :
      data += (int64_t) 273 * 24 * 3600 * 1000;
      break; 
    case 11  :
      data += (int64_t) 304 * 24 * 3600 * 1000;
      break; 
    case 12  :
      data += (int64_t) 334 * 24 * 3600 * 1000;
      break; 
    default : 
      data = (int64_t) -4398046511103;}
    
    //Add extra time for leap years and leap seconds up until 2050
    if (data >= 5097600000) //Leap year 2004
      data += (int64_t) 24 * 3600 * 1000;
    if (data >= 63158400000) //Leap second 2006-01-01:00.00.00.000
      data += (int64_t) 1000;
    if (data >= 131328001000) //Leap year 2008
      data += (int64_t) 24 * 3600 * 1000;
    if (data >= 157852801000) //Leap second 2008-01-01:00.00.00.000
      data += (int64_t) 1000;
    if (data >= 257558402000) //Leap year 2012
    data += (int64_t) 24 * 3600 * 1000;
    if (data >= 268185602000) //Leap second 2012-07-01:00.00.00.000
      data += (int64_t) 1000;
    if (data >= 362793603000) //Leap second 2015-07-01:00.00.00.000
      data += (int64_t) 1000;
    if (data >= 383788804000) //Leap year 2016
      data += (int64_t) 24 * 3600 * 1000;
    if (data >= 410313604000) //Leap second 2017-01-01:00.00.00.000
      data += (int64_t) 1000;
    if (data >= 510019205000) //Leap year 2020
      data += (int64_t) 24 * 3600 * 1000;
    if (data >= 636249605000) //Leap year 2024
      data += (int64_t) 24 * 3600 * 1000;
    if (data >= 762480005000) //Leap year 2028
      data += (int64_t) 24 * 3600 * 1000;
    if (data >= 888710405000) //Leap year 2032
      data += (int64_t) 24 * 3600 * 1000;
    if (data >= 1014940805000) //Leap year 2036
      data += (int64_t) 24 * 3600 * 1000;
    if (data >= 1141171205000) //Leap year 2040
      data += (int64_t) 24 * 3600 * 1000;
    if (data >= 1267401605000) //Leap year 2044
      data += (int64_t) 24 * 3600 * 1000;
    if (data >= 1393632005000) //Leap year 2048
      data += (int64_t) 24 * 3600 * 1000;
    //Add the corresponding number of milliseconds for day, hour, minute, second, and millisecond, respectively
    data += (int64_t) (((int64_t) strtol(day, &ptr1, 10) - 1) * 24 * 3600 * 1000);
    data += (int64_t) (((int64_t) strtol(hour, &ptr1, 10)) * 3600 * 1000);
    data += (int64_t) (((int64_t) strtol(minute, &ptr1, 10)) * 60 * 1000);
    data += (int64_t) (((int64_t) strtol(second, &ptr1, 10)) * 1000);
    data += (int64_t) (((int64_t) strtol(centisecond, &ptr1, 10)) * 10);
  }
  return data;
}
