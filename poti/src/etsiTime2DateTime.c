#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#define __STDC_FORMAT_MACROS
#include <inttypes.h>

void ConvertETSITimestapItstoDateTime(int64_t etsitime){

  unsigned int yy = 0;
  unsigned int mm = 0;
  unsigned int dd = 1;
  unsigned int h  = 0;
  unsigned int m  = 0;
  unsigned int s  = 0;
  unsigned int ms = 0;

  //Subtract extra time for leap years and leap seconds up until 2050
  if (etsitime >= 1393632004000) //Leap year 2048
    etsitime -= (int64_t) 24 * 3600 * 1000;
  if (etsitime >= 1267401604000) //Leap year 2044
    etsitime -= (int64_t) 24 * 3600 * 1000;
  if (etsitime >= 1141171204000) //Leap year 2040
    etsitime -= (int64_t) 24 * 3600 * 1000;
  if (etsitime >= 1014940804000) //Leap year 2036
    etsitime -= (int64_t) 24 * 3600 * 1000;
  if (etsitime >= 888710404000) //Leap year 2032
    etsitime -= (int64_t) 24 * 3600 * 1000;
  if (etsitime >= 762480004000) //Leap year 2028
    etsitime -= (int64_t) 24 * 3600 * 1000;
  if (etsitime >= 636249604000) //Leap year 2024
    etsitime -= (int64_t) 24 * 3600 * 1000;
  if (etsitime >= 510019204000) //Leap year 2020
    etsitime -= (int64_t) 24 * 3600 * 1000;
  if (etsitime >= 383788804000) //Leap year 2016
    etsitime -= (int64_t) 24 * 3600 * 1000;
  if (etsitime >= 362793603000) //Leap second 2015-07-01:00.00.00.000
    etsitime -= (int64_t) 1000;
  if (etsitime >= 268185602000) //Leap second 2012-07-01:00.00.00.000
    etsitime -= (int64_t) 1000;
  if (etsitime >= 257558402000) //Leap year 2012
    etsitime -= (int64_t) 24 * 3600 * 1000;
  if (etsitime >= 157852801000) //Leap second 2008-01-01:00.00.00.000
    etsitime -= (int64_t) 1000;
  if (etsitime >= 131328001000) //Leap year 2008
    etsitime -= (int64_t) 24 * 3600 * 1000;
  if (etsitime >= 63158400000) //Leap second 2006-01-01:00.00.00.000
    etsitime -= (int64_t) 1000;
  if (etsitime >= 5097600000) //Leap year 2004
    etsitime -= (int64_t) 24 * 3600 * 1000;

  //Find the number of years since 2004-01-01
  while(etsitime >= 31536000000){
    yy++;
    etsitime -= 31536000000;
  }

  //Find the month
  //December
  if (etsitime >= 28857600000){
    mm = 12;
    etsitime -= 28857600000;
  }
  //November
  else if (etsitime >= 26265600000){
    mm = 11;
    etsitime -= 26265600000;
  }
  //October
  else if (etsitime >= 23587200000){
    mm = 10;
    etsitime -= 23587200000;
  }
  //September
  else if (etsitime >= 20995200000){
    mm = 9;
    etsitime -= 20995200000;
  }
  //August
  else if (etsitime >= 18316800000){
    mm = 8;
    etsitime -= 18316800000;
  }
  //July
  else if (etsitime >= 15638400000){
    mm = 7;
    etsitime -= 15638400000;
  }
  //June
  else if (etsitime >= 13046400000){
    mm = 6;
    etsitime -= 13046400000;
  }
  //May
  else if (etsitime >= 10368000000){
    mm = 5;
    etsitime -= 10368000000;
  }
  //April
  else if (etsitime >= 7776000000){
    mm = 4;
    etsitime -= 7776000000;
  }
  //March
  else if (etsitime >= 5097600000){
    mm = 3;
    etsitime -= 5097600000;
  }
  //February
  else if (etsitime >= 2678400000){
    mm = 2;
    etsitime -= 2678400000;
  }
  //January
  else{
    mm = 1;
  }

  //Find the number of days
  while(etsitime >= 86400000){
    dd++;
    etsitime -= 86400000;
  }
  //Find the number of hours
  while(etsitime >= 3600000){
    h++;
    etsitime -= 3600000;
  }
  //Find the number of minutes
  while(etsitime >= 60000){
    m++;
    etsitime -= 60000;
  }
  //Find the number of seconds
  while(etsitime >= 1000){
    s++;
    etsitime -= 1000;
  }
  //Print the date-time
  printf("%d-%02d-%02d %02d%02d%02d.%03d\n", yy+2004, mm, dd, h, m, s,(int) etsitime);
}

int main(int argc, char *argv[]){

  if (argc != 2){
    printf("Usage Convert <number>\n");
    return 1;
  }
  ConvertETSITimestapItstoDateTime(strtoull(argv[1], NULL, 10));
  return 0;}
