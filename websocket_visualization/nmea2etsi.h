/*****************************************************/
/* nmea2etsi.h NMEA to ETSI CDD conversion functions */
/* Author: Henrik Eriksson                           */
/* SP Technical Research Institute of Sweden         */
/*****************************************************/

#ifndef NMEA2ETSI_H
#define NMEA2ETSI_H

//#include "poti.h"

uint32_t    ConvertLatitudeNMEAtoETSICDD(char *latitude, char *ns, char *stat);
uint32_t    ConvertLongitudeNMEAtoETSICDD(char *longitude, char *ew, char *stat);
uint16_t    ConvertSpeedValueNMEAtoETSICDD(char *spd, char *stat);
uint16_t    ConvertHeadingValueNMEAtoETSICDD(char *hdg, char *stat);
int32_t     ConvertAltitudeValueNMEAtoETSICDD(char *alt, char *stat);
uint64_t    ConvertTimestapItsNMEAtoETSICDD(char *time, char *date, char *stat);

#endif
