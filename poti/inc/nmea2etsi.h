/*****************************************************/
/* nmea2etsi.h NMEA to ETSI CDD conversion functions */
/* Author: Henrik Eriksson                           */
/* SP Technical Research Institute of Sweden         */
/*****************************************************/

#ifndef NMEA2ETSI_H
#define NMEA2ETSI_H

#include "poti.h"

Latitude      ConvertLatitudeNMEAtoETSICDD(char *latitude, char *ns, char *stat);
Longitude     ConvertLongitudeNMEAtoETSICDD(char *longitude, char *ew, char *stat);
SpeedValue    ConvertSpeedValueNMEAtoETSICDD(char *spd, char *stat);
HeadingValue  ConvertHeadingValueNMEAtoETSICDD(char *hdg, char *stat);
AltitudeValue ConvertAltitudeValueNMEAtoETSICDD(char *alt, char *stat);
TimestampIts  ConvertTimestapItsNMEAtoETSICDD(char *time, char *date, char *stat);

#endif
