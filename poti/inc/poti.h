/*********************************************/
/* poti.h POTI header file                   */
/* Author: Henrik Eriksson                   */
/* SP Technical Research Institute of Sweden */
/*********************************************/

#ifndef POTI_H
#define POTI_H

#include <inttypes.h>

typedef int32_t Latitude;
#define Latitude_oneMicrodegreeNorth 10
#define Latitude_oneMicrodegreeSouth -10
#define Latitude_unavailable 900000001

typedef int32_t Longitude;
#define Longitude_oneMicrodegreeEast 10
#define Longitude_oneMicrodegreeWest -10
#define Longitude_unavailable 1800000001

typedef uint16_t SemiAxisLength;
#define SemiAxisLength_oneCentimeter 1
#define SemiAxisLength_outOfRange 4094
#define SemiAxisLength_unavailable 4095

typedef uint16_t HeadingValue;
#define wgs84North 0
#define wgs84East 900
#define wgs84South 1800
#define wgs84West 2700
#define HeadingValue_unavailable 3601

typedef struct PosConfidenceEllipse {
    SemiAxisLength  semiMajorConfidence;
    SemiAxisLength  semiMinorConfidence;
    HeadingValue    semiMajorOrientation;
} PosConfidenceEllipse;

typedef int32_t AltitudeValue;
#define referenceEllipsoidSurface 0
#define AltitudeValue_oneCentimeter 1
#define AltitudeValue_unavailable 800001

typedef enum AltitudeConfidence {
    alt_000_01 = 0,
    alt_000_02 = 1,
    alt_000_05 = 2,
    alt_000_10 = 3,
    alt_000_20 = 4,
    alt_000_50 = 5,
    alt_001_00 = 6,
    alt_002_00 = 7,
    alt_005_00 = 8,
    alt_010_00 = 9,
    alt_020_00 = 10,
    alt_050_00 = 11,
    alt_100_00 = 12,
    alt_200_00 = 13,
    AltitudeConfidence_outOfRange = 14,
    AltitudeConfidence_unavailable = 15
} AltitudeConfidence;

typedef struct Altitude {
    AltitudeValue   altitudeValue;
    AltitudeConfidence altitudeConfidence;
} Altitude;

typedef uint16_t  HeadingConfidence;
#define equalOrWithinZeroPointOneDegree 1
#define equalOrWithinOneDegree 10
#define HeadingConfidence_outOfRange 126
#define HeadingConfidence_unavailable 127

typedef struct Heading {
    HeadingValue    headingValue;
    HeadingConfidence headingConfidence;
} Heading;

typedef uint8_t PerformanceClass;
#define PerformanceClass_unavailable 0
#define performanceClassA 1
#define performanceClassB 2

typedef uint16_t SpeedValue;
#define standstill 0
#define oneCentimeterPerSec 1
#define SpeedValue_unavailable 16383

typedef uint8_t SpeedConfidence;
#define equalOrWithinOneCentimeterPerSec 1
#define equalOrWithinOneMeterPerSec 100
#define SpeedConfidence_outOfRange 126
#define SpeedConfidence_unavailable 127

typedef struct Speed {
    SpeedValue      speedValue;
    SpeedConfidence speedConfidence;
} Speed;

typedef uint64_t TimestampIts;
#define utcStartOf2004 0
#define oneMillisecAfterUTCStartOf2004 1

#endif
