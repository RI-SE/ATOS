/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
/**
 * @file filehandler.cpp
 * @author Adam Eriksson (adam.eriksson@astazero.com)
 * @brief atm Reads trajectory files in .ATOS (ATOS home folder)
 * @version 0.1
 * @date 2020-10-16
 * 
 * @copyright Copyright (c) 2020 
 * 
 */

#include "trajfilehandler.hpp"

#define MONR_BUFFER_LENGTH 1024
/**
 * @brief 
 * 
 * @param line 
 * @param buffer 
 * @return int 
 */
int parseTraj(std::string line,std::vector<char>& buffer)
{
    struct timeval relTime;
    CartesianPosition position;
    SpeedType speed;
    AccelerationType acceleration;
    TrajectoryFileHeader fileHeader;
    TrajectoryFileLine fileLine;

    double curvature = 0;
    std::stringstream ss (line);
    std::string segment;
    getline(ss,segment,';');
    ssize_t printedBytes;
    int debug = 0;

    memset(&fileHeader, 0, sizeof (fileHeader));
    memset(&fileLine, 0, sizeof (fileLine));

    std::vector<char> tmpvector(MONR_BUFFER_LENGTH);
    std::vector<char> cstr(line.c_str(), line.c_str() + line.size() + 1);

    if(segment.compare("TRAJECTORY") == 0){

        UtilParseTrajectoryFileHeader(cstr.data(),&fileHeader);
        if ((printedBytes = encodeTRAJMessageHeader(fileHeader.ID > UINT16_MAX ? 0 : (uint16_t) fileHeader.ID,
                                                fileHeader.majorVersion, fileHeader.name,
                                                strlen(fileHeader.name), fileHeader.numberOfLines,
                                                tmpvector.data(), tmpvector.size(), debug)) == -1) {
        LogMessage(LOG_LEVEL_ERROR, "Unable to encode trajectory message");

        return -1;
        }


    }
    else if (segment.compare("LINE") == 0){


        UtilParseTrajectoryFileLine(cstr.data(), &fileLine);

        relTime.tv_sec = (time_t) fileLine.time;
        relTime.tv_usec = (time_t) ((fileLine.time - relTime.tv_sec) * 1000000);
        position.xCoord_m = fileLine.xCoord;
        position.yCoord_m = fileLine.yCoord;
        position.isPositionValid = fileLine.zCoord != NULL;
        position.zCoord_m = position.isPositionValid ? *fileLine.zCoord : 0;
        position.heading_rad = fileLine.heading;
        position.isHeadingValid = true;
        speed.isLongitudinalValid = fileLine.longitudinalVelocity != NULL;
        speed.isLateralValid = fileLine.lateralVelocity != NULL;
        speed.longitudinal_m_s = fileLine.longitudinalVelocity != NULL ? *fileLine.longitudinalVelocity : 0;
        speed.lateral_m_s = fileLine.lateralVelocity != NULL ? *fileLine.lateralVelocity : 0;
        acceleration.isLongitudinalValid = fileLine.longitudinalAcceleration != NULL;
        acceleration.isLateralValid = fileLine.lateralAcceleration != NULL;
        acceleration.longitudinal_m_s2 =
            fileLine.longitudinalAcceleration != NULL ? *fileLine.longitudinalAcceleration : 0;

        acceleration.lateral_m_s2 = fileLine.lateralAcceleration != NULL ? *fileLine.lateralAcceleration : 0;
        if ((printedBytes = encodeTRAJMessagePoint(&relTime, position, speed, acceleration,
                                                   (float)fileLine.curvature, tmpvector.data(),
                                                   tmpvector.size(), debug)) == -1) {
            return -1;
        }
    }
    else if(segment.compare("ENDTRAJECTORY")== 0){

        if((printedBytes = encodeTRAJMessageFooter(tmpvector.data(), tmpvector.size(), debug))==-1){
            return -1;

        }

    }
    else{

        UtilParseTrajectoryFileLine(cstr.data(),&fileLine);


        relTime.tv_sec = (time_t) fileLine.time;
        relTime.tv_usec = (time_t) ((fileLine.time - relTime.tv_sec) * 1000000);
        position.xCoord_m = fileLine.xCoord;
        position.yCoord_m = fileLine.yCoord;
        position.isPositionValid = fileLine.zCoord != NULL;
        position.zCoord_m = position.isPositionValid ? *fileLine.zCoord : 0;
        position.heading_rad = fileLine.heading;
        position.isHeadingValid = true;
        speed.isLongitudinalValid = fileLine.longitudinalVelocity != NULL;
        speed.isLateralValid = fileLine.lateralVelocity != NULL;
        speed.longitudinal_m_s = fileLine.longitudinalVelocity != NULL ? *fileLine.longitudinalVelocity : 0;
        speed.lateral_m_s = fileLine.lateralVelocity != NULL ? *fileLine.lateralVelocity : 0;
        acceleration.isLongitudinalValid = fileLine.longitudinalAcceleration != NULL;
        acceleration.isLateralValid = fileLine.lateralAcceleration != NULL;
        acceleration.longitudinal_m_s2 =
            fileLine.longitudinalAcceleration != NULL ? *fileLine.longitudinalAcceleration : 0;

        acceleration.lateral_m_s2 = fileLine.lateralAcceleration != NULL ? *fileLine.lateralAcceleration : 0;
        if ((printedBytes = encodeTRAJMessagePoint(&relTime, position, speed, acceleration,
                                                   (float)fileLine.curvature, tmpvector.data(),
                                                   tmpvector.size(), debug)) == -1) {
            return -1;
        }
    }
    tmpvector.resize(printedBytes);
    for (auto val : tmpvector) buffer.push_back(val);


    return printedBytes;


}
