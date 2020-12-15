#ifndef	GEOFENCE_H
#define	GEOFENCE_H

#include <iostream>
#include <vector>

#include "util.h"

class Geofence
{
public:
    std::vector<CartesianPosition> polygonPoints;
    std::string name = "";
    bool isPermitted = false;
    double minHeight = 0;
    double maxHeight = 0;

};

#endif
