#ifndef	GEOFENCE_H
#define	GEOFENCE_H

#include <iostream>
#include <vector>

#include "util.h"
#include "positioning.h"

class Geofence
{
public:
    std::vector<CartesianPosition> polygonPoints;
    std::string name = "";
    bool isPermitted = false;
    double minHeight = 0;
    double maxHeight = 0;

	bool permits(const CartesianPosition &position) const;
	bool forbids(const CartesianPosition &position) const;
};

#endif
