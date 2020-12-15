#ifndef	GEOFENCE_H
#define	GEOFENCE_H

#include <iostream>
#include <vector>
#include <regex>

#include "util.h"
#include "positioning.h"

class Geofence
{
public:

	Geofence() = default;
	~Geofence() { polygonPoints.clear(); }
	Geofence(const Geofence& other);

    std::vector<CartesianPosition> polygonPoints;
    std::string name = "";
    bool isPermitted = false;
    double minHeight = 0;
    double maxHeight = 0;

	bool permits(const CartesianPosition &position) const;
	bool forbids(const CartesianPosition &position) const;

	void initializeFromFile(const std::string& fileName);

private:
	static const std::regex fileHeaderPattern;
	static const std::regex fileLinePattern;
	static const std::regex fileFooterPattern;
};

#endif
