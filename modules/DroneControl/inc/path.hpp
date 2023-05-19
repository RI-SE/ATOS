#pragma once

#include <vector>
#include <string>
#include <iterator>
#include "tinyxml2.h"
#include <iostream>
#include <list>
#include <string>

namespace ABD
{
    class Segment
    {
    public:
        ~Segment() = default;
        std::string description;
        float x;
        float y;
        float z;
        float time;
        float velocity;
        int heading;
    };

    class Path
    {
    public:
        Path(const std::string &path);
        ~Path() = default;
        float OriginLatitiude;
        float OriginLongitude;
        float OriginAltitude;
        float bearing;
        std::string path_to_file;
        void getPath(void);
        std::list<Segment> offsetPath(float xOffset, float yOffset, float zOffset);
        std::list<Segment> segmentsList;
        std::list<Segment> dronePath;
    };

}
