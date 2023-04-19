#pragma once

#include <vector>
#include <string>
#include <iterator>
#include "tinyxml2.h"
#include <iostream>
#include <iterator>
#include <list>
#include <string>

namespace ABD
{
    class Segment
    {
    public:
        Segment();
        ~Segment() = default;
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
        void get_path(void);
        void get_traj(float x_offset, float y_offset, float z_offset);
        std::list<Segment> segmentsList;
        std::list<Segment> dronePath;
    };

}
