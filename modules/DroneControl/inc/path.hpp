#pragma once

#include <vector>
#include <string>

namespace ABD 
{
    class Segment{
        public:
            Segment();
            ~Segment() = default;
    };

    class Path{
        public:
            Path(const std::string& path);
            ~Path() = default;

        private:
            std::vector<Segment> segments;
    };

}
