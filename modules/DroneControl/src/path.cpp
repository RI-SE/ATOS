#include "path.hpp"

namespace ABD
{

    Path::Path(const std::string &path)
    {
        // Todo construct path using a filepath to a .path file by calling getPath()
        path_to_file = path;
    }

    // This function writes to the path's segments list, returns nothing
    void Path::getPath(void)
    {
        // load xml doc based on path argument
        tinyxml2::XMLDocument doc;
        doc.LoadFile(path_to_file.c_str());

        // get root element ("Path" in this case)
        tinyxml2::XMLNode *root = doc.RootElement();
        if (root == NULL)
        {
            return;
        }

        // get OriginDatum element
        tinyxml2::XMLNode *origin = root->FirstChildElement()->FirstChildElement();

        // assign origin values from OriginDatum attributes
        const tinyxml2::XMLAttribute *attribute = origin->ToElement()->FirstAttribute();

        // gathering each attribute by steping through the header
        // doing it this way may present issues for other xml files with different
        OriginLatitiude = std::stof(attribute->Value(), NULL);
        attribute = attribute->Next();
        OriginLongitude = std::stof(attribute->Value(), NULL);
        attribute = attribute->Next();
        OriginAltitude = std::stof(attribute->Value(), NULL);
        attribute = attribute->Next();
        bearing = std::stof(attribute->Value(), NULL);

        tinyxml2::XMLNode *segmentStart = origin->Parent()->NextSibling()->FirstChild();


        while (segmentStart)
        { // iterate through all waypoints in the .path file

            Segment segment;

            // get segment description
            const tinyxml2::XMLAttribute *description =
                segmentStart->ToElement()->FirstAttribute();

            segment.description = description->Value();

            // get start point xyz values
            const tinyxml2::XMLAttribute *startpoint =
                segmentStart->FirstChild()->ToElement()->FirstAttribute();

            segment.x = std::stof(startpoint->Value(), NULL);
            startpoint = startpoint->Next();
            segment.y = std::stof(startpoint->Value(), NULL);
            segment.z = 0;

            // get time
            startpoint = startpoint->Next();
            segment.time = std::stof(startpoint->Value(), NULL);

            // get velocity
            startpoint = startpoint->Next();
            segment.velocity = std::stof(startpoint->Value(), NULL);

            // get heading
            startpoint = startpoint->Next();
            segment.heading = std::stoi(startpoint->Value(), NULL);

            // add to list
            segmentsList.push_back(segment);

            // move to next segment
            segmentStart = segmentStart->NextSibling();
        }

    }

    // This function returns a list of segments with the specified offset from the paths segment list
    std::list<Segment> Path::offsetPath(double xOffset, double yOffset, double zOffset) 
    {
        // This will change to Path::drone path to fly above the segments list
        std::list<Segment> newPath; 
        std::list<Segment>::iterator it;
        for (it = segmentsList.begin(); it != segmentsList.end(); ++it) {
            Segment s;
            s.description = "Drone Path: " + it->description;
            s.x = it->x + x_offset;
            s.y = it->y + y_offset;
            s.z = it->z + z_offset;
            s.time = it->time;
            s.velocity = it->velocity;
            s.heading = it->heading;
            newPath.push_back(s);
        }
        return newPath;
    }
}