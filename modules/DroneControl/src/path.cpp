#include "path.hpp"

namespace ABD
{

    Path::Path(const std::string &path)
    {
        // Todo construct path using a filepath to a .path file
        path_to_file = path;
    }

    void Path::getPath(void)
    {

        // load xml doc based on path argument
        tinyxml2::XMLDocument doc;
        doc.LoadFile(path_to_file.c_str());

        // get root element ("Path" in this case)
        tinyxml2::XMLNode *root = doc.RootElement();
        if (root == NULL)
        {
            //cout << "root in NULL, check the path" << endl;
            return;
        }

        // get OriginDatum element
        tinyxml2::XMLNode *origin = root->FirstChildElement()->FirstChildElement();
        //cout << origin->ToElement()->Name() << endl;

        // assign origin values from OriginDatum attributes
        const tinyxml2::XMLAttribute *attribute = origin->ToElement()->FirstAttribute();

        // const XMLAttribute *attribute = element->FindAttribute("OriginLatitiude");
        // trying to get search to work instead
        // not having search may present issues for other xml files with different
        // formats

        OriginLatitiude = std::stof(attribute->Value(), NULL);
        //cout << attribute->Name() << ": " << OriginLatitiude << endl;

        attribute = attribute->Next();
        OriginLongitude = std::stof(attribute->Value(), NULL);
        //cout << attribute->Name() << ": " << OriginLongitude << endl;

        attribute = attribute->Next();
        OriginAltitude = std::stof(attribute->Value(), NULL);
        //cout << attribute->Name() << ": " << OriginAltitude << endl;I

        attribute = attribute->Next();
        bearing = std::stof(attribute->Value(), NULL);
        //cout << attribute->Name() << ": " << bearing << endl;

        tinyxml2::XMLNode *segmentStart = origin->Parent()->NextSibling()->FirstChild();


        while (segmentStart)
        { // while loop for all waypoints

            Segment segment;

            // get segment description
            const tinyxml2::XMLAttribute *description =
                segmentStart->ToElement()->FirstAttribute();

            segment.description = description->Value();

            //cout << description->Name() << ": " << segment.description << endl;

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

    /*
    Segment::Segment()
    {
        // Todo construct segment
    }
    */

}