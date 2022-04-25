#include "journal.hpp"
#include "logging.h"
#include <fstream>
#include <stdexcept>

std::string Journal::toString() const {
    std::string retval = "";
    retval += moduleName + " journal\n\tFiles:\n";
    for (const auto &file : containedFiles) {
        retval += "\t\t* " + file.string() + "\n";
    }
    retval += "\tStart / end references:\n";
    retval += "\t\ts: " + startReference.toString() + "\n";
    retval += "\t\te: " + stopReference.toString();
    return retval;
}

void Journal::Bookmark::place(const fs::path &path, const bool placeAtBeginning) {
    std::ifstream istrm(path, placeAtBeginning ? std::ios_base::in
                                                : std::ios_base::ate);
    if (istrm.is_open()) {
        RCLCPP_DEBUG(get_logger(), "Storing bookmark to file %s", path.c_str());
        filePosition = istrm.tellg();
        filePath = path;
        valid = true;
        istrm.close();
    }
    else {
        throw std::runtime_error("Failed to open file " + path.string());
    }
}