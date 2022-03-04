#include "journal.hpp"
#include "logging.h"
#include <fstream>

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
        LogMessage(LOG_LEVEL_DEBUG, "Storing bookmark to file %s", path.c_str());
        filePosition = istrm.tellg();
        filePath = path;
        valid = true;
        istrm.close();
    }
    else {
        LogMessage(LOG_LEVEL_ERROR, "Unable to open file %s", path.c_str());
    }
}