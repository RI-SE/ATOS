/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "journalmodel.hpp"
#include <fstream>
#include <stdexcept>

std::string JournalModel::toString() const {
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

void JournalModel::Bookmark::place(const fs::path &path, const bool placeAtBeginning) {
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