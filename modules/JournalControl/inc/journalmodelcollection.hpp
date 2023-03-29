/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef JOURNALMODELCOLLECTION_HPP
#define JOURNALMODELCOLLECTION_HPP

#include <unordered_set>
#include <chrono>
#include <string>
#include <vector>
#include "journalmodel.hpp"
#include "loggable.hpp"

// Add a C++20 type
namespace std::chrono {
	typedef duration<int64_t, ratio<60*60*24>> days;
}

class JournalModelCollection : public std::unordered_set<JournalModel>, public Loggable {
public:
	JournalModelCollection(rclcpp::Logger log) : Loggable(log) {}
	void placeStartBookmarks();
	void placeStopBookmarks();
	void insertNonBookmarked();
	int dumpToFile();
	std::string toString() const {
		std::string retval = "";
		for (const auto &journal : *this) {
			retval += journal.toString() + "\n";
		}
		if (this->size() > 0) {
			retval.pop_back();
		}
		return retval;
	}
private:
	std::chrono::time_point<std::chrono::system_clock, std::chrono::days> startDay;
	std::chrono::time_point<std::chrono::system_clock, std::chrono::days> stopDay;


    static std::string getDateAsString(const std::chrono::system_clock::time_point &date);
    static int printJournalHeaderTo(std::ofstream &ostrm);
    static std::string getCurrentDateAsString();
    static std::vector<fs::path> getJournalFilesFrom(const std::chrono::system_clock::time_point &date);
    static std::vector<fs::path> getJournalFilesFromToday() {
	    return getJournalFilesFrom(std::chrono::system_clock::now());
    }
    static int printFilesTo(const fs::path &inputDirectory, std::ostream &outputFile);
};

#endif // JOURNALMODELCOLLECTION_HPP