#ifndef JOURNALCOLLECTION_HPP
#define JOURNALCOLLECTION_HPP

#include <unordered_set>
#include <chrono>
#include <string>
#include <vector>
#include "journal.hpp"
#include "loggable.hpp"

// Add a C++20 type
namespace std::chrono {
	typedef duration<int64_t, ratio<60*60*24>> days;
}

class JournalCollection : public std::unordered_set<Journal>, public Loggable {
public:
	JournalCollection(rclcpp::Logger log) : Loggable(log) {}
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

#endif // JOURNALCOLLECTION_HPP