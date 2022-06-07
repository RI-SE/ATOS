#ifndef JOURNAL_HPP
#define JOURNAL_HPP

#if __GNUC__ > 8 || (__GNUC__ == 8 && __GNUC_MINOR__ >= 1)
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

#include <set>
#include "loggable.hpp"

class Journal : public Loggable {
public:
	class Bookmark : public Loggable {
	private:
		fs::path filePath;
		std::streampos filePosition;
	public:
		Bookmark(rclcpp::Logger log) : Loggable(log) {}
		bool valid = false;
		void place(const fs::path &path, const bool placeAtBeginning=false);
		const std::streampos& getPosition() const { return filePosition; }
		const fs::path& getFilePath() const { return filePath; }
		std::string toString() const {
			return valid ? getFilePath().string() + " @" + std::to_string(getPosition())
						 : "<unset>";
		}
	};

	Journal(rclcpp::Logger log) : Loggable(log), startReference(log), stopReference(log) {}
	std::string moduleName;
	mutable Bookmark startReference;
	mutable Bookmark stopReference;
	mutable std::set<fs::path> containedFiles;

	bool operator==(const Journal &other) const {
		return this->moduleName == other.moduleName;
	}

	std::string toString() const;
};

//! Template specialization of std::hash for Journal
namespace std {
	template <> struct hash<Journal> {
		size_t operator() (const Journal &journal) const {
			return std::hash<std::string>{}(journal.moduleName);
		}
	};
}

#endif // JOURNAL_HPP