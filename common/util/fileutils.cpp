#include <iostream>
#include "fileutils.hpp"

namespace Util {

	/**
	 * @brief Checks so that all the required directories exist and that all configuration files exists.
	 * 
	 * @param installationPath Path to the installation directory.
	 */
  void verifyTestDirectory(const std::string& installationPath) {
		const auto atosDir = getTestDirectoryPath();

		// check that all expected directories exists
    if (!std::filesystem::is_directory(atosDir)) {
      std::filesystem::create_directories(atosDir);
    }

		for (auto expectedDir : excpectedDirs) {
			auto dir = atosDir / std::filesystem::path(expectedDir);
			if (!std::filesystem::is_directory(dir)) {
				std::filesystem::create_directories(dir);
			}
		}

		// check that that all expected files exists
		for (auto expectedFile : expectedFiles) {
			auto sysconfFile = std::filesystem::path(installationPath + "/etc/" + expectedFile);
			auto filePath = atosDir / std::filesystem::path("conf/" + expectedFile);
			if (!std::filesystem::exists(filePath)) {
				std::filesystem::copy(sysconfFile, filePath);
			}
		}
  }


	/**
	 * @brief Fetches the absolute path to where a directory is stored, ending with a forward slash.
	 * 
	 * @param directoryName The name of the directory to fetch.
	 * @return std::filesystem::path Absolute path to the directory.
	 */
  std::filesystem::path getDirectoryPath(const std::string& directoryName) {
		const auto atosDir = getTestDirectoryPath();
		return atosDir / std::filesystem::path(directoryName);
	}


	/**
	 * @brief Fetches the absolute path to the test directory.
	 * 
	 * @return std::filesystem::path Absolute path of the test directory.
	 */
	std::filesystem::path getTestDirectoryPath() {
		const std::string homeDir = getenv("HOME");
		const std::string atosDir = homeDir + "/.astazero/ATOS";
		return std::filesystem::path(atosDir);
	}


	/**
	 * @brief Deletes a file from the specified file path relative to the ATOS directory.
	 * 
	 * @param filePath The file path to the file to delete.
	 * @return int 0 if successful, -1 otherwise.
	 */
	int deleteFile(const std::string& filePath) {
		if (filePath == "") {
			std::cerr << "Attempt to call delete on unspecified file name\n";
			return -1;
		}
		else if (filePath == "..") {
			std::cerr << "Attempt to call delete on file and navigate out of directory\n";
		}

		auto dirPath = getTestDirectoryPath();
		auto file = dirPath / std::filesystem::path(filePath);
		std::filesystem::remove(file);

		return 0;
	}


	/**
	 * @brief Deletes a speciefied file in a directory.
	 * 
	 * @param fileName The name of the file to delete.
	 * @param directory The name of the directory of which the file is located.
	 * @return int 0 if successful, -1 otherwise.
	 */
	int deleteFile(const std::string& fileName, const std::string& directory) {
		if (fileName == "") {
			std::cerr << "Attempt to call delete on unspecified file name\n";
			return -1;
		}
		else if (fileName == "..") {
			std::cerr << "Attempt to call delete on file and navigate out of directory\n";
		}

		auto dirPath = getDirectoryPath(directory);
		auto file = dirPath / std::filesystem::path(fileName);
		std::filesystem::remove(file);

		return 0;
	}


	/**
	 * @brief Delete all files in a directory.
	 * 
	 * @param directory The files in the directory to delete.
	 */
	int deleteFiles(const std::string& directory) {
		auto dirPath = std::filesystem::path(getDirectoryPath(directory));
		for (auto& file : std::filesystem::directory_iterator(dirPath)) {
			try {
				std::filesystem::remove(file);
			}
			catch (std::filesystem::filesystem_error& e) {
				return -1;
			}
		}
		return 0;
	}
} // namespace Utils