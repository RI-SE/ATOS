#include <iostream>
#include "fileutils.hpp"

namespace Util {

	/**
	 * @brief Map for expected directories.
	 * 
	 */
	std::map<DirectoryPath, std::string> expectedDirs = {
    {CONF_DIR_NAME, "conf"},
    {GEOFENCE_DIR_NAME, "geofence"},
    {JOURNAL_DIR_NAME, "journal"},
    {TRAJ_DIR_NAME, "traj"},
    {OBJECTS_DIR_NAME, "objects"},
    {ODR_DIR_NAME, "odr"},
    {OSC_DIR_NAME, "osc"}
  };

	/**
	 * @brief Map for expected file names.
	 * 
	 */
  std::map<FilePath, std::string> expectedFiles = {
    {TEST_FILE_NAME, "test.conf"},
    {PARAMS_YAML_FILE_NAME, "params.yaml"}
  };


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

		for (const auto& [key, expectedDir] : expectedDirs) {
			auto dir = atosDir / std::filesystem::path(expectedDir);
			if (!std::filesystem::is_directory(dir)) {
				std::filesystem::create_directories(dir);
			}
		}

		// check that that all expected files exists
		for (const auto& [key, expectedFile] : expectedFiles) {
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
  std::filesystem::path getDirectoryPath(const enum DirectoryPath directoryName) {
		const auto atosDir = getTestDirectoryPath();
		return atosDir / std::filesystem::path(expectedDirs[directoryName]);
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
	 */
	void deleteFile(const std::string& filePath) {
		if (filePath == "") {
			throw std::invalid_argument("Attempt to call delete on unspecified file name");
		}
		else if (filePath == "..") {
			throw std::invalid_argument("Attempt to call delete on file and navigate out of directory");
		}

		auto dirPath = getTestDirectoryPath();
		auto file = dirPath / std::filesystem::path(filePath);
		std::filesystem::remove(file);
	}


	/**
	 * @brief Deletes a speciefied file in a directory.
	 * 
	 * @param fileName The name of the file to delete.
	 * @param directory The name of the directory of which the file is located.
	 */
	void deleteFile(const std::string& fileName, const enum DirectoryPath directory) {
		if (fileName == "") {
			throw std::invalid_argument("Attempt to call delete on unspecified file name");
		}
		else if (fileName == "..") {
			throw std::invalid_argument("Attempt to call delete on file and navigate out of directory");
		}

		auto dirPath = getDirectoryPath(directory);
		auto file = dirPath / std::filesystem::path(fileName);
		std::filesystem::remove(file);
	}


	/**
	 * @brief Delete all files in a directory.
	 * 
	 * @param directory The files in the directory to delete.
	 */
	void deleteAllFilesInDirectory(const enum DirectoryPath directory) {
		auto dirPath = getDirectoryPath(directory);
		for (auto& file : std::filesystem::directory_iterator(dirPath)) {
				std::filesystem::remove(file);
		}
	}
} // namespace Utils