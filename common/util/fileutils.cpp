#include "fileutils.hpp"

namespace Util {

	/**
	 * @brief Checks so that all the required directories exist and that all configuration files exists.
	 * 
	 * @param installationPath Path to the installation directory.
	 */
  void verifyTestDirectory(const std::string& installationPath) {
    const std::filesystem::path homeDir = getenv("HOME");
    const std::filesystem::path atosDir = homeDir / std::filesystem::path(".astazero/ATOS");

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
	 * @return std::string Absolute path to the directory.
	 */
  std::string getDirectoryPath(const std::string& directoryName) {
		const std::string homeDir = getenv("HOME");
		const std::string atosDir = homeDir + "/.astazero/ATOS";
		return atosDir + "/" + directoryName + "/";
	}
} // namespace Utils