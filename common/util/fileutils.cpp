#include "fileutils.hpp"

namespace Util {

  int verifyTestDirectory(const std::string& installationPath) {
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

		return 0;
  }




} // namespace Utils