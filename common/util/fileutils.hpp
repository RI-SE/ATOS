#pragma once

#include <filesystem>
#include <vector>


namespace Util {

  const std::vector<std::string> excpectedDirs = {
    "conf",
    "geofence",
    "journal",
    "traj",
    "objects",
    "odr",
    "osc"
  };

  const std::vector<std::string> expectedFiles = {
    "test.conf",
    "params.yaml"
  };

  void verifyTestDirectory(const std::string& installationPath);
  std::filesystem::path getDirectoryPath(const std::string& directoryName);
  std::filesystem::path getTestDirectoryPath();
  int deleteFile(const std::string& filePath);
  int deleteFile(const std::string& fileName, const std::string& directory);
  int deleteFiles(const std::string& directory);

} // namespace Util
