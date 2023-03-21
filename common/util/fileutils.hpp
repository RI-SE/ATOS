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
  std::string getDirectoryPath(const std::string& directoryName);
  std::string getTestDirectoryPath();
  int deleteFile(const std::string& filePath);
  int deleteFile(const std::string& fileName, const std::string& directory);
  int deleteFiles(const std::string& directory);

} // namespace Util












// // File parsing functions
// int UtilCheckTrajectoryFileFormat(const char *path, size_t pathLen);
// int UtilParseTrajectoryFileHeader(char *headerLine, TrajectoryFileHeader * header);
// int UtilParseTrajectoryFileFooter(char *footerLine);
// int UtilParseTrajectoryFileLine(char *fileLine, TrajectoryFileLine * line);