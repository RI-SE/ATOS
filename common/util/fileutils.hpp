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

} // namespace Util











// // File system functions
// int UtilDeleteTrajectoryFiles(void);
// int UtilDeleteGeofenceFiles(void);
// int UtilDeleteObjectFiles(void);

// int UtilDeleteTrajectoryFile(const char * geofencePath, const size_t nameLen);
// int UtilDeleteGeofenceFile(const char * geofencePath, const size_t nameLen);
// int UtilDeleteObjectFile(const char * geofencePath, const size_t nameLen);
// int UtilDeleteGenericFile(const char * genericFilePath, const size_t nameLen);

// // File parsing functions
// int UtilCheckTrajectoryFileFormat(const char *path, size_t pathLen);
// int UtilParseTrajectoryFileHeader(char *headerLine, TrajectoryFileHeader * header);
// int UtilParseTrajectoryFileFooter(char *footerLine);
// int UtilParseTrajectoryFileLine(char *fileLine, TrajectoryFileLine * line);