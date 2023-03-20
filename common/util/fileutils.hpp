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



  int verifyTestDirectory(const std::string& installationPath);


  // int copyFile(const std::string& source, const std::string& destination);
  // void getDirectoryPath(const std::string& path);
  // int deleteFiles(const std::string& path);

} // namespace Util











// // File system functions
// int UtilVerifyTestDirectory(const char* installationPath);
// int UtilCopyFile(const char* source, const size_t sourceLen, const char* dest, const size_t destLen);
// void UtilGetTestDirectoryPath(char* path, size_t pathLen);
// void UtilGetJournalDirectoryPath(char* path, size_t pathLen);
// void UtilGetConfDirectoryPath(char* path, size_t pathLen);
// void UtilGetTrajDirectoryPath(char* path, size_t pathLen);
// void UtilGetOdrDirectoryPath(char* path, size_t pathLen);
// void UtilGetOscDirectoryPath(char* path, size_t pathLen);
// void UtilGetGeofenceDirectoryPath(char* path, size_t pathLen);
// void UtilGetObjectDirectoryPath(char* path, size_t pathLen);

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