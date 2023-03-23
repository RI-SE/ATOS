#pragma once

#include <filesystem>
#include <vector>
#include <map>


namespace Util {

  enum DirectoryPath {
    CONF,
    GEOFENCE,
    JOURNAL,
    TRAJ,
    OBJECTS,
    ODR,
    OSC
  };

  enum FilePath {
    TEST,
    PARAMS
  };

  std::map<DirectoryPath, std::string> expectedDirs = {
    {CONF, "conf"},
    {GEOFENCE, "geofence"},
    {JOURNAL, "journal"},
    {TRAJ, "traj"},
    {OBJECTS, "objects"},
    {ODR, "odr"},
    {OSC, "osc"}
  };

  std::map<FilePath, std::string> expectedFiles = {
    {TEST, "test.conf"},
    {PARAMS, "params.yaml"}
  };


  void verifyTestDirectory(const std::string& installationPath);
  std::filesystem::path getDirectoryPath(const enum DirectoryPath directoryName);
  std::filesystem::path getTestDirectoryPath();
  void deleteFile(const std::string& filePath);
  void deleteFile(const std::string& fileName, const enum DirectoryPath directory);
  void deleteFiles(const enum DirectoryPath directory);

} // namespace Util
