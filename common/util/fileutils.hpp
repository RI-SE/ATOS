#pragma once

#include <filesystem>
#include <vector>
#include <map>


namespace Util {

  enum DirectoryPath {
    CONF_DIR_NAME,
    GEOFENCE_DIR_NAME,
    JOURNAL_DIR_NAME,
    TRAJ_DIR_NAME,
    OBJECTS_DIR_NAME,
    ODR_DIR_NAME,
    OSC_DIR_NAME
  };

  enum FilePath {
    TEST_FILE_NAME,
    PARAMS_YAML_FILE_NAME
  };


  void verifyTestDirectory(const std::string& installationPath);
  std::filesystem::path getDirectoryPath(const enum DirectoryPath directoryName);
  std::filesystem::path getTestDirectoryPath();
  void deleteFile(const std::string& filePath);
  void deleteFile(const std::string& fileName, const enum DirectoryPath directory);
  void deleteFiles(const enum DirectoryPath directory);

} // namespace Util
