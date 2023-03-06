# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

cmake_minimum_required(VERSION 3.10)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_VERBOSE_MAKEFILE OFF CACHE BOOL "Enable verbose make")
set(CMAKE_COLOR_MAKEFILE   ON)
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
project(atos VERSION 1.0.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	#add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# point out where CMake scripts are located
set(PROJECT_CMAKE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")
set(UTIL_CMAKE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/util/cmake")
list(APPEND CMAKE_MODULE_PATH "${PROJECT_CMAKE_ROOT}")
list(APPEND CMAKE_MODULE_PATH "${UTIL_CMAKE_ROOT}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Enable or disable modules
set(WITH_SUPERVISION OFF CACHE BOOL "Enable Supervision module")
set(WITH_VISUALIZATION OFF CACHE BOOL "Enable Visualization module")
set(WITH_OBJECT_CONTROL ON CACHE BOOL "Enable ObjectControl module")
set(WITH_DIRECT_CONTROL OFF CACHE BOOL "Enable DirectControl module")
set(WITH_BACK_TO_START OFF CACHE BOOL "Enable BackToStart module")
set(WITH_SYSTEM_CONTROL ON CACHE BOOL "Enable SystemControl module")
set(WITH_TIME_CONTROL OFF CACHE BOOL "Enable TimeControl module")
set(WITH_JOURNAL_CONTROL ON CACHE BOOL "Enable JournalControl module")
set(WITH_TRAJECTORYLET_STREAMER ON CACHE BOOL "Enable TrajectoryletStreamer module")
set(WITH_OSI_ADAPTER ON CACHE BOOL "Enable OSIAdapter module")
set(WITH_ESMINI_ADAPTER ON CACHE BOOL "Enable EsminiAdapter module")
set(WITH_MQTT_BRIDGE ON CACHE BOOL "Enable MQTTBridge module")

set(ENABLE_TESTS ON CACHE BOOL "Enable testing on build")

# Create list of all enabled modules
if(WITH_SUPERVISION)
        list(APPEND ENABLED_MODULES Supervision)
endif()
if(WITH_VISUALIZATION)
        list(APPEND ENABLED_MODULES Visualization)
endif()
if(WITH_DIRECT_CONTROL)
        list(APPEND ENABLED_MODULES DirectControl)
endif()
if(WITH_BACK_TO_START)
        list(APPEND ENABLED_MODULES BackToStart)
endif()
if(WITH_SYSTEM_CONTROL)
	list(APPEND ENABLED_MODULES SystemControl)
endif()
if(WITH_TIME_CONTROL)
	list(APPEND ENABLED_MODULES TimeControl)
endif()
if(WITH_OBJECT_CONTROL)
	list(APPEND ENABLED_MODULES ObjectControl)
endif()
if(WITH_JOURNAL_CONTROL)
	list(APPEND ENABLED_MODULES JournalControl)
endif()
if(WITH_TRAJECTORYLET_STREAMER)
        list(APPEND ENABLED_MODULES TrajectoryletStreamer)
endif()
if(WITH_OSI_ADAPTER)
        list(APPEND ENABLED_MODULES OSIAdapter)
endif()
if(WITH_ESMINI_ADAPTER)
        list(APPEND ENABLED_MODULES EsminiAdapter)
endif()
if(WITH_MQTT_BRIDGE)
        list(APPEND ENABLED_MODULES MQTTBridge)
endif()

# Add corresponding subprojects
add_subdirectory(iso22133)
add_subdirectory(common/time)
add_subdirectory(common/shmem)
add_subdirectory(common/osihandler)
add_subdirectory(common/sockets)
add_subdirectory(common)
add_subdirectory(modules/ATOSBase)

foreach(MODULE ${ENABLED_MODULES})
	add_subdirectory(modules/${MODULE})
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


# Install configuration
include(GNUInstallDirs)
install(DIRECTORY launch DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/)
file(GLOB CONF_FILES ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.conf)
file(GLOB PARAMS_FILE ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.yaml)
install(FILES ${CONF_FILES} ${PARAMS_FILE} DESTINATION ${CMAKE_INSTALL_SYSCONFDIR})

# Add post install instructions
add_subdirectory(cmake/cpack)
