cmake_minimum_required(VERSION 3.10)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_VERBOSE_MAKEFILE OFF CACHE BOOL "Enable verbose make")
set(CMAKE_COLOR_MAKEFILE   ON)

project(maestro VERSION 0.5.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	#add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# point out where CMake scripts are located
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/util/cmake)

#point out the ros install location for modules
set_property(GLOBAL PROPERTY ros_install_dir_property "lib/maestro")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(maestro_msgs REQUIRED)
find_package(Systemd REQUIRED)

# Enable or disable modules
set(WITH_SCENARIO_CONTROL ON CACHE BOOL "Enable ScenarioControl module")
set(WITH_SUPERVISION OFF CACHE BOOL "Enable Supervision module")
set(WITH_VISUALIZATION OFF CACHE BOOL "Enable Visualization module")
set(WITH_RELATIVE_KINEMATICS ON CACHE BOOL "Enable RelativeKinematics module")
set(WITH_DIRECT_CONTROL OFF CACHE BOOL "Enable DirectControl module")
set(WITH_OBJECT_MONITORING OFF CACHE BOOL "Enable ObjectMonitoring module")
set(WITH_BACK_TO_START OFF CACHE BOOL "Enable BackToStart module")
set(WITH_SYSTEM_CONTROL ON CACHE BOOL "Enable SystemControl module")
set(WITH_TIME_CONTROL ON CACHE BOOL "Enable TimeControl module")

set(ENABLE_TESTS ON CACHE BOOL "Enable testing on build")


##########WAS HERE BEFORE MERGE
#set(MAESTRO_TEST_DIR ".maestro" CACHE STRING "Directory where test data is to be stored")

#set(MAESTRO_JOURNAL_DIR "${MAESTRO_TEST_DIR}/journal")
#set(MAESTRO_TRAJ_DIR "${MAESTRO_TEST_DIR}/traj")
#set(MAESTRO_GEOFENCE_DIR "${MAESTRO_TEST_DIR}/geofence")
#set(MAESTRO_OBJECTS_DIR "${MAESTRO_TEST_DIR}/objects")


# Dependencies
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/util/cmake)
find_package(Systemd REQUIRED)

# Configure structure of output
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Create list of all enabled modules
if(WITH_SCENARIO_CONTROL)
        list(APPEND ENABLED_MODULES ScenarioControl)
endif()
if(WITH_SUPERVISION)
        list(APPEND ENABLED_MODULES Supervision)
endif()
if(WITH_VISUALIZATION)
        list(APPEND ENABLED_MODULES Visualization)
endif()
if(WITH_DIRECT_CONTROL)
        list(APPEND ENABLED_MODULES DirectControl)
endif()
if(WITH_OBJECT_MONITORING)
        list(APPEND ENABLED_MODULES ObjectMonitoring)
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
if(WITH_RELATIVE_KINEMATICS)
	list(APPEND ENABLED_MODULES RelativeKinematics)
	set(WITH_OBJECT_CONTROL OFF)
else()
	set(WITH_OBJECT_CONTROL ON)
endif()

# Add corresponding subprojects
add_subdirectory(util/C/time)
add_subdirectory(util/C/logging)
add_subdirectory(util/C/mqbus)
add_subdirectory(util/C/shmem)
add_subdirectory(util/C/iso22133)
add_subdirectory(util/C/mathutils)
add_subdirectory(util/C/osihandler)
add_subdirectory(common)
add_subdirectory(core)

foreach(MODULE ${ENABLED_MODULES})
	add_subdirectory(modules/${MODULE})
endforeach()

######WAS HERE BEFORE
# Ensure .maestro directory is created at build time in build directory
#add_custom_target(configurationDirectory ALL
#	COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/conf
#		${CMAKE_BINARY_DIR}/${MAESTRO_CONFIGURATION_DIR}
#)
#add_custom_target(buildTimeDirectory ALL
#	COMMAND ${CMAKE_COMMAND} -E make_directory
#		${CMAKE_BINARY_DIR}/${MAESTRO_JOURNAL_DIR}
#		${CMAKE_BINARY_DIR}/${MAESTRO_TRAJ_DIR}
#		${CMAKE_BINARY_DIR}/${MAESTRO_GEOFENCE_DIR}
#		${CMAKE_BINARY_DIR}/${MAESTRO_OBJECTS_DIR}
#)

# Create directory for test data in user home directory
#install(CODE "MESSAGE(STATUS \"Creating home directory environment under $ENV{HOME}\")")
#install(DIRECTORY DESTINATION "$ENV{HOME}/${MAESTRO_TEST_DIR}")
#install(DIRECTORY DESTINATION "$ENV{HOME}/${MAESTRO_JOURNAL_DIR}")
#install(DIRECTORY DESTINATION "$ENV{HOME}/${MAESTRO_TRAJ_DIR}")
#install(DIRECTORY DESTINATION "$ENV{HOME}/${MAESTRO_GEOFENCE_DIR}")
#install(DIRECTORY DESTINATION "$ENV{HOME}/${MAESTRO_OBJECTS_DIR}")
#install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/conf DESTINATION "$ENV{HOME}/${MAESTRO_TEST_DIR}")

#install(CODE "execute_process(COMMAND chown -R $ENV{USER} $ENV{HOME}/${MAESTRO_TEST_DIR})")

# Add post install instructions
#add_subdirectory(cmake/postinstall)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


# Install configuration
include(GNUInstallDirs)
file(GLOB CONF_FILES ${CMAKE_CURRENT_SOURCE_DIR}/conf/*)
install(FILES ${CONF_FILES} DESTINATION ${CMAKE_INSTALL_SYSCONFDIR})

# Add post install instructions
add_subdirectory(cmake/cpack)
