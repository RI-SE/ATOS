cmake_minimum_required(VERSION 3.10)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_VERBOSE_MAKEFILE OFF CACHE BOOL "Enable verbose make")
set(CMAKE_COLOR_MAKEFILE   ON)

project(Maestro VERSION 0.5.0)

# Enable or disable modules
set(WITH_SCENARIO_CONTROL ON CACHE BOOL "Enable ScenarioControl module")
set(WITH_SUPERVISION ON CACHE BOOL "Enable Supervision module")
set(WITH_VISUALIZATION OFF CACHE BOOL "Enable Visualization module")
set(WITH_RELATIVE_KINEMATICS OFF CACHE BOOL "Enable RelativeKinematics module")
set(WITH_DIRECT_CONTROL OFF CACHE BOOL "Enable DirectControl module")
set(WITH_OBJECT_MONITORING OFF CACHE BOOL "Enable ObjectMonitoring module")
set(WITH_BACK_TO_START OFF CACHE BOOL "Enable BackToStart module")

set(ENABLE_TESTS ON CACHE BOOL "Enable testing on build")

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
if(WITH_RELATIVE_KINEMATICS)
	list(APPEND ENABLED_MODULES RelativeKinematics)
	set(WITH_OBJECT_CONTROL OFF)
else()
	set(WITH_OBJECT_CONTROL ON)
endif()


#Delete comments below when ObjectControl becomes an Module instead
#of a thread in Core 
#if(WITH_OBJECT_CONTROL)
#	list(APPEND ENABLED_MODULES ObjectControl)
#endif()

# Testing
if(ENABLE_TESTS)
	enable_testing()
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

# Install configuration
include(GNUInstallDirs)
file(GLOB CONF_FILES ${CMAKE_CURRENT_SOURCE_DIR}/conf/*)
install(FILES ${CONF_FILES} DESTINATION ${CMAKE_INSTALL_SYSCONFDIR})

# Add post install instructions
add_subdirectory(cmake/cpack)
