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
#set(WITH_VISUALIZATION ON CACHE BOOL "Enable Visualization module")
#set(WITH_RELATIVE_KINEMATICS OFF CACHE BOOL "Enable RelativeKinematics module")
set(WITH_DIRECT_CONTROL ON CACHE BOOL "Enable DirectControl module")
set(WITH_OBJECT_MONITORING OFF CACHE BOOL "Enable ObjectMonitoring module")
set(WITH_BACK_TO_START ON CACHE BOOL "Enable BackToStart module")

set(ENABLE_TESTS ON CACHE BOOL "Enable testing on build")

set(MAESTRO_TEST_DIR ".maestro" CACHE STRING "Directory where test data is to be stored")

set(MAESTRO_JOURNAL_DIR "${MAESTRO_TEST_DIR}/journal")
set(MAESTRO_TRAJ_DIR "${MAESTRO_TEST_DIR}/traj")
set(MAESTRO_GEOFENCE_DIR "${MAESTRO_TEST_DIR}/geofence")
set(MAESTRO_OBJECTS_DIR "${MAESTRO_TEST_DIR}/objects")

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

# Ensure .maestro directory is created at build time in build directory
add_custom_target(configurationDirectory ALL
	COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/conf
		${CMAKE_BINARY_DIR}/${MAESTRO_CONFIGURATION_DIR}
	)
add_custom_target(buildTimeDirectory ALL
	COMMAND ${CMAKE_COMMAND} -E make_directory
		${CMAKE_BINARY_DIR}/${MAESTRO_JOURNAL_DIR}
		${CMAKE_BINARY_DIR}/${MAESTRO_TRAJ_DIR}
		${CMAKE_BINARY_DIR}/${MAESTRO_GEOFENCE_DIR}
		${CMAKE_BINARY_DIR}/${MAESTRO_OBJECTS_DIR}
)

# Create directory for test data in user home directory
install(CODE "MESSAGE(STATUS \"Creating home directory environment under $ENV{HOME}\")")
install(DIRECTORY DESTINATION "$ENV{HOME}/${MAESTRO_TEST_DIR}")
install(DIRECTORY DESTINATION "$ENV{HOME}/${MAESTRO_JOURNAL_DIR}")
install(DIRECTORY DESTINATION "$ENV{HOME}/${MAESTRO_TRAJ_DIR}")
install(DIRECTORY DESTINATION "$ENV{HOME}/${MAESTRO_GEOFENCE_DIR}")
install(DIRECTORY DESTINATION "$ENV{HOME}/${MAESTRO_OBJECTS_DIR}")
install(DIRECTORY ${CMAKE_SOURCE_DIR}/conf DESTINATION "$ENV{HOME}/${MAESTRO_TEST_DIR}")

install(CODE "execute_process(COMMAND chown -R $ENV{USER} $ENV{HOME}/${MAESTRO_TEST_DIR})")

# Add post install instructions
add_subdirectory(cmake/postinstall)
