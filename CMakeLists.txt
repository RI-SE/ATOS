cmake_minimum_required(VERSION 3.10)

project(Maestro)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_COLOR_MAKEFILE   ON)

# Configure structure of output
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_subdirectory(util/C)
add_subdirectory(core)
add_subdirectory(modules/ScenarioControl)
add_subdirectory(modules/Supervision)
add_subdirectory(modules/Visualization)

# Create directory for test data in user home directory
install(CODE "MESSAGE(STATUS \"Creating home directory environment under $ENV{HOME}\")")
install(DIRECTORY DESTINATION "$ENV{HOME}/.maestro" USE_SOURCE_PERMISSIONS)
install(DIRECTORY DESTINATION "$ENV{HOME}/.maestro/journal" USE_SOURCE_PERMISSIONS)
install(DIRECTORY DESTINATION "$ENV{HOME}/.maestro/traj" USE_SOURCE_PERMISSIONS)
install(DIRECTORY DESTINATION "$ENV{HOME}/.maestro/geofence" USE_SOURCE_PERMISSIONS)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/conf
	DESTINATION "$ENV{HOME}/.maestro" USE_SOURCE_PERMISSIONS)

# Ensure libraries are reloaded after install
install(CODE "execute_process(COMMAND ldconfig)")
