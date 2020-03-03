cmake_minimum_required(VERSION 3.10)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
install(DIRECTORY DESTINATION "$ENV{HOME}/.maestro")
install(DIRECTORY DESTINATION "$ENV{HOME}/.maestro/journal")
install(DIRECTORY DESTINATION "$ENV{HOME}/.maestro/traj")
install(DIRECTORY DESTINATION "$ENV{HOME}/.maestro/geofence")
install(DIRECTORY ${CMAKE_SOURCE_DIR}/conf DESTINATION "$ENV{HOME}/.maestro")

install(CODE "execute_process(COMMAND chown -R $ENV{USER} $ENV{HOME}/.maestro)")

