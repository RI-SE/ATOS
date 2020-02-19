cmake_minimum_required(VERSION 3.10)

project(Maestro)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_subdirectory(util/C)
add_subdirectory(core)
add_subdirectory(modules/ScenarioControl)
add_subdirectory(modules/Supervision)
add_subdirectory(modules/Visualization)
