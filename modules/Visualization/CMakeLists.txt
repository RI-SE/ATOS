cmake_minimum_required(VERSION 3.1)

project(Visualization)
# This module is an example of how to set up a new module external to the Maestro executable

# Set C and C++ dialects
SET(CMAKE_C_STANDARD 11)
SET(CMAKE_C_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(inc)
include_directories(../../util/C/logging)
include_directories(../../util/C/time)
include_directories(../../util/C/MQBus)
include_directories(../../server/inc)
include(GNUInstallDirs)


# Create library
add_library(MaestroLogging
  ../../util/C/logging/logging.h
  ../../util/C/logging/logging.c
)

add_library(MaestroTime
  ../../util/C/time/maestroTime.h
  ../../util/C/time/maestroTime.c
)

add_library(MQBus
  ../../util/C/MQBus/mqbus.h
  ../../util/C/MQBus/mqbus.c
)

# Create library
add_library(util
  ../../server/src/util.c
  ../../server/inc/util.h
)

add_executable(Visualization
    src/main.c)

install(TARGETS Visualization DESTINATION bin)

target_link_libraries(Visualization MaestroTime MaestroLogging util)
target_link_libraries(util MQBus MaestroLogging MaestroTime)
target_link_libraries(MQBus rt m)



