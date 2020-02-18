cmake_minimum_required(VERSION 3.1)

SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_C_STANDARD 11)
SET(CMAKE_C_STANDARD_REQUIRED ON)

project(Supervision)


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
  ../../server/src/iso22133.c
  ../../server/inc/iso22133.h
  ../../server/inc/positioning.h
  ../../server/src/positioning.c
)

add_executable(Supervision
    src/main.cpp
    src/geofence.cpp
    inc/geofence.h
    src/trajectory.cpp
    inc/trajectory.h
    inc/regexpatterns.h
    src/supervisionstate.cpp
    inc/supervisionstate.h
)

install(TARGETS Supervision DESTINATION bin)

target_link_libraries(Supervision MaestroTime MaestroLogging util)
target_link_libraries(util MQBus MaestroTime MaestroLogging)
target_link_libraries(MQBus rt m)



