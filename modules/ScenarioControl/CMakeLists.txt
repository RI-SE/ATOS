cmake_minimum_required(VERSION 2.8)

project(ScenarioControl)
# This module is an example of how to set up a new module external to the Maestro executable


include_directories(inc)
include_directories(src)
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

add_library(Trigger
    src/trigger.cpp
    inc/trigger.h
)

add_executable(ScenarioControl src/main.cpp)

install(TARGETS ScenarioControl DESTINATION bin)

target_link_libraries(ScenarioControl MaestroTime MaestroLogging util Trigger)
target_link_libraries(util MQBus MaestroTime MaestroLogging)
target_link_libraries(MQBus rt m)



