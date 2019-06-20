cmake_minimum_required(VERSION 2.8)

project(DummyModule)



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

add_executable(dummy src/main.cpp)

install(TARGETS dummy DESTINATION bin)

target_link_libraries(dummy MaestroTime MaestroLogging util)
target_link_libraries(util MQBus MaestroTime)
target_link_libraries(MQBus rt m)



