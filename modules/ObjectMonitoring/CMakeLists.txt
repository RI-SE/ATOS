cmake_minimum_required(VERSION 3.1)
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_C_STANDARD 11)
SET(CMAKE_C_STANDARD_REQUIRED ON)

project(ObjectMonitoring)
# This module is an example of how to set up a new module external to the Maestro executable


include_directories(inc)
include_directories(../../util/C/logging)
include_directories(../../util/C/time)
include_directories(../../util/C/MQBus)
include_directories(../../server/inc)
include(GNUInstallDirs)

find_package(Threads)

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

add_library(util
  ../../server/inc/util.h
  ../../server/src/util.c
)

add_executable(ObjectMonitoring
	src/main.cpp
)

add_library(ConnectionHandler
	src/connectionhandler.cpp
	inc/connectionhandler.h
)

add_library(ProtocolData
	src/protocoldata.cpp
	inc/protocoldata.h
	src/ISO22133ProtocolData.cpp
	inc/ISO22133ProtocolData.h
)

install(TARGETS ObjectMonitoring DESTINATION bin)

target_link_libraries(ObjectMonitoring MaestroTime MaestroLogging util ConnectionHandler)
target_link_libraries(ConnectionHandler ProtocolData ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(util MQBus MaestroLogging MaestroTime)
target_link_libraries(MQBus rt m)



