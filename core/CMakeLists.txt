cmake_minimum_required(VERSION 3.1)

project(Core)

add_executable(Core 
	src/objectcontrol.c
	src/logger.c
	src/main.c
	src/systemcontrol.c
	src/timecontrol.c
)

add_library(MaestroUtil
	src/util.c
	src/iso22133.c
	src/positioning.c
	src/datadictionary.c
)

if(NOT TARGET MaestroTime)
	add_library(MaestroTime STATIC
		${CMAKE_CURRENT_SOURCE_DIR}/../util/C/time/maestroTime.c
	)
endif()

if(NOT TARGET MaestroLogging)
	add_library(MaestroLogging STATIC
		${CMAKE_CURRENT_SOURCE_DIR}/../util/C/logging/logging.c
	)
endif()

if(NOT TARGET MaestroMQ)
	add_library(MaestroMQ STATIC
		${CMAKE_CURRENT_SOURCE_DIR}/../util/C/mqbus/mqbus.c
	)
	target_link_libraries(MaestroMQ rt)
endif()

target_link_libraries(Core LINK_PUBLIC MaestroTime MaestroLogging MaestroUtil)
target_include_directories(Core PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${CMAKE_CURRENT_SOURCE_DIR}/../util/C/time
	${CMAKE_CURRENT_SOURCE_DIR}/../util/C/logging
)

target_link_libraries(MaestroUtil LINK_PUBLIC MaestroTime MaestroLogging MaestroMQ)
target_link_libraries(MaestroUtil LINK_PUBLIC m)
target_include_directories(MaestroUtil PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${CMAKE_CURRENT_SOURCE_DIR}/../util/C/time
	${CMAKE_CURRENT_SOURCE_DIR}/../util/C/logging
	${CMAKE_CURRENT_SOURCE_DIR}/../util/C/mqbus
)
