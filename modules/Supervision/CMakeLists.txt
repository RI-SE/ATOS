cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(Supervision)

set(SUPERVISION_TARGET ${CMAKE_PROJECT_NAME})
set(UTIL_TARGET MaestroUtil)
set(TIME_TARGET MaestroTime)
set(LOGGING_TARGET MaestroLogging)
set(MESSAGE_BUS_TARGET MaestroMQ)

add_executable(Supervision
	${CMAKE_CURRENT_SOURCE_DIR}/src/geofence.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/supervisionstate.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/trajectory.cpp
)

if(NOT TARGET ${TIME_TARGET})
	add_library(${TIME_TARGET} STATIC
		${CMAKE_CURRENT_SOURCE_DIR}/../../util/C/time/maestroTime.c
	)
endif()

if(NOT TARGET ${LOGGING_TARGET})
	add_library(${LOGGING_TARGET} STATIC
		${CMAKE_CURRENT_SOURCE_DIR}/../../util/C/logging/logging.c
	)
endif()

if(NOT TARGET ${MESSAGE_BUS_TARGET})
	add_library(${MESSAGE_BUS_TARGET} STATIC
		${CMAKE_CURRENT_SOURCE_DIR}/../../util/C/mqbus/mqbus.c
	)
	target_link_libraries(${MESSAGE_BUS_TARGET} rt)
endif()

if(NOT TARGET ${UTIL_TARGET})
	add_library(${UTIL_TARGET} STATIC
		${CMAKE_CURRENT_SOURCE_DIR}/../../core/src/util.c
		${CMAKE_CURRENT_SOURCE_DIR}/../../core/src/iso22133.c
		${CMAKE_CURRENT_SOURCE_DIR}/../../core/src/positioning.c
		${CMAKE_CURRENT_SOURCE_DIR}/../../core/src/datadictionary.c
	)
	target_link_libraries(${UTIL_TARGET} ${LOGGING_TARGET} ${TIME_TARGET} ${MESSAGE_BUS_TARGET} m)
	target_include_directories(${UTIL_TARGET} PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/../../core/inc
		${CMAKE_CURRENT_SOURCE_DIR}/../../util/C/mqbus
		${CMAKE_CURRENT_SOURCE_DIR}/../../util/C/logging
		${CMAKE_CURRENT_SOURCE_DIR}/../../util/C/time
	)
endif()

target_link_libraries(${SUPERVISION_TARGET} LINK_PUBLIC ${TIME_TARGET} ${UTIL_TARGET} ${LOGGING_TARGET})
target_include_directories(${SUPERVISION_TARGET} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${CMAKE_CURRENT_SOURCE_DIR}/../../util/C/time
	${CMAKE_CURRENT_SOURCE_DIR}/../../util/C/logging
	${CMAKE_CURRENT_SOURCE_DIR}/../../util/C/mqbus
	${CMAKE_CURRENT_SOURCE_DIR}/../../core/inc
)
