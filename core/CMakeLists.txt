cmake_minimum_required(VERSION 3.1)

project(Core)

set(CORE_TARGET ${CMAKE_PROJECT_NAME})
set(UTIL_TARGET MaestroUtil)
set(TIME_TARGET MaestroTime)
set(LOGGING_TARGET MaestroLogging)
set(MESSAGE_BUS_TARGET MaestroMQ)

add_executable(${CORE_TARGET} 
	${CMAKE_CURRENT_SOURCE_DIR}/src/objectcontrol.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/logger.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/systemcontrol.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/timecontrol.c
)

add_library(${UTIL_TARGET}
	${CMAKE_CURRENT_SOURCE_DIR}/src/util.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/iso22133.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/positioning.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/datadictionary.c
)

if(NOT TARGET ${TIME_TARGET})
	add_library(${TIME_TARGET} STATIC
		${CMAKE_CURRENT_SOURCE_DIR}/../util/C/time/maestroTime.c
	)
endif()

if(NOT TARGET ${LOGGING_TARGET})
	add_library(${LOGGING_TARGET} STATIC
		${CMAKE_CURRENT_SOURCE_DIR}/../util/C/logging/logging.c
	)
endif()

if(NOT TARGET ${MESSAGE_BUS_TARGET})
	add_library(${MESSAGE_BUS_TARGET} STATIC
		${CMAKE_CURRENT_SOURCE_DIR}/../util/C/mqbus/mqbus.c
	)
	target_link_libraries(${MESSAGE_BUS_TARGET} rt)
endif()

target_link_libraries(${CORE_TARGET} LINK_PUBLIC ${TIME_TARGET} ${LOGGING_TARGET} ${UTIL_TARGET})
target_include_directories(${CORE_TARGET} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${CMAKE_CURRENT_SOURCE_DIR}/../util/C/time
	${CMAKE_CURRENT_SOURCE_DIR}/../util/C/logging
)

target_link_libraries(${UTIL_TARGET} LINK_PUBLIC ${TIME_TARGET} ${LOGGING_TARGET} ${MESSAGE_BUS_TARGET})
target_link_libraries(${UTIL_TARGET} LINK_PUBLIC m)
target_include_directories(${UTIL_TARGET} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${CMAKE_CURRENT_SOURCE_DIR}/../util/C/time
	${CMAKE_CURRENT_SOURCE_DIR}/../util/C/logging
	${CMAKE_CURRENT_SOURCE_DIR}/../util/C/mqbus
)
