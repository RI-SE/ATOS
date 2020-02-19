cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(dummy)

# Define target names
set(DUMMY_TARGET ${PROJECT_NAME})
set(UTIL_TARGET MaestroUtil)
set(TIME_TARGET MaestroTime)
set(LOGGING_TARGET MaestroLogging)
set(MESSAGE_BUS_TARGET MaestroMQ)

# Define paths to related files
set(CORE_ROOT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../core)
set(CORE_INCLUDE_PATH ${CORE_ROOT_PATH}/inc)
set(CORE_SOURCE_PATH ${CORE_ROOT_PATH}/src)
set(UTIL_ROOT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../util/C)
set(TIME_PATH ${UTIL_ROOT_PATH}/time)
set(LOGGING_PATH ${UTIL_ROOT_PATH}/logging)
set(MESSAGE_BUS_PATH ${UTIL_ROOT_PATH}/mqbus)

# Create project main executable target
add_executable(${DUMMY_TARGET}
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
)

# If building locally, it is necessary to add relevant util targets
if(NOT TARGET ${TIME_TARGET})
	add_library(${TIME_TARGET} STATIC
		${TIME_PATH}/maestroTime.c
	)
endif()

if(NOT TARGET ${LOGGING_TARGET})
	add_library(${LOGGING_TARGET} STATIC
		${LOGGING_PATH}/logging.c
	)
endif()

if(NOT TARGET ${MESSAGE_BUS_TARGET})
	add_library(${MESSAGE_BUS_TARGET} STATIC
		${MESSAGE_BUS_PATH}/mqbus.c
	)
	target_link_libraries(${MESSAGE_BUS_TARGET} rt)
endif()

if(NOT TARGET ${UTIL_TARGET})
	add_library(${UTIL_TARGET} STATIC
		${CORE_SOURCE_PATH}/util.c
		${CORE_SOURCE_PATH}/iso22133.c
		${CORE_SOURCE_PATH}/positioning.c
		${CORE_SOURCE_PATH}/datadictionary.c
	)
	target_link_libraries(${UTIL_TARGET} ${LOGGING_TARGET} ${TIME_TARGET} ${MESSAGE_BUS_TARGET} m)
	target_include_directories(${UTIL_TARGET} PUBLIC
		${CORE_INCLUDE_PATH}
		${MESSAGE_BUS_PATH}
		${LOGGING_PATH}
		${TIME_PATH}
	)
endif()

# Link project executable to util libraries
target_link_libraries(${DUMMY_TARGET} LINK_PUBLIC ${TIME_TARGET} ${UTIL_TARGET} ${LOGGING_TARGET})
target_include_directories(${DUMMY_TARGET} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${LOGGING_PATH}
	${TIME_PATH}
	${CORE_INCLUDE_PATH}
)
