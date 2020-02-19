cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(Supervision)

add_executable(Supervision
	src/geofence.cpp
	src/main.cpp
	src/supervisionstate.cpp
	src/trajectory.cpp
)

target_link_libraries(Supervision LINK_PUBLIC MaestroTime MaestroLogging MaestroMQ MaestroUtil)
target_include_directories(Supervision PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${CMAKE_CURRENT_SOURCE_DIR}/../../util/C/time
	${CMAKE_CURRENT_SOURCE_DIR}/../../util/C/logging
	${CMAKE_CURRENT_SOURCE_DIR}/../../util/C/mqbus
	${CMAKE_CURRENT_SOURCE_DIR}/../../core/inc
)
