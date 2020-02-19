cmake_minimum_required(VERSION 3.1)

project(Visualization)

# Set C and C++ dialects
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(Visualization
	src/main.c
)

target_link_libraries(Visualization LINK_PUBLIC MaestroTime MaestroLogging MaestroUtil)
target_include_directories(Visualization PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/../../util/C/time
	${CMAKE_CURRENT_SOURCE_DIR}/../../util/C/logging
	${CMAKE_CURRENT_SOURCE_DIR}/../../core/inc
)
