cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(ScenarioControl)

add_executable(ScenarioControl
	src/action.cpp
	src/booleantrigger.cpp
	src/braketrigger.cpp
	src/causality.cpp
	src/externalaction.cpp
	src/isotrigger.cpp
	src/main.cpp
	src/scenario.cpp
	src/trigger.cpp
)

target_link_libraries(ScenarioControl LINK_PUBLIC MaestroTime MaestroLogging MaestroMQ MaestroUtil)
target_include_directories(ScenarioControl PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${CMAKE_CURRENT_SOURCE_DIR}/../../util/C/time
	${CMAKE_CURRENT_SOURCE_DIR}/../../util/C/logging
	${CMAKE_CURRENT_SOURCE_DIR}/../../util/C/mqbus
	${CMAKE_CURRENT_SOURCE_DIR}/../../core/inc
)
