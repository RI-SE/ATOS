cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(rest_bridge)
find_package(atos_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

# Define target names
set(REST_BRIDGE_TARGET ${PROJECT_NAME})

set(COREUTILS_LIBRARY ATOSCoreUtil)
set(COMMON_LIBRARY ATOSCommon) # Common library for ATOS with e.g. Trajectory class
set(SOCKET_LIBRARY TCPUDPSocket) # Socket library for TCP/UDP communication

get_target_property(COMMON_HEADERS ${COMMON_LIBRARY} INCLUDE_DIRECTORIES)
get_target_property(COREUTILS_HEADERS ${COREUTILS_LIBRARY} INCLUDE_DIRECTORIES)

include(GNUInstallDirs)

# Create project main executable target
add_executable(${REST_BRIDGE_TARGET}
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/restbridge.cpp
)
# Link project executable to common libraries
set(TARGET_LIBRARIES
	${COREUTILS_LIBRARY}
	${SOCKET_LIBRARY}
	${COMMON_LIBRARY}
	${CURL_LIBRARIES} curl)

target_link_libraries(${REST_BRIDGE_TARGET} ${TARGET_LIBRARIES})

# Add include directories
target_include_directories(${REST_BRIDGE_TARGET} PUBLIC SYSTEM
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${COMMON_HEADERS}
	${COREUTILS_HEADERS}
)

# ROS specific rules
ament_target_dependencies(${REST_BRIDGE_TARGET}
	rclcpp
	std_msgs
	std_srvs
	atos_interfaces
)

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${REST_BRIDGE_TARGET}\")")
install(TARGETS ${REST_BRIDGE_TARGET}
	RUNTIME DESTINATION "${CMAKE_INSTALL_LIBDIR}/atos"
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
