# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(osi_adapter)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(atos_interfaces REQUIRED)

# Define target names
set(OSI_ADAPTER_TARGET ${PROJECT_NAME})

set(OSI_LIBRARY OSIHandler)
set(ATOS_COMMON_LIBRARY ATOSCommon)
set(SOCKET_LIBRARY TCPUDPSocket)

get_target_property(OSI_HEADERS ${OSI_LIBRARY} INCLUDE_DIRECTORIES)
get_target_property(SOCKET_HEADERS ${SOCKET_LIBRARY} INCLUDE_DIRECTORIES)

include(GNUInstallDirs)

# Create project main executable target
add_executable(${OSI_ADAPTER_TARGET}
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/osiadapter.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/serverfactory.cpp
)
# Link project executable to util libraries
target_link_libraries(${OSI_ADAPTER_TARGET} 
	${ATOS_COMMON_LIBRARY}
	${OSI_LIBRARY}
	${SOCKET_LIBRARY}
)

# ROS specific settings
ament_target_dependencies(${OSI_ADAPTER_TARGET}
  rclcpp
  std_msgs
  atos_interfaces
)

target_include_directories(${OSI_ADAPTER_TARGET} PUBLIC SYSTEM
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${OSI_HEADERS}
	${SOCKET_HEADERS}
)

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${OSI_ADAPTER_TARGET}\")")
install(TARGETS ${OSI_ADAPTER_TARGET}
	RUNTIME DESTINATION "${CMAKE_INSTALL_LIBDIR}/atos"
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)