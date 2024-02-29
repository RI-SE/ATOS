# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

cmake_minimum_required(VERSION 3.10)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(amqp_bridge)
find_package(rclcpp REQUIRED)
find_package(atos_interfaces REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(PKG_AMQPCLIENT REQUIRED IMPORTED_TARGET libSimpleAmqpClient)
# find_package(libSimpleAmqpClient)
# find_package(amqpcpp REQUIRED)

# Define target names
set(AMQP_BRIDGE_TARGET ${PROJECT_NAME})
set(ATOS_COMMON_LIBRARY ATOSCommon)
get_target_property(COMMON_HEADERS ${ATOS_COMMON_LIBRARY} INCLUDE_DIRECTORIES)

include(GNUInstallDirs)

# Create project main executable target
add_executable(${AMQP_BRIDGE_TARGET}
  	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/amqpbridge.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/amqpclient.cpp
)

# Link project executable to util libraries
target_link_libraries(${AMQP_BRIDGE_TARGET}
	${ATOS_COMMON_LIBRARY}
	PkgConfig::PKG_AMQPCLIENT
)

target_include_directories(${AMQP_BRIDGE_TARGET} PUBLIC SYSTEM
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${CMAKE_INSTALL_INCLUDEDIR}
	${COMMON_HEADERS}
)

# ROS specific rules
ament_target_dependencies(${AMQP_BRIDGE_TARGET}
  rclcpp
  atos_interfaces
)

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${AMQP_BRIDGE_TARGET}\")")
install(TARGETS ${AMQP_BRIDGE_TARGET}
	RUNTIME DESTINATION "${CMAKE_INSTALL_LIBDIR}/atos"
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)