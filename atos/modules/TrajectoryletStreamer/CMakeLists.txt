# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(trajectorylet_streamer)

find_package(rclcpp REQUIRED)
find_package(atos_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

set(ATOS_COMMON_LIBRARY ATOSCommon)
get_target_property(COMMON_HEADERS ${ATOS_COMMON_LIBRARY} INCLUDE_DIRECTORIES)

# Define target names
set(TRAJECTORYLET_STREAMER_TARGET ${PROJECT_NAME})

include(GNUInstallDirs)

# Create project main executable target
add_executable(${TRAJECTORYLET_STREAMER_TARGET}
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/trajectoryletstreamer.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/trajectorypublisher.cpp
)
# Link project executable to util libraries
target_link_libraries(${TRAJECTORYLET_STREAMER_TARGET}
	${ATOS_COMMON_LIBRARY}
)
target_include_directories(${TRAJECTORYLET_STREAMER_TARGET} PUBLIC SYSTEM
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${COMMON_HEADERS}
)

# ROS specific settings
ament_target_dependencies(${TRAJECTORYLET_STREAMER_TARGET}
  rclcpp
  std_msgs
  nav_msgs
  tf2_geometry_msgs
  atos_interfaces
  tf2
)

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${TRAJECTORYLET_STREAMER_TARGET}\")")
install(TARGETS ${TRAJECTORYLET_STREAMER_TARGET}
	RUNTIME DESTINATION "${CMAKE_INSTALL_LIBDIR}/atos"
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

