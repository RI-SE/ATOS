# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(pointcloud_publisher)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(atos_interfaces REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(pcl_conversions REQUIRED)

# Define target names
set(POINTCLOUD_PUBLISHER_TARGET ${PROJECT_NAME})

set(ATOS_COMMON_LIBRARY ATOSCommon)
get_target_property(COMMON_HEADERS ${ATOS_COMMON_LIBRARY} INCLUDE_DIRECTORIES)

include(GNUInstallDirs)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Create project main executable target
add_executable(${POINTCLOUD_PUBLISHER_TARGET}
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/pointcloudpublisher.cpp
)
# Link project executable to util libraries
target_link_libraries(${POINTCLOUD_PUBLISHER_TARGET} 
	${ATOS_COMMON_LIBRARY}
	${ATOS_COMMON_LIBRARY}
  ${PCL_LIBRARIES}
)

# ROS specific settings
ament_target_dependencies(${POINTCLOUD_PUBLISHER_TARGET}
  rclcpp
  std_msgs
  sensor_msgs
  atos_interfaces
  pcl_conversions
)

target_include_directories(${POINTCLOUD_PUBLISHER_TARGET} PUBLIC SYSTEM
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${COMMON_HEADERS}
)

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${OSI_ADAPTER_TARGET}\")")
install(TARGETS ${POINTCLOUD_PUBLISHER_TARGET}
	RUNTIME DESTINATION "${CMAKE_INSTALL_LIBDIR}/atos"
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)