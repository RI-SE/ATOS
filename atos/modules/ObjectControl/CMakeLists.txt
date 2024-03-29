# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)



project(object_control)

find_package(rclcpp REQUIRED)
find_package(atos_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)


# Define target names
set(OBJECT_CONTROL_TARGET ${PROJECT_NAME})

set(COREUTILS_LIBRARY ATOSCoreUtil)
set(ISO_22133_LIBRARY ISO22133)
set(TIME_LIBRARY ATOSTime)
set(ATOS_COMMON_LIBRARY ATOSCommon)
set(FILESYSTEM_LIBRARY stdc++fs)
set(THREAD_LIBRARY pthread)
set(OSI_LIBRARY OSIHandler)

get_target_property(OSI_HEADERS ${OSI_LIBRARY} INCLUDE_DIRECTORIES)
get_target_property(TIME_HEADERS ${TIME_LIBRARY} INCLUDE_DIRECTORIES)

include(GNUInstallDirs)

# Create project main executable target
add_executable(${OBJECT_CONTROL_TARGET}
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/testobject.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/relativetestobject.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/objectlistener.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/objectconnection.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/channel.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/states/state.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/states/idle.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/states/initialized.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/states/connecting.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/states/ready.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/states/aborting.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/states/clearing.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/states/testlive.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/states/disarming.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/states/armed.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/states/done.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/states/remotecontrolled.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/objectcontrol.cpp
)
# Link project executable to util libraries
target_link_libraries(${OBJECT_CONTROL_TARGET} 
	${TIME_LIBRARY}
	${COREUTILS_LIBRARY}
	${FILESYSTEM_LIBRARY}
	${THREAD_LIBRARY}
	${ATOS_COMMON_LIBRARY}
	${ISO_22133_LIBRARY}
	${OSI_LIBRARY}
)

# ROS specific settings
ament_target_dependencies(${OBJECT_CONTROL_TARGET}
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2_geometry_msgs
  atos_interfaces
  tf2
)

target_include_directories(${OBJECT_CONTROL_TARGET} PUBLIC SYSTEM
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${OSI_HEADERS}
	${TIME_HEADERS}
)

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${OBJECT_CONTROL_TARGET}\")")
install(TARGETS ${OBJECT_CONTROL_TARGET}
	RUNTIME DESTINATION "${CMAKE_INSTALL_LIBDIR}/atos"
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

