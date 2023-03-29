# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(atos_base)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(atos_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Define target names
set(ATOS_BASE_TARGET ${PROJECT_NAME})

set(COREUTILS_LIBRARY ATOSCoreUtil)
set(COMMON_LIBRARY ATOSCommon)
set(POSITIONING_LIBRARY MaestroPositioning)
set(TIME_LIBRARY ATOSTime)

get_target_property(COREUTILS_HEADERS ${COREUTILS_LIBRARY} INCLUDE_DIRECTORIES)
get_target_property(COMMON_HEADERS ${COMMON_LIBRARY} INCLUDE_DIRECTORIES)

include(GNUInstallDirs)

# Create project main executable target
add_executable(${ATOS_BASE_TARGET}
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/ATOSbase.cpp
)
# Link project executable to util libraries
target_link_libraries(${ATOS_BASE_TARGET}
	${rclcpp_LIBRARIES}
	${COREUTILS_LIBRARY}
	${COMMON_LIBRARY}
)
target_include_directories(${ATOS_BASE_TARGET} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${COREUTILS_HEADERS}
	${COMMON_HEADERS}
)

# ROS specific rules
ament_target_dependencies(${ATOS_BASE_TARGET}
  rclcpp
  std_msgs
  std_srvs
  atos_interfaces
	ament_index_cpp
)

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${ATOS_BASE_TARGET}\")")
install(TARGETS ${ATOS_BASE_TARGET} 
	RUNTIME DESTINATION "${CMAKE_INSTALL_LIBDIR}/atos"
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

