# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

cmake_minimum_required(VERSION 3.8)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(ATOSCommon LANGUAGES C CXX)

include(GNUInstallDirs)
include(FindPkgConfig)

set(ATOS_COMMON_TARGET ${TARGET_NAMESPACE}${PROJECT_NAME})
set(ATOS_UTIL_TARGET ${TARGET_NAMESPACE}ATOSCoreUtil)

find_package(Eigen3 REQUIRED IMPORTED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(atos_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(foxglove_msgs REQUIRED)
pkg_check_modules(PROJ REQUIRED proj)

set(TIME_LIBRARY ATOSTime)
set(SHARED_MEMORY_LIBRARY MaestroSHM)
set(ISO_22133_LIBRARY ISO22133)
set(MATH_LIBRARY m)
set(FILESYSTEM_LIBRARY stdc++fs)
set(PTHREAD_LIBRARY pthread)

get_target_property(TIME_HEADERS ${TIME_LIBRARY} INCLUDE_DIRECTORIES)
get_target_property(SHARED_MEMORY_HEADERS ${SHARED_MEMORY_LIBRARY} INCLUDE_DIRECTORIES)
get_target_property(ISO_22133_HEADERS ${ISO_22133_LIBRARY} INCLUDE_DIRECTORIES)


# Built targets
add_library(${ATOS_UTIL_TARGET} SHARED
	${CMAKE_CURRENT_SOURCE_DIR}/util.c
)

add_library(${ATOS_COMMON_TARGET} SHARED
	${CMAKE_CURRENT_SOURCE_DIR}/trajectory.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/objectconfig.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/module.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/journal.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/type.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/CRSTransformation.cpp
)

# Includes
target_include_directories(${ATOS_UTIL_TARGET} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
	${TIME_HEADERS}
	${MESSAGE_BUS_HEADERS}
    ${ISO_22133_HEADERS}
	${SHARED_MEMORY_HEADERS}
	${rclcpp_INCLUDE_DIRS}
)
target_include_directories(${ATOS_COMMON_TARGET} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}
	${rclcpp_INCLUDE_DIRS}
)

# Linking
target_link_libraries(${ATOS_UTIL_TARGET}
	${TIME_LIBRARY}
	${MESSAGE_BUS_LIBRARY}
	${MATH_LIBRARY}
	${ISO_22133_LIBRARY}
	${SHARED_MEMORY_LIBRARY}
	${rclcpp_LIBRARIES}
)

target_link_libraries(${ATOS_COMMON_TARGET}
	${ATOS_UTIL_TARGET}
	${EIGEN_LIBRARY}
	${FILESYSTEM_LIBRARY}
	${rclcpp_LIBRARIES}
)

# ROS specifics
ament_target_dependencies(${ATOS_COMMON_TARGET}
  rclcpp
  std_msgs
  std_srvs
  nav_msgs
  sensor_msgs
  atos_interfaces
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  foxglove_msgs
  PROJ
)

# Exported headers
set_target_properties(${ATOS_UTIL_TARGET} PROPERTIES
	PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/util.h
)
set_property(TARGET ${ATOS_COMMON_TARGET} APPEND PROPERTY
	PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/CRSTransformation.hpp
)
set_property(TARGET ${ATOS_UTIL_TARGET} APPEND PROPERTY
	PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/testUtils/testUtils.hpp
)
set_property(TARGET ${ATOS_COMMON_TARGET} APPEND PROPERTY
	PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/trajectory.hpp
)
set_property(TARGET ${ATOS_COMMON_TARGET} APPEND PROPERTY
        PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/objectconfig.hpp
)
set_property(TARGET ${ATOS_COMMON_TARGET} APPEND PROPERTY
	PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/regexpatterns.hpp
)
set_property(TARGET ${ATOS_COMMON_TARGET} APPEND PROPERTY
	PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/module.hpp
)
set_property(TARGET ${ATOS_COMMON_TARGET} APPEND PROPERTY
	PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/loggable.hpp
)
set_property(TARGET ${ATOS_COMMON_TARGET} APPEND PROPERTY
	PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/journal.hpp
)
set_property(TARGET ${ATOS_COMMON_TARGET} APPEND PROPERTY
	PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/type.h
)

# Tests
add_executable(test_relativetrajectory tests/test_relativetrajectory.cpp)
add_test(relative_trajectory_point_test
	${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test_relativetrajectory)
target_link_libraries(test_relativetrajectory
	${ATOS_COMMON_TARGET}
)

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${ATOS_UTIL_TARGET}\")")
install(TARGETS ${ATOS_UTIL_TARGET}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(CODE "MESSAGE(STATUS \"Installing target ${ATOS_COMMON_TARGET}\")")
install(TARGETS ${ATOS_COMMON_TARGET}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
