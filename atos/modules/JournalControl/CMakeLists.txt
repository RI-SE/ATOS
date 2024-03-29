# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(journal_control)

find_package(rclcpp REQUIRED)
find_package(atos_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# Define target names
set(JOURNAL_CONTROL_TARGET ${PROJECT_NAME})

set(COREUTILS_LIBRARY ATOSCoreUtil)
set(FILESYSTEM_LIBRARY stdc++fs)
set(COMMON_LIBRARY ATOSCommon)

get_target_property(COMMON_HEADERS ${COMMON_LIBRARY} INCLUDE_DIRECTORIES)
get_target_property(COREUTILS_HEADERS ${COREUTILS_LIBRARY} INCLUDE_DIRECTORIES)

include(GNUInstallDirs)

# Create project main executable target
add_executable(${JOURNAL_CONTROL_TARGET}
	${CMAKE_CURRENT_SOURCE_DIR}/src/journalmodelcollection.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/journalcontrol.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/journalmodel.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
)
# Link project executable to util libraries
target_link_libraries(${JOURNAL_CONTROL_TARGET}
	${rclcpp_LIBRARIES}
	${FILESYSTEM_LIBRARY}
	${COREUTILS_LIBRARY}
	${COMMON_LIBRARY}
)
target_include_directories(${JOURNAL_CONTROL_TARGET} PUBLIC SYSTEM
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${COREUTILS_HEADERS}
	${COMMON_HEADERS}
)

# ROS specific rules
ament_target_dependencies(${JOURNAL_CONTROL_TARGET}
  rclcpp
  std_msgs
  atos_interfaces
)

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${JOURNAL_CONTROL_TARGET}\")")
install(TARGETS ${JOURNAL_CONTROL_TARGET} 
	RUNTIME DESTINATION "${CMAKE_INSTALL_LIBDIR}/atos"
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

