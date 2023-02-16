# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(esmini_adapter)
find_package(atos_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(esminiLib REQUIRED)

# Define target names
set(ESMINI_ADAPTER_TARGET ${PROJECT_NAME})

set(ATOS_COMMON_LIBRARY ATOSCommon)
set(COREUTILS_LIBRARY ATOSCoreUtil)
get_target_property(COMMON_HEADERS ${ATOS_COMMON_LIBRARY} INCLUDE_DIRECTORIES)

include(GNUInstallDirs)

# Create project main executable target
add_executable(${ESMINI_ADAPTER_TARGET}
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/esminiadapter.cpp
)
# Link project executable to util libraries
target_link_libraries(${ESMINI_ADAPTER_TARGET}
	${COREUTILS_LIBRARY}
	${ATOS_COMMON_LIBRARY}
	${esminiLib_LIBRARIES}
)

target_include_directories(${ESMINI_ADAPTER_TARGET} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${COMMON_HEADERS}
	${esminiLib_INCLUDE_DIRS}
)

# ROS specific rules
ament_target_dependencies(${ESMINI_ADAPTER_TARGET}
  rclcpp
  std_msgs
  atos_interfaces
  tf2
)

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${ESMINI_ADAPTER_TARGET}\")")
install(TARGETS ${ESMINI_ADAPTER_TARGET}
	RUNTIME DESTINATION "${CMAKE_INSTALL_LIBDIR}/atos"
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
