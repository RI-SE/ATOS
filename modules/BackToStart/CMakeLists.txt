# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(back_to_start)
find_package(rclcpp REQUIRED)
find_package(atos_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

# Define target names
set(BACK_TO_START ${PROJECT_NAME})

set(ATOS_COMMON_LIBRARY ATOSCommon)
set(COREUTILS_LIBRARY ATOSCoreUtil)

get_target_property(COMMON_HEADERS ${ATOS_COMMON_LIBRARY} INCLUDE_DIRECTORIES)
get_target_property(COREUTILS_HEADERS ${COREUTILS_LIBRARY} INCLUDE_DIRECTORIES)

include(GNUInstallDirs)

# Create project main executable target
add_executable(${BACK_TO_START}
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/backtostart.cpp
)

set(TARGET_LIBRARIES
	${ATOS_COMMON_LIBRARY}
	${COREUTILS_LIBRARY}
)

# Link project executable to util libraries
target_link_libraries(${BACK_TO_START}
    ${TARGET_LIBRARIES}
)
target_include_directories(${BACK_TO_START} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${COMMON_HEADERS}
	${COREUTILS_HEADERS}
)

# ROS specific rules
ament_target_dependencies(${BACK_TO_START}
	rclcpp
	atos_interfaces
	geometry_msgs
)

if(BUILD_TESTING)
	find_package(ament_cmake_ros REQUIRED)
	file(GLOB TESTFILES "tests/*.cpp")
	set(SRCFILES "src/backtostart.cpp")

	ament_add_ros_isolated_gtest(${BACK_TO_START}_test ${TESTFILES} ${SRCFILES})
	target_link_libraries(${BACK_TO_START}_test ${TARGET_LIBRARIES})
	target_include_directories(${BACK_TO_START}_test PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/inc
		${COMMON_HEADERS}
		${COREUTILS_HEADERS}
	)

	# ROS specific rules
	ament_target_dependencies(${BACK_TO_START}_test
		rclcpp
		atos_interfaces
		geometry_msgs
	)
endif()

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${BACK_TO_START}\")")
install(TARGETS ${BACK_TO_START} 
	RUNTIME DESTINATION "${CMAKE_INSTALL_LIBDIR}/atos"
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)


