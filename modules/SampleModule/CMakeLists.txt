cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(sample_module)
find_package(atos_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

# Define target names
set(SAMPLE_MODULE_TARGET ${PROJECT_NAME})

set(COREUTILS_LIBRARY ATOSCoreUtil)
set(COMMON_LIBRARY ATOSCommon) # Common library for ATOS with e.g. Trajectory class
set(SOCKET_LIBRARY TCPUDPSocket) # Socket library for TCP/UDP communication

get_target_property(COMMON_HEADERS ${COMMON_LIBRARY} INCLUDE_DIRECTORIES)
get_target_property(COREUTILS_HEADERS ${COREUTILS_LIBRARY} INCLUDE_DIRECTORIES)

include(GNUInstallDirs)

# Create project main executable target
add_executable(${SAMPLE_MODULE_TARGET}
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/samplemodule.cpp
)
# Link project executable to common libraries
set(TARGET_LIBRARIES
	${COREUTILS_LIBRARY}
	${SOCKET_LIBRARY}
	${COMMON_LIBRARY}
)
target_link_libraries(${SAMPLE_MODULE_TARGET} ${TARGET_LIBRARIES})

# Add include directories
target_include_directories(${SAMPLE_MODULE_TARGET} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${COMMON_HEADERS}
	${COREUTILS_HEADERS}
)

# ROS specific rules
ament_target_dependencies(${SAMPLE_MODULE_TARGET}
	rclcpp
	std_msgs
	std_srvs
	atos_interfaces
)

if(BUILD_TESTING)
	find_package(ament_cmake_ros REQUIRED)
	file(GLOB TESTFILES "test/*.cpp")
	set(SRCFILES "src/samplemodule.cpp")

	ament_add_ros_isolated_gtest(${SAMPLE_MODULE_TARGET}_test ${TESTFILES} ${SRCFILES})
	target_link_libraries(${SAMPLE_MODULE_TARGET}_test ${TARGET_LIBRARIES})
	target_include_directories(${SAMPLE_MODULE_TARGET}_test PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/inc
		${COMMON_HEADERS}
	)

	# ROS specific rules
	ament_target_dependencies(${SAMPLE_MODULE_TARGET}_test
	rclcpp
	std_msgs
	std_srvs
	atos_interfaces
	)
endif()

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${SAMPLE_MODULE_TARGET}\")")
install(TARGETS ${SAMPLE_MODULE_TARGET}
	RUNTIME DESTINATION "${CMAKE_INSTALL_LIBDIR}/atos"
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
