cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Adding tinyxml2 to the CMake
find_package(TinyXML2 REQUIRED)

project(drone_control)
find_package(atos_interfaces REQUIRED)
# Define target names
set(DRONE_CONTROL_TARGET ${PROJECT_NAME})

set(COMMON_LIBRARY ATOSCommon) # Common library for ATOS with e.g. Trajectory class

get_target_property(COMMON_HEADERS ${COMMON_LIBRARY} INCLUDE_DIRECTORIES)

include(GNUInstallDirs)

# Create project main executable target
add_executable(${DRONE_CONTROL_TARGET}
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/dronecontrol.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/path.cpp
)

# Link project executable to common libraries
target_link_libraries(${DRONE_CONTROL_TARGET}
	${COREUTILS_LIBRARY}
	${SOCKET_LIBRARY}
	${COMMON_LIBRARY}
	${TinyXML2_LIBRARIES} #TODO: ADD LIB TINYXML
)

target_include_directories(${DRONE_CONTROL_TARGET} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${COMMON_HEADERS}
	${TinyXML2_INCLUDE_DIRS} #added
)

# ROS specific rules
ament_target_dependencies(${DRONE_CONTROL_TARGET}
  rclcpp
  std_msgs
  atos_interfaces
)

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${DRONE_CONTROL_TARGET}\")")
install(TARGETS ${DRONE_CONTROL_TARGET} 
	RUNTIME DESTINATION "${CMAKE_INSTALL_LIBDIR}/atos"
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
