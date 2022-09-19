cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(trajectorylet_streamer)

find_package(rclcpp REQUIRED)
find_package(maestro_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

set(MAESTRO_COMMON_LIBRARY MaestroCommon)
get_target_property(COMMON_HEADERS ${MAESTRO_COMMON_LIBRARY} INCLUDE_DIRECTORIES)

# Define target names
set(TRAJECTORYLET_STREAMER_TARGET ${PROJECT_NAME})

include(GNUInstallDirs)

# Create project main executable target
add_executable(${TRAJECTORYLET_STREAMER_TARGET}
	${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/trajectoryletstreamer.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/trajectorypublisher.cpp
)
# Link project executable to util libraries
target_link_libraries(${TRAJECTORYLET_STREAMER_TARGET}
	${MAESTRO_COMMON_LIBRARY}
)

# ROS specific settings
ament_target_dependencies(${TRAJECTORYLET_STREAMER_TARGET}
  rclcpp
  std_msgs
  nav_msgs
  tf2_geometry_msgs
  maestro_interfaces
  tf2
)

target_include_directories(${TRAJECTORYLET_STREAMER_TARGET} PUBLIC
	${COMMON_HEADERS}
)

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${TRAJECTORYLET_STREAMER_TARGET}\")")
install(TARGETS ${TRAJECTORYLET_STREAMER_TARGET}
	RUNTIME DESTINATION "${CMAKE_INSTALL_LIBDIR}/maestro"
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

