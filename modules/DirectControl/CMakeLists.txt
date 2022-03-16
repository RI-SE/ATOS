cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(direct_control)

# Define target names
set(DIRECT_CONTROL_TARGET ${PROJECT_NAME})

set(COREUTILS_LIBRARY MaestroCoreUtil)
set(ISO_22133_LIBRARY MaestroISO22133)
set(POSITIONING_LIBRARY MaestroPositioning)
set(TIME_LIBRARY MaestroTime)
set(COMMON_LIBRARY MaestroCommon)
set(SOCKET_LIBRARY TCPUDPSocket)
set(THREAD_LIBRARY pthread)

get_target_property(COMMON_HEADERS ${COMMON_LIBRARY} INCLUDE_DIRECTORIES)

include(GNUInstallDirs)

# Create project main executable target
add_executable(${DIRECT_CONTROL_TARGET}
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/directcontrol.cpp
)
# Link project executable to util libraries
target_link_libraries(${DIRECT_CONTROL_TARGET}
	${TIME_LIBRARY}
	${COREUTILS_LIBRARY}
	${LOGGING_LIBRARY}
	${SOCKET_LIBRARY}
	${THREAD_LIBRARY}
	${COMMON_LIBRARY}
)

target_include_directories(${DIRECT_CONTROL_TARGET} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${COMMON_HEADERS}
)

# ROS specific rules
rosidl_target_interfaces(${DIRECT_CONTROL_TARGET}
	maestro_msgs "rosidl_typesupport_cpp"
)
ament_target_dependencies(${DIRECT_CONTROL_TARGET}
  rclcpp
  std_msgs
)

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${DIRECT_CONTROL_TARGET}\")")
install(TARGETS ${DIRECT_CONTROL_TARGET} 
	RUNTIME DESTINATION "${CMAKE_INSTALL_LIBDIR}/maestro"
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
