cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(maestro_base)

find_package(rclcpp REQUIRED)

# Define target names
set(MAESTRO_BASE_TARGET ${PROJECT_NAME})

set(COREUTILS_LIBRARY MaestroCoreUtil)
set(COMMON_LIBRARY MaestroCommon)
set(POSITIONING_LIBRARY MaestroPositioning)
set(TIME_LIBRARY MaestroTime)
set(LOGGING_LIBRARY MaestroLogging)

get_target_property(COREUTILS_HEADERS ${COREUTILS_LIBRARY} INCLUDE_DIRECTORIES)
get_target_property(COMMON_HEADERS ${COMMON_LIBRARY} INCLUDE_DIRECTORIES)

include(GNUInstallDirs)

# Create project main executable target
add_executable(${MAESTRO_BASE_TARGET}
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/maestrobase.cpp
)
# Link project executable to util libraries
target_link_libraries(${MAESTRO_BASE_TARGET}
	${rclcpp_LIBRARIES}
	${COREUTILS_LIBRARY}
	${COMMON_LIBRARY}
)
target_include_directories(${MAESTRO_BASE_TARGET} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${COREUTILS_HEADERS}
	${COMMON_HEADERS}
)

# ROS specific rules
rosidl_target_interfaces(${MAESTRO_BASE_TARGET}
	maestro_msgs "rosidl_typesupport_cpp"
)
ament_target_dependencies(${MAESTRO_BASE_TARGET}
  rclcpp
  std_msgs
)

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${MAESTRO_BASE_TARGET}\")")
install(TARGETS ${MAESTRO_BASE_TARGET} 
	RUNTIME DESTINATION "${CMAKE_INSTALL_LIBDIR}/maestro"
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

