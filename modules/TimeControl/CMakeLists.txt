cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)


project(time_control)

# Define target names
set(TIME_CONTROL_TARGET ${PROJECT_NAME})

set(COREUTILS_LIBRARY MaestroCoreUtil)
set(TIME_LIBRARY MaestroTime)
set(LOGGING_LIBRARY MaestroLogging)
set(MAESTRO_COMMON_LIBRARY MaestroCommon)
set(FILESYSTEM_LIBRARY stdc++fs)
set(THREAD_LIBRARY pthread)


include(GNUInstallDirs)

# Create project main executable target
add_executable(${TIME_CONTROL_TARGET}
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/timecontrol.cpp
)
# Link project executable to util libraries
target_link_libraries(${TIME_CONTROL_TARGET}
	${TIME_LIBRARY}
	${COREUTILS_LIBRARY}
	${LOGGING_LIBRARY}
	${FILESYSTEM_LIBRARY}
	${THREAD_LIBRARY}
	${MAESTRO_COMMON_LIBRARY}
)

rosidl_target_interfaces(${TIME_CONTROL_TARGET}
	maestro_msgs "rosidl_typesupport_cpp"
)
ament_target_dependencies(${TIME_CONTROL_TARGET}
  rclcpp
  std_msgs
)
  
target_include_directories(${TIME_CONTROL_TARGET} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${TIME_HEADERS}
)

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${TIME_CONTROL_TARGET}\")")
install(TARGETS ${TIME_CONTROL_TARGET}
	RUNTIME DESTINATION "${CMAKE_INSTALL_LIBDIR}/maestro"
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
