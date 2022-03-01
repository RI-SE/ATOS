cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(journal_control)

find_package(rclcpp REQUIRED)

# Define target names
set(JOURNAL_CONTROL_TARGET ${PROJECT_NAME})

set(LOGGING_LIBRARY MaestroLogging)
set(COREUTILS_LIBRARY MaestroCoreUtil)
set(FILESYSTEM_LIBRARY stdc++fs)

include(GNUInstallDirs)

# Create project main executable target
add_executable(${JOURNAL_CONTROL_TARGET}
	${CMAKE_CURRENT_SOURCE_DIR}/src/journalcollection.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/journalcontrol.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/journal.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
)
# Link project executable to util libraries
target_link_libraries(${JOURNAL_CONTROL_TARGET}
	${rclcpp_LIBRARIES}
	${LOGGING_LIBRARY}
	${FILESYSTEM_LIBRARY}
	${COREUTILS_LIBRARY}
)
target_include_directories(${JOURNAL_CONTROL_TARGET} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${COREUTILS_HEADERS}
)

# ROS specific rules
rosidl_target_interfaces(${JOURNAL_CONTROL_TARGET}
	maestro_msgs "rosidl_typesupport_cpp"
)
ament_target_dependencies(${JOURNAL_CONTROL_TARGET}
  rclcpp
  std_msgs
)

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${JOURNAL_CONTROL_TARGET}\")")
install(TARGETS ${JOURNAL_CONTROL_TARGET} 
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

