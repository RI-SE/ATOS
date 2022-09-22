cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)



project(driver_model)

find_package(rclcpp REQUIRED)
find_package(maestro_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)


# Define target names
set(OBJECT_CONTROL_TARGET ${PROJECT_NAME})

set(COREUTILS_LIBRARY MaestroCoreUtil)
set(ISO_22133_LIBRARY ISO22133)
set(TIME_LIBRARY MaestroTime)
set(LOGGING_LIBRARY MaestroLogging)
set(MAESTRO_COMMON_LIBRARY MaestroCommon)
set(FILESYSTEM_LIBRARY stdc++fs)
set(THREAD_LIBRARY pthread)
set(OSI_LIBRARY OSIHandler)

get_target_property(OSI_HEADERS ${OSI_LIBRARY} INCLUDE_DIRECTORIES)
get_target_property(TIME_HEADERS ${TIME_LIBRARY} INCLUDE_DIRECTORIES)

include(GNUInstallDirs)

# Create project main executable target
add_executable(${OBJECT_CONTROL_TARGET}
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/osiadapter.cpp
)
# Link project executable to util libraries
target_link_libraries(${OBJECT_CONTROL_TARGET} 
	${TIME_LIBRARY}
	${COREUTILS_LIBRARY}
	${LOGGING_LIBRARY}
	${FILESYSTEM_LIBRARY}
	${THREAD_LIBRARY}
	${MAESTRO_COMMON_LIBRARY}
	${ISO_22133_LIBRARY}
	${OSI_LIBRARY}
)

# ROS specific settings
ament_target_dependencies(${OBJECT_CONTROL_TARGET}
  rclcpp
  std_msgs
  nav_msgs
  maestro_interfaces
  tf2
)

target_include_directories(${OBJECT_CONTROL_TARGET} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${OSI_HEADERS}
	${TIME_HEADERS}
)

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${OBJECT_CONTROL_TARGET}\")")
install(TARGETS ${OBJECT_CONTROL_TARGET}
	RUNTIME DESTINATION "${CMAKE_INSTALL_LIBDIR}/maestro"
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)