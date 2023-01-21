cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)


project(system_control)
find_package(rclcpp REQUIRED)
find_package(atos_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# Define target names
set(SYSTEM_CONTROL_TARGET ${PROJECT_NAME})

set(COREUTILS_LIBRARY ATOSCoreUtil)
set(TIME_LIBRARY MaestroTime)
set(LOGGING_LIBRARY MaestroLogging)
set(ATOS_COMMON_LIBRARY ATOSCommon)

include(GNUInstallDirs)

# Create project main executable target
add_executable(${SYSTEM_CONTROL_TARGET}
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/systemcontrol.cpp
)

ament_target_dependencies(${SYSTEM_CONTROL_TARGET}
  rclcpp
  std_msgs
  atos_interfaces
)
  
target_link_libraries(${SYSTEM_CONTROL_TARGET}
	${TIME_LIBRARY}
	${COREUTILS_LIBRARY}
	${LOGGING_LIBRARY}
	${ATOS_COMMON_LIBRARY}
)

target_include_directories(${SYSTEM_CONTROL_TARGET} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${TIME_HEADERS}
)

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${SYSTEM_CONTROL_TARGET}\")")

install(TARGETS ${SYSTEM_CONTROL_TARGET}
	RUNTIME DESTINATION "${CMAKE_INSTALL_LIBDIR}/atos"
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

