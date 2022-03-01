cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)


project(system_control)

# Define target names
set(SYSTEM_CONTROL_TARGET ${PROJECT_NAME})

set(COREUTILS_LIBRARY MaestroCoreUtil)
set(TIME_LIBRARY MaestroTime)
set(LOGGING_LIBRARY MaestroLogging)
set(MAESTRO_COMMON_LIBRARY MaestroCommon)

include(GNUInstallDirs)

# Create project main executable target
add_executable(${SYSTEM_CONTROL_TARGET}
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/systemcontrol.cpp
)
# Link project executable to util libraries

rosidl_target_interfaces(${SYSTEM_CONTROL_TARGET}
	maestro_msgs "rosidl_typesupport_cpp"
)
ament_target_dependencies(${SYSTEM_CONTROL_TARGET}
  rclcpp
  std_msgs
)
  
target_link_libraries(${SYSTEM_CONTROL_TARGET}
	${TIME_LIBRARY}
	${COREUTILS_LIBRARY}
	${LOGGING_LIBRARY}
	${MAESTRO_COMMON_LIBRARY}
)

target_include_directories(${SYSTEM_CONTROL_TARGET} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${TIME_HEADERS}
)

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${SYSTEM_CONTROL_TARGET}\")")

install(TARGETS ${SYSTEM_CONTROL_TARGET}
	RUNTIME DESTINATION "${CMAKE_INSTALL_LIBDIR}/maestro"
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Ensure linkage is reloaded after install
install(CODE "execute_process(COMMAND ldconfig)")
