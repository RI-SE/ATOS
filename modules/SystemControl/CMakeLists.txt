cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)


project(SystemControl)

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
target_link_libraries(${SYSTEM_CONTROL_TARGET} LINK_PUBLIC
	${TIME_LIBRARY}
	${COREUTILS_LIBRARY}
	${LOGGING_LIBRARY}
	${MAESTRO_COMMON_LIBRARY}
)

ament_target_dependencies(${SYSTEM_CONTROL_TARGET} PUBLIC
  rclcpp
  std_msgs
  maestro_msgs)
  
target_include_directories(${SYSTEM_CONTROL_TARGET} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${TIME_HEADERS}
)

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${SYSTEM_CONTROL_TARGET}\")")

#ros installation
get_property(ros_install_dir GLOBAL PROPERTY ros_install_dir_property)

install(TARGETS 
	SystemControl
 DESTINATION ${ros_install_dir})

# Ensure linkage is reloaded after install
install(CODE "execute_process(COMMAND ldconfig)")
