cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(DirectControl)

# Define target names
set(DIRECT_CONTROL_TARGET ${PROJECT_NAME})

set(COREUTILS_LIBRARY MaestroCoreUtil)
set(ISO_22133_LIBRARY MaestroISO22133)
set(POSITIONING_LIBRARY MaestroPositioning)
set(TIME_LIBRARY MaestroTime)
set(LOGGING_LIBRARY MaestroLogging)
set(MESSAGE_BUS_LIBRARY MaestroMQ)
set(SOCKET_LIBRARY TCPUDPSocket)
set(THREAD_LIBRARY pthread)

include(GNUInstallDirs)

# Create project main executable target
add_executable(${DIRECT_CONTROL_TARGET}
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/directcontrol.cpp
)
# Link project executable to util libraries
target_link_libraries(${DIRECT_CONTROL_TARGET} LINK_PUBLIC
	${TIME_LIBRARY}
	${COREUTILS_LIBRARY}
	${LOGGING_LIBRARY}
	${SOCKET_LIBRARY}
	${THREAD_LIBRARY}
)

ament_target_dependencies(${DIRECT_CONTROL_TARGET} PUBLIC
  rclcpp
  std_msgs
  maestro_msgs)

target_include_directories(${DIRECT_CONTROL_TARGET} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
)


# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${DIRECT_CONTROL_TARGET}\")")

#ros installation
get_property(ros_install_dir GLOBAL PROPERTY ros_install_dir_property)

install(TARGETS 
	${DIRECT_CONTROL_TARGET}
 DESTINATION ${ros_install_dir})

 # Ensure linkage is reloaded after install
install(CODE "execute_process(COMMAND ldconfig)")