cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(relative_kinematics)

# Define target names
set(RELATIVE_KINEMATICS_TARGET ${PROJECT_NAME})

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
add_executable(${RELATIVE_KINEMATICS_TARGET}
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/scenariohandler.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/testobject.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/objectlistener.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/states/state.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/states/idle.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/states/initialized.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/states/connecting.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/states/ready.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/states/aborting.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/states/testlive.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/states/disarming.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/states/armed.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/states/done.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/RelativeKinematicsModule.cpp
)
# Link project executable to util libraries
target_link_libraries(${RELATIVE_KINEMATICS_TARGET}
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
rosidl_target_interfaces(${RELATIVE_KINEMATICS_TARGET}
	maestro_msgs "rosidl_typesupport_cpp"
)
ament_target_dependencies(${RELATIVE_KINEMATICS_TARGET}
  rclcpp
  std_msgs
)

target_include_directories(${RELATIVE_KINEMATICS_TARGET} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${OSI_HEADERS}
	${TIME_HEADERS}
)

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${RELATIVE_KINEMATICS_TARGET}\")")
install(TARGETS ${RELATIVE_KINEMATICS_TARGET}
	RUNTIME DESTINATION "${CMAKE_INSTALL_LIBDIR}/maestro"
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Ensure linkage is reloaded after install
install(CODE "execute_process(COMMAND ldconfig)")

