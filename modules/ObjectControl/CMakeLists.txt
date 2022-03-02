cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(ObjectControl)

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
	${CMAKE_CURRENT_SOURCE_DIR}/src/objectcontrol.cpp
)
# Link project executable to util libraries
target_link_libraries(${OBJECT_CONTROL_TARGET} LINK_PUBLIC
	${TIME_LIBRARY}
	${COREUTILS_LIBRARY}
	${LOGGING_LIBRARY}
	${FILESYSTEM_LIBRARY}
	${THREAD_LIBRARY}
	${MAESTRO_COMMON_LIBRARY}
	${ISO_22133_LIBRARY}
	${OSI_LIBRARY}
)
ament_target_dependencies(${OBJECT_CONTROL_TARGET} PUBLIC
  rclcpp
  std_msgs
  maestro_msgs)

target_include_directories(${OBJECT_CONTROL_TARGET} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${OSI_HEADERS}
	${TIME_HEADERS}
)


# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${OBJECT_CONTROL_TARGET}\")")

#ros installation
get_property(ros_install_dir GLOBAL PROPERTY ros_install_dir_property)

install(TARGETS 
	ObjectControl
 DESTINATION ${ros_install_dir})

install(TARGETS ${OBJECT_CONTROL_TARGET} 
	RUNTIME DESTINATION "${CMAKE_INSTALL_LIBDIR}/maestro"
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
