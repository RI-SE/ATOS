cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-fpermissive") #temporary


project(SystemControl)

# Define target names
set(SYSTEM_CONTROL_TARGET ${PROJECT_NAME})

set(COREUTILS_LIBRARY MaestroCoreUtil)
set(TIME_LIBRARY MaestroTime)
set(LOGGING_LIBRARY MaestroLogging)
set(MAESTRO_COMMON_LIBRARY MaestroCommon)
set(FILESYSTEM_LIBRARY stdc++fs)
set(THREAD_LIBRARY pthread)


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
	${FILESYSTEM_LIBRARY}
	${THREAD_LIBRARY}
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
install(TARGETS ${SYSTEM_CONTROL_TARGET} 
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Ensure linkage is reloaded after install
install(CODE "execute_process(COMMAND ldconfig)")
