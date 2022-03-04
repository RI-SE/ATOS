cmake_minimum_required(VERSION 3.8)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(visualization LANGUAGES C CXX)

# Define target names
set(VISUALIZATION_TARGET ${PROJECT_NAME})

set(COREUTILS_LIBRARY MaestroCoreUtil)
set(ISO_22133_LIBRARY ISO22133)
set(TIME_LIBRARY MaestroTime)
set(LOGGING_LIBRARY MaestroLogging)
set(MESSAGE_BUS_LIBRARY MaestroMQ)
set(TCP_UDP_Socket_LIBRARY TCPUDPSocket)
include(GNUInstallDirs)

# Create project main executable target
add_executable(${VISUALIZATION_TARGET}
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/trajfilehandler.cpp
)

# Link project executable to util libraries
target_link_libraries(${VISUALIZATION_TARGET} LINK_PUBLIC
	${COREUTILS_LIBRARY}
	${ISO_22133_LIBRARY}
	${POSITIONING_LIBRARY}
	${TIME_LIBRARY}
	${LOGGING_LIBRARY}
	${MESSAGE_BUS_LIBRARY}
	${SYSTEMD_LIBRARIES}
	${TCP_UDP_Socket_LIBRARY}
	stdc++fs
)

target_include_directories(${VISUALIZATION_TARGET} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# Generate service file with correct details
message(STATUS "Configuring ${PROJECT_NAME} service file")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/linux/maestro-visualization.service.in
	${PROJECT_BINARY_DIR}/maestro-visualization.service
)

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${VISUALIZATION_TARGET}\")")
install(TARGETS ${VISUALIZATION_TARGET} 
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install the service
install(CODE "MESSAGE(STATUS \"Installing ${PROJECT_NAME} service to ${CMAKE_INSTALL_FULL_LIBDIR}/systemd/system\")")
install(FILES ${PROJECT_BINARY_DIR}/maestro-visualization.service
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/systemd/system
  COMPONENT service
)
