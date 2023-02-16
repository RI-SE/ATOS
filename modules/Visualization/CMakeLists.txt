# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

cmake_minimum_required(VERSION 3.8)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(visualization LANGUAGES C CXX)

# Define target names
set(VISUALIZATION_TARGET ${PROJECT_NAME})

set(COREUTILS_LIBRARY ATOSCoreUtil)
set(ISO_22133_LIBRARY ISO22133)
set(TIME_LIBRARY MaestroTime)
set(LOGGING_LIBRARY MaestroLogging)
set(MESSAGE_BUS_LIBRARY MaestroMQ)
set(TCP_UDP_Socket_LIBRARY TCPUDPSocket)
include(GNUInstallDirs)

find_package(Systemd REQUIRED)

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

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${VISUALIZATION_TARGET}\")")
install(TARGETS ${VISUALIZATION_TARGET} 
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
