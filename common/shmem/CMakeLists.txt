# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

cmake_minimum_required(VERSION 3.1)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(MaestroSHM LANGUAGES C CXX)

set(SHARED_MEMORY_TARGET ${TARGET_NAMESPACE}${PROJECT_NAME})

include(GNUInstallDirs)

add_library(${SHARED_MEMORY_TARGET} SHARED
	${CMAKE_CURRENT_SOURCE_DIR}/shmem.cpp
)

target_include_directories(${SHARED_MEMORY_TARGET} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

set_target_properties(${SHARED_MEMORY_TARGET} PROPERTIES
	PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/shmem.h
)

target_link_libraries(${SHARED_MEMORY_TARGET} rt pthread)

install(CODE "MESSAGE(STATUS \"Installing target ${SHARED_MEMORY_TARGET}\")")
install(TARGETS ${SHARED_MEMORY_TARGET}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
