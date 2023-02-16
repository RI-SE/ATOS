# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

cmake_minimum_required(VERSION 3.1)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(ATOSTime LANGUAGES C)

set(TIME_TARGET ${TARGET_NAMESPACE}${PROJECT_NAME})

include(GNUInstallDirs)

add_library(${TIME_TARGET} SHARED
	${CMAKE_CURRENT_SOURCE_DIR}/atosTime.c
)

target_include_directories(${TIME_TARGET} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

set_target_properties(${TIME_TARGET} PROPERTIES
	PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/atosTime.h
)

install(CODE "MESSAGE(STATUS \"Installing target ${TIME_TARGET}\")")
install(TARGETS ${TIME_TARGET} 
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
