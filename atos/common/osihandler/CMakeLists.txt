# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(OSIHandler LANGUAGES C CXX)

set(OSI_HANDLER_LIBRARY_TARGET ${TARGET_NAMESPACE}${PROJECT_NAME})

find_package(Protobuf 2.6.1 REQUIRED)
find_package(open_simulation_interface 3.3.1 REQUIRED)
find_package(Eigen3 REQUIRED IMPORTED)

include(GNUInstallDirs)

add_library(${OSI_HANDLER_LIBRARY_TARGET} SHARED
      ${CMAKE_CURRENT_SOURCE_DIR}/osi_handler.cpp
)

# pthread might not be needed depending on your protobuf version. 
target_link_libraries(${OSI_HANDLER_LIBRARY_TARGET} LINK_PUBLIC
	${PROTOBUF_LIBRARY}
	open_simulation_interface
    pthread
	${EIGEN_LIBRARY}
)

target_include_directories(${OSI_HANDLER_LIBRARY_TARGET} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>  	 
)
set_target_properties(${OSI_HANDLER_LIBRARY_TARGET} PROPERTIES
	PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/osi_handler.hpp

)

install(CODE "MESSAGE(STATUS \"Installing target ${OSI_HANDLER_LIBRARY_TARGET}\")")
install(TARGETS ${OSI_HANDLER_LIBRARY_TARGET}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

