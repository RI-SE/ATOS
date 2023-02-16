cmake_minimum_required(VERSION 3.1)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(TCPUDPSocket LANGUAGES C CXX)

set(SOCKET_TARGET ${TARGET_NAMESPACE}${PROJECT_NAME})

include(GNUInstallDirs)

cmake_policy(SET CMP0079 NEW)
# When building all packages in util top CMakeLists not only this lib we need to set this cmake policy to OLD due to target namespace are not unique in the same build tree in util
# ref for info about the policy https://cmake.org/cmake/help/latest/policy/CMP0002.html 
cmake_policy(SET CMP0002 OLD)

add_executable(TEST_CAN
	test_can.cpp
)

add_executable(TEST_SOCKET
	test_socket.cpp
)

add_library(${SOCKET_TARGET} SHARED
	${CMAKE_CURRENT_SOURCE_DIR}/canhandler.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/socket.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/client.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/server.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/tcphandler.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/udphandler.cpp
)

target_include_directories(${SOCKET_TARGET} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

set_property(TARGET ${SOCKET_TARGET} APPEND PROPERTY
	PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/canhandler.hpp
)
set_property(TARGET ${SOCKET_TARGET} APPEND PROPERTY
	PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/socket.hpp
)
set_property(TARGET ${SOCKET_TARGET} APPEND PROPERTY
	PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/server.hpp
)
set_property(TARGET ${SOCKET_TARGET} APPEND PROPERTY
	PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/client.hpp
)
set_property(TARGET ${SOCKET_TARGET} APPEND PROPERTY
	PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/socketexceptions.hpp
)
set_property(TARGET ${SOCKET_TARGET} APPEND PROPERTY
	PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/tcphandler.hpp
)
set_property(TARGET ${SOCKET_TARGET} APPEND PROPERTY
	PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/udphandler.hpp
)

target_include_directories(TEST_CAN PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(TEST_CAN
	${SOCKET_TARGET}
)
if (${CMAKE_SYSTEM_NAME} STREQUAL "Android")
target_link_libraries(TEST_SOCKET
	${SOCKET_TARGET}
)
else()
target_link_libraries(TEST_SOCKET
        ${SOCKET_TARGET}
        pthread
)
endif()


install(CODE "MESSAGE(STATUS \"Installing target ${SOCKET_TARGET}\")")
install(TARGETS ${SOCKET_TARGET}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
