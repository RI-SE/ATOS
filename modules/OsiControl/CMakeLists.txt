cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(OsiControl)

# Define target names
set(OSI_TARGET ${PROJECT_NAME})

set(COREUTILS_LIBRARY MaestroCoreUtil)
set(ISO_22133_LIBRARY MaestroISO22133)
set(POSITIONING_LIBRARY MaestroPositioning)
set(TIME_LIBRARY MaestroTime)
set(LOGGING_LIBRARY MaestroLogging)
set(DATA_DICTIONARY_LIBRARY MaestroDataDictionary)
set(MESSAGE_BUS_LIBRARY MaestroMQ)
set(SOCKET_LIBRARY TCPUDPSocket)
set(THREAD_LIBRARY pthread)
set(OSI_LIBRARY OSIHandler)

get_target_property(OSI_HEADERS ${OSI_LIBRARY} INCLUDE_DIRECTORIES)

include(GNUInstallDirs)

# Create project main executable target
add_executable(${OSI_TARGET}
	${CMAKE_CURRENT_SOURCE_DIR}/src/osicontrol.cpp
)
# Link project executable to util libraries
target_link_libraries(${OSI_TARGET} LINK_PUBLIC
	${TIME_LIBRARY}
	${COREUTILS_LIBRARY}
	${LOGGING_LIBRARY}
	${SOCKET_LIBRARY}
	${THREAD_LIBRARY}
	${DATA_DICTIONARY_LIBRARY}
	${OSI_LIBRARY}
)

target_include_directories(${OSI_TARGET} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${OSI_HEADERS}
)

#target_include_directories(${TARGET_NAME} PUBLIC ${OSI_LIB_NAME})


# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${OSI_TARGET}\")")
install(TARGETS ${OSI_TARGET} 
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Ensure linkage is reloaded after install
install(CODE "execute_process(COMMAND ldconfig)")

