cmake_minimum_required(VERSION 3.1)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
project(Core LANGUAGES C CXX)

# Specify locations for logs and shared memory
# WARNING: These are used for installation and could hose
# 	your system if e.g. specified to "/"
set(SYSTEM_SHARED_MEMORY_PATH "/dev/shm/maestro")

set(CORE_TARGET ${PROJECT_NAME})
set(COREUTILS_TARGET MaestroCoreUtil)
set(JOURNAL_CONTROL_TARGET JournalControl)
set(DATA_DICTIONARY_TARGET MaestroDataDictionary)

set(TIME_LIBRARY MaestroTime)
set(LOGGING_LIBRARY MaestroLogging)
set(MESSAGE_BUS_LIBRARY MaestroMQ)
set(SHARED_MEMORY_LIBRARY MaestroSHM)
set(ISO_22133_LIBRARY ISO22133)
set(MATH_LIBRARY m)
set(FILESYSTEM_LIBRARY stdc++fs)
set(PTHREAD_LIBRARY pthread)

# Get header directories for build interface
get_target_property(TIME_HEADERS ${TIME_LIBRARY} INCLUDE_DIRECTORIES)
get_target_property(LOGGING_HEADERS ${LOGGING_LIBRARY} INCLUDE_DIRECTORIES)
get_target_property(MESSAGE_BUS_HEADERS ${MESSAGE_BUS_LIBRARY} INCLUDE_DIRECTORIES)
get_target_property(SHARED_MEMORY_HEADERS ${SHARED_MEMORY_LIBRARY} INCLUDE_DIRECTORIES)
get_target_property(ISO_22133_HEADERS ${ISO_22133_LIBRARY} INCLUDE_DIRECTORIES)

include(GNUInstallDirs)

if(TurnOffObjectControl STREQUAL "No")
		add_executable(${CORE_TARGET}
			${CMAKE_CURRENT_SOURCE_DIR}/src/objectcontrol.c
			${CMAKE_CURRENT_SOURCE_DIR}/src/main.c
			${CMAKE_CURRENT_SOURCE_DIR}/src/systemcontrol.c
			${CMAKE_CURRENT_SOURCE_DIR}/src/timecontrol.c
			)
endif()

if(TurnOffObjectControl STREQUAL "Yes")
		add_definitions(-DTURN_OFF_OBJECT_CONTROL)
		add_executable(${CORE_TARGET}
			${CMAKE_CURRENT_SOURCE_DIR}/src/main.c
			${CMAKE_CURRENT_SOURCE_DIR}/src/systemcontrol.c
			${CMAKE_CURRENT_SOURCE_DIR}/src/timecontrol.c
			)
endif()

add_library(${JOURNAL_CONTROL_TARGET} SHARED
	${CMAKE_CURRENT_SOURCE_DIR}/src/journalcontrol.cpp

)
target_include_directories(${JOURNAL_CONTROL_TARGET} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
	${LOGGING_HEADERS}
)

add_library(${COREUTILS_TARGET} SHARED
	${CMAKE_CURRENT_SOURCE_DIR}/src/util.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/journal.cpp
)
target_include_directories(${COREUTILS_TARGET} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
	${TIME_HEADERS}
	${LOGGING_HEADERS}
	${MESSAGE_BUS_HEADERS}
	${ISO_22133_HEADERS}
)
set_target_properties(${COREUTILS_TARGET} PROPERTIES
	PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/inc/util.h
)
set_property(TARGET ${COREUTILS_TARGET} APPEND PROPERTY
	PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/inc/journal.h
)

add_library(${DATA_DICTIONARY_TARGET} SHARED
	${CMAKE_CURRENT_SOURCE_DIR}/src/datadictionary.c
)
target_include_directories(${DATA_DICTIONARY_TARGET} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
	${LOGGING_HEADERS}
	${MESSAGE_BUS_HEADERS}
	${SHARED_MEMORY_HEADERS}
)
set_target_properties(${DATA_DICTIONARY_TARGET} PROPERTIES
	PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/inc/datadictionary.h
)

# Linking
target_link_libraries(${CORE_TARGET} LINK_PUBLIC
	${TIME_LIBRARY}
	${LOGGING_LIBRARY}
	${JOURNAL_CONTROL_TARGET}
	${COREUTILS_TARGET}
	${DATA_DICTIONARY_TARGET}
	${ISO_22133_LIBRARY}
	${SYSTEMD_LIBRARIES}
	${PTHREAD_LIBRARY}
)

target_link_libraries(${COREUTILS_TARGET} LINK_PUBLIC
	${TIME_LIBRARY}
	${LOGGING_LIBRARY}
	${MESSAGE_BUS_LIBRARY}
	${MATH_LIBRARY}
	${ISO_22133_LIBRARY}
)

target_link_libraries(${DATA_DICTIONARY_TARGET} LINK_PUBLIC
	${SHARED_MEMORY_LIBRARY}
	${COREUTILS_TARGET}
)

target_link_libraries(${JOURNAL_CONTROL_TARGET} LINK_PUBLIC
	${FILESYSTEM_LIBRARY}
	${COREUTILS_TARGET}
	${DATA_DICTIONARY_TARGET}
)

# Generate service file with correct details
message(STATUS "Configuring ${PROJECT_NAME} service file")
list(LENGTH ENABLED_MODULES NUMBER_ENABLED_MODULES)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/linux/maestro-core.service.in
	${PROJECT_BINARY_DIR}/maestro-core.service
)

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${CORE_TARGET}\")")
install(TARGETS ${CORE_TARGET} ${COREUTILS_TARGET}
		${DATA_DICTIONARY_TARGET} ${JOURNAL_CONTROL_TARGET}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Give built target ability to set system clock
install(CODE "MESSAGE(STATUS \"Giving system time setting capabilities to target ${CORE_TARGET}\")")
install(CODE "execute_process(COMMAND setcap CAP_SYS_TIME+ep ${CMAKE_INSTALL_FULL_BINDIR}/${CORE_TARGET})")

# Create shared memory directory and give user access
install(CODE "MESSAGE(STATUS \"Creating shared memory directory ${SYSTEM_SHARED_MEMORY_PATH} owned by user $ENV{USER}\")")
install(DIRECTORY DESTINATION ${SYSTEM_SHARED_MEMORY_PATH})
install(CODE "execute_process(COMMAND chown $ENV{USER} ${SYSTEM_SHARED_MEMORY_PATH})")

# Install the service
install(CODE "MESSAGE(STATUS \"Installing ${PROJECT_NAME} service to ${CMAKE_INSTALL_FULL_LIBDIR}/systemd/system\")")
install(FILES ${PROJECT_BINARY_DIR}/maestro-core.service
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/systemd/system
  COMPONENT service
)
