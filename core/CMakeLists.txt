cmake_minimum_required(VERSION 3.8)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
project(Core LANGUAGES C CXX)

# Specify locations for logs and shared memory
# WARNING: These are used for installation and could hose
# 	your system if e.g. specified to "/"

set(CORE_TARGET ${PROJECT_NAME})

set(COREUTILS_LIBRARY MaestroCoreUtil)
set(COMMON_LIBRARY MaestroCommon)
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
get_target_property(COREUTILS_HEADERS ${COREUTILS_LIBRARY} INCLUDE_DIRECTORIES)
get_target_property(COMMON_HEADERS ${COMMON_LIBRARY} INCLUDE_DIRECTORIES)

include(GNUInstallDirs)


if(WITH_OBJECT_CONTROL)
    add_executable(${CORE_TARGET}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/objectcontrol.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c
    )
else()
    message(STATUS "Excluding ObjectControl")
    add_definitions(-DDISABLE_OBJECT_CONTROL)
    add_executable(${CORE_TARGET}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c
    )
endif()

target_include_directories(${CORE_TARGET} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc)

# Linking
target_link_libraries(${CORE_TARGET} LINK_PUBLIC
	${TIME_LIBRARY}
	${LOGGING_LIBRARY}
	${JOURNAL_CONTROL_TARGET}
	${COREUTILS_LIBRARY}
	${ISO_22133_LIBRARY}
	${SYSTEMD_LIBRARIES}
	${PTHREAD_LIBRARY}
)

# Generate service file with correct details
message(STATUS "Configuring ${PROJECT_NAME} service file")
list(LENGTH ENABLED_MODULES NUMBER_ENABLED_MODULES)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/linux/maestro-core.service.in
	${PROJECT_BINARY_DIR}/maestro-core.service
)

# Installation rules
# Give built target ability to set system clock
install(CODE "MESSAGE(STATUS \"Giving system time setting capabilities to target ${CORE_TARGET}\")")
install(CODE "execute_process(COMMAND setcap CAP_SYS_TIME+ep ${CMAKE_INSTALL_FULL_BINDIR}/${CORE_TARGET})")

# Install the service
install(CODE "MESSAGE(STATUS \"Installing ${PROJECT_NAME} service to ${CMAKE_INSTALL_FULL_LIBDIR}/systemd/system\")")
install(FILES ${PROJECT_BINARY_DIR}/maestro-core.service
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/systemd/system
  COMPONENT service
)

install(CODE "MESSAGE(STATUS \"Installing target ${CORE_TARGET}\")")
install(TARGETS ${CORE_TARGET}
    RUNTIME DESTINATION "${CMAKE_INSTALL_LIBDIR}/maestro"
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
