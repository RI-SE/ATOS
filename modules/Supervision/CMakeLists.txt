cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(Supervision LANGUAGES C CXX)

# Define target names
set(SUPERVISION_TARGET ${PROJECT_NAME})

set(COREUTILS_LIBRARY MaestroCoreUtil)
set(DATA_DICTIONARY_LIBRARY MaestroDataDictionary)
set(ISO_22133_LIBRARY ISO22133)
set(TIME_LIBRARY MaestroTime)
set(LOGGING_LIBRARY MaestroLogging)
set(MESSAGE_BUS_LIBRARY MaestroMQ)

include(GNUInstallDirs)

# Create project main executable target
add_executable(${SUPERVISION_TARGET}
	${CMAKE_CURRENT_SOURCE_DIR}/src/geofence.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/supervisionstate.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/trajectory.cpp
)

# Link project executable to util libraries
target_link_libraries(${SUPERVISION_TARGET} LINK_PUBLIC
	${TIME_LIBRARY}
	${COREUTILS_LIBRARY}
        ${DATA_DICTIONARY_LIBRARY}
	${ISO_22133_LIBRARY}
        ${POSITIONING_LIBRARY}
        ${LOGGING_LIBRARY}
	${SYSTEMD_LIBRARIES}
)
target_include_directories(${SUPERVISION_TARGET} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# Generate service file with correct details
message(STATUS "Configuring ${PROJECT_NAME} service file")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/linux/maestro-supervision.service.in
	${PROJECT_BINARY_DIR}/maestro-supervision.service
)

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${SUPERVISION_TARGET}\")")
install(TARGETS ${SUPERVISION_TARGET} 
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install the service
install(CODE "MESSAGE(STATUS \"Installing ${PROJECT_NAME} service to ${CMAKE_INSTALL_FULL_LIBDIR}/systemd/system\")")
install(FILES ${PROJECT_BINARY_DIR}/maestro-supervision.service
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/systemd/system
  COMPONENT service
)
