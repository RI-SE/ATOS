cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(ScenarioControl LANGUAGES C CXX)

# Define target names
set(SCENARIO_CONTROL_TARGET ${PROJECT_NAME})

set(COREUTILS_LIBRARY MaestroCoreUtil)
set(DATA_DICTIONARY_LIBRARY MaestroDataDictionary)
set(ISO_22133_LIBRARY ISO22133)
set(TIME_LIBRARY MaestroTime)
set(LOGGING_LIBRARY MaestroLogging)
set(MESSAGE_BUS_LIBRARY MaestroMQ)

include(GNUInstallDirs)

# Create project main executable target
add_executable(${SCENARIO_CONTROL_TARGET}
	${CMAKE_CURRENT_SOURCE_DIR}/src/action.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/booleantrigger.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/braketrigger.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/distancetrigger.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/causality.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/externalaction.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/isotrigger.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/scenario.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/trigger.cpp
)

# Link project executable to util libraries
target_link_libraries(${SCENARIO_CONTROL_TARGET} LINK_PUBLIC
	${TIME_LIBRARY}
	${COREUTILS_LIBRARY}
        ${DATA_DICTIONARY_LIBRARY}
	${LOGGING_LIBRARY}
	${MESSAGE_BUS_LIBRARY}
	${POSITIONING_LIBRARY}
	${ISO_22133_LIBRARY}
	${SYSTEMD_LIBRARIES}
)
target_include_directories(${SCENARIO_CONTROL_TARGET} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# Generate service file with correct details
message(STATUS "Configuring ${PROJECT_NAME} service file")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/linux/maestro-scenario-control.service.in
	${PROJECT_BINARY_DIR}/maestro-scenario-control.service
)

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${SCENARIO_CONTROL_TARGET}\")")
install(TARGETS ${SCENARIO_CONTROL_TARGET} 
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install the service
install(CODE "MESSAGE(STATUS \"Installing ${PROJECT_NAME} service to ${CMAKE_INSTALL_FULL_LIBDIR}/systemd/system\")")
install(FILES ${PROJECT_BINARY_DIR}/maestro-scenario-control.service
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/systemd/system
  COMPONENT service
)

