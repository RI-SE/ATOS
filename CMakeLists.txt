cmake_minimum_required(VERSION 3.10)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_COLOR_MAKEFILE   ON)

project(Maestro)

set(MAESTRO_TEST_DIR ".maestro")
set(MAESTRO_JOURNAL_DIR "${MAESTRO_TEST_DIR}/journal")
set(MAESTRO_TRAJ_DIR "${MAESTRO_TEST_DIR}/traj")
set(MAESTRO_GEOFENCE_DIR "${MAESTRO_TEST_DIR}/geofence")

# Configure structure of output
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_subdirectory(util/C)
add_subdirectory(core)
add_subdirectory(modules/ScenarioControl)
add_subdirectory(modules/Supervision)
add_subdirectory(modules/Visualization)
add_subdirectory(modules/ObjectMonitoring)

# Ensure .maestro directory is created at build time in build directory
add_custom_target(configurationDirectory ALL
	COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/conf
		${CMAKE_BINARY_DIR}/${MAESTRO_CONFIGURATION_DIR}
	)
add_custom_target(buildTimeDirectory ALL
	COMMAND ${CMAKE_COMMAND} -E make_directory [
		${CMAKE_BINARY_DIR}/${MAESTRO_JOURNAL_DIR}
		${CMAKE_BINARY_DIR}/${MAESTRO_TRAJ_DIR}
		${CMAKE_BINARY_DIR}/${MAESTRO_GEOFENCE_DIR}
		]
)

# Create directory for test data in user home directory
install(CODE "MESSAGE(STATUS \"Creating home directory environment under $ENV{HOME}\")")
install(DIRECTORY DESTINATION "$ENV{HOME}/${MAESTRO_TEST_DIR}")
install(DIRECTORY DESTINATION "$ENV{HOME}/${MAESTRO_JOURNAL_DIR}")
install(DIRECTORY DESTINATION "$ENV{HOME}/${MAESTRO_TRAJ_DIR}")
install(DIRECTORY DESTINATION "$ENV{HOME}/${MAESTRO_GEOFENCE_DIR}")
install(DIRECTORY ${CMAKE_SOURCE_DIR}/conf DESTINATION "$ENV{HOME}/${MAESTRO_TEST_DIR}")

install(CODE "execute_process(COMMAND chown -R $ENV{USER} $ENV{HOME}/${MAESTRO_TEST_DIR})")

