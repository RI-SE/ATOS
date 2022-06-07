cmake_minimum_required(VERSION 3.1)
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_C_STANDARD 11)
SET(CMAKE_C_STANDARD_REQUIRED ON)

project(object_monitoring LANGUAGES C CXX)

# Define target names
set(OBJECT_MONITORING_TARGET ${PROJECT_NAME})

set(COREUTILS_LIBRARY MaestroCoreUtil)
set(ISO_22133_LIBRARY ISO22133)
set(TIME_LIBRARY MaestroTime)
set(LOGGING_LIBRARY MaestroLogging)
set(MESSAGE_BUS_LIBRARY MaestroMQ)

include(GNUInstallDirs)

find_package(Threads REQUIRED)
find_library(paho-mqtt3a NAMES libpaho-mqtt3a.so REQUIRED)
find_library(paho-mqtt3c NAMES libpaho-mqtt3c.so REQUIRED)
add_library(pahomqtt3a SHARED IMPORTED)
add_library(pahomqtt3c SHARED IMPORTED)
set_property(TARGET pahomqtt3a PROPERTY IMPORTED_LOCATION ${paho-mqtt3a})
set_property(TARGET pahomqtt3c PROPERTY IMPORTED_LOCATION ${paho-mqtt3c})

# Create project main executable target
add_executable(${OBJECT_MONITORING_TARGET}
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/connectionhandler.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/protocoldata.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/ISO22133ProtocolData.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/mqttconnectionhandler.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/mqtttopichandlers.cpp
)

# Link project executable to util libraries
target_link_libraries(${OBJECT_MONITORING_TARGET} LINK_PUBLIC
	${TIME_LIBRARY}
	${LOGGING_LIBRARY}
	${COREUTILS_LIBRARY}
	${CMAKE_THREAD_LIBS_INIT}
	pahomqtt3a
	pahomqtt3c
	${SYSTEMD_LIBRARIES}
)

target_include_directories(${OBJECT_MONITORING_TARGET} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# Generate service file with correct details
message(STATUS "Configuring ${PROJECT_NAME} service file")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/linux/maestro-object-monitoring.service.in
	${PROJECT_BINARY_DIR}/maestro-object-monitoring.service
)

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${OBJECT_MONITORING_TARGET}\")")
install(TARGETS ${OBJECT_MONITORING_TARGET}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install the service
install(CODE "MESSAGE(STATUS \"Installing ${PROJECT_NAME} service to ${CMAKE_INSTALL_FULL_LIBDIR}/systemd/system\")")
install(FILES ${PROJECT_BINARY_DIR}/maestro-object-monitoring.service
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/systemd/system
  COMPONENT service
)
