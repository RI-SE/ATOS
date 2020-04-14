cmake_minimum_required(VERSION 3.1)
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_C_STANDARD 11)
SET(CMAKE_C_STANDARD_REQUIRED ON)

project(ObjectMonitoring LANGUAGES C CXX)

# Define target names
set(OBJECT_MONITORING_TARGET ${PROJECT_NAME})

set(COREUTILS_LIBRARY MaestroCoreUtil)
set(ISO_22133_LIBRARY MaestroISO22133)
set(POSITIONING_LIBRARY MaestroPositioning)
set(TIME_LIBRARY MaestroTime)
set(LOGGING_LIBRARY MaestroLogging)
set(MESSAGE_BUS_LIBRARY MaestroMQ)

include(GNUInstallDirs)

find_package(Threads REQUIRED)

# Create project main executable target
add_executable(${OBJECT_MONITORING_TARGET}
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/connectionhandler.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/protocoldata.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/ISO22133ProtocolData.cpp
)

# Link project executable to util libraries
target_link_libraries(${OBJECT_MONITORING_TARGET} LINK_PUBLIC
	${TIME_LIBRARY}
	${LOGGING_LIBRARY}
	${COREUTILS_LIBRARY}
	${CMAKE_THREAD_LIBS_INIT}
)

target_include_directories(${OBJECT_MONITORING_TARGET} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${OBJECT_MONITORING_TARGET}\")")
install(TARGETS ${OBJECT_MONITORING_TARGET}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Ensure linkage is reloaded after install
install(CODE "execute_process(COMMAND ldconfig)")
