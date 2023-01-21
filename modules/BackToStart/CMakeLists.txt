cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(back_to_start)

# Define target names
set(BACK_TO_START ${PROJECT_NAME})

set(COREUTILS_LIBRARY ATOSCoreUtil)
set(ISO_22133_LIBRARY MaestroISO22133)
set(POSITIONING_LIBRARY MaestroPositioning)
set(TIME_LIBRARY MaestroTime)
set(ATOS_COMMON_LIBRARY ATOSCommon)
set(LOGGING_LIBRARY MaestroLogging)
set(MESSAGE_BUS_LIBRARY MaestroMQ)
set(ATOS_COMMON_LIBRARY ATOSCommon)

include(GNUInstallDirs)

# Create project main executable target
add_executable(${BACK_TO_START}
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
)
# Link project executable to util libraries
target_link_libraries(${BACK_TO_START} LINK_PUBLIC
	${TIME_LIBRARY}
	${COREUTILS_LIBRARY}
	${LOGGING_LIBRARY}
        ${FILESYSTEM_LIBRARY}
        ${ATOS_COMMON_LIBRARY}
)
target_include_directories(${BACK_TO_START} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${BACK_TO_START}\")")
install(TARGETS ${BACK_TO_START} 
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)


