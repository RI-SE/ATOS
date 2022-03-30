cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(scenario_control LANGUAGES C CXX)

find_package(rclcpp REQUIRED)
find_package(maestro_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(xodr CONFIG REQUIRED)

set(AD_XOLIB_LIB_PATH "/usr/local/lib/ad-xolib")
set(AD_XOLIB_INCLUDE_PATH "/usr/local/include/ad-xolib")

find_library(XOSC_LIBRARY
NAMES xosc
HINTS "${AD_XOLIB_LIB_PATH}"
)
find_library(XODR_LIBRARY
NAMES xodr
HINTS "${AD_XOLIB_LIB_PATH}"
)
find_path(AD_XOLIB_INCLUDE
NAMES xosc.h xodr.h common.h 
PATHS "${AD_XOLIB_INCLUDE_PATH}"
)

#find_package(xodr CONFIG REQUIRED)
#find_library(XOSC xosc REQUIRED)
#if(NOT XODR_LIBRARY)
#  message(FATAL_ERROR "library not found")
#endif()

# Define target names
set(SCENARIO_CONTROL_TARGET ${PROJECT_NAME})

set(COREUTILS_LIBRARY MaestroCoreUtil)
set(ISO_22133_LIBRARY ISO22133)
set(COMMON_LIBRARY MaestroCommon)

get_target_property(COMMON_HEADERS ${COMMON_LIBRARY} INCLUDE_DIRECTORIES)

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
	${CMAKE_CURRENT_SOURCE_DIR}/src/scenariocontrol.cpp
)

# Link project executable to util libraries
target_link_libraries(${SCENARIO_CONTROL_TARGET}
	${rclcpp_LIBRARIES}
	${COREUTILS_LIBRARY}
	${POSITIONING_LIBRARY}
	${ISO_22133_LIBRARY}
	${COMMON_LIBRARY}
	${XODR_LIBRARY}
	${XOSC_LIBRARY}

)
target_include_directories(${SCENARIO_CONTROL_TARGET} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${COMMON_HEADERS}
	${AD_XOLIB_INCLUDE}
)

# ROS specific rules
ament_target_dependencies(${SCENARIO_CONTROL_TARGET}
  rclcpp
  std_msgs
  maestro_interfaces
)

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${SCENARIO_CONTROL_TARGET}\")")
install(TARGETS ${SCENARIO_CONTROL_TARGET} 
	RUNTIME DESTINATION "${CMAKE_INSTALL_LIBDIR}/maestro"
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

