cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(osi_adapter)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(atos_interfaces REQUIRED)
find_package(Boost REQUIRED)

# Define target names
set(OSI_ADAPTER_TARGET ${PROJECT_NAME})

set(OSI_LIBRARY OSIHandler)
set(ATOS_COMMON_LIBRARY ATOSCommon)

get_target_property(OSI_HEADERS ${OSI_LIBRARY} INCLUDE_DIRECTORIES)

include(GNUInstallDirs)

# Create project main executable target
add_executable(${OSI_ADAPTER_TARGET}
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/osiadapter.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/server.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/tcpserver.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/udpserver.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/serverfactory.cpp
)
# Link project executable to util libraries
target_link_libraries(${OSI_ADAPTER_TARGET} 
	${ATOS_COMMON_LIBRARY}
	${OSI_LIBRARY}
)

# ROS specific settings
ament_target_dependencies(${OSI_ADAPTER_TARGET}
  rclcpp
  std_msgs
  atos_interfaces
)

target_include_directories(${OSI_ADAPTER_TARGET} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${OSI_HEADERS}
	${Boost_INCLUDE_DIRS}
)

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${OSI_ADAPTER_TARGET}\")")
install(TARGETS ${OSI_ADAPTER_TARGET}
	RUNTIME DESTINATION "${CMAKE_INSTALL_LIBDIR}/atos"
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)