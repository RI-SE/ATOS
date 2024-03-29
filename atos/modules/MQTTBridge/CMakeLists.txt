# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

cmake_minimum_required(VERSION 3.10)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(mqtt_bridge)
find_package(rclcpp REQUIRED)
find_package(atos_interfaces REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(PahoMqttCpp REQUIRED)

# Define target names
set(MQTT_BRIDGE_TARGET ${PROJECT_NAME})
set(ATOS_COMMON_LIBRARY ATOSCommon)
get_target_property(COMMON_HEADERS ${ATOS_COMMON_LIBRARY} INCLUDE_DIRECTORIES)

include(GNUInstallDirs)

# Create project main executable target
add_executable(${MQTT_BRIDGE_TARGET}
	${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/mqttclientwrapper.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/mqttsubscribercallback.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/mqttbridge.cpp
)

# Link project executable to util libraries
target_link_libraries(${MQTT_BRIDGE_TARGET}
	${ATOS_COMMON_LIBRARY}
	${nlohmann_json}
	paho-mqttpp3
	paho-mqtt3as
)

target_include_directories(${MQTT_BRIDGE_TARGET} PUBLIC SYSTEM
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	${CMAKE_INSTALL_INCLUDEDIR}
	${COMMON_HEADERS}
)

# ROS specific rules
ament_target_dependencies(${MQTT_BRIDGE_TARGET}
  rclcpp
  atos_interfaces
)

# Installation rules
install(CODE "MESSAGE(STATUS \"Installing target ${MQTT_BRIDGE_TARGET}\")")
install(TARGETS ${MQTT_BRIDGE_TARGET}
	RUNTIME DESTINATION "${CMAKE_INSTALL_LIBDIR}/atos"
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)