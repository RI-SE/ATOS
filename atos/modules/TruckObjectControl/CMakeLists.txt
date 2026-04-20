# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(truck_object_control)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenSSL REQUIRED)

include(GNUInstallDirs)

set(COREUTILS_LIBRARY ATOSCoreUtil)

add_executable(${PROJECT_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/truckobjectcontrol.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/inc
  ${CMAKE_CURRENT_SOURCE_DIR}/../../common
)
target_link_libraries(${PROJECT_NAME}
  ${COREUTILS_LIBRARY}
  OpenSSL::SSL
  OpenSSL::Crypto
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  ament_index_cpp
)

add_executable(atos_truck_simulator
  ${CMAKE_CURRENT_SOURCE_DIR}/src/main_simulator.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/atostrucksimulator.cpp
)
target_include_directories(atos_truck_simulator PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/inc
  ${CMAKE_CURRENT_SOURCE_DIR}/../../common
)
target_link_libraries(atos_truck_simulator
  ${COREUTILS_LIBRARY}
)
ament_target_dependencies(atos_truck_simulator
  rclcpp
  std_msgs
  ament_index_cpp
)

install(CODE "MESSAGE(STATUS \"Installing target ${PROJECT_NAME}\")")
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION "${CMAKE_INSTALL_LIBDIR}/atos"
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(CODE "MESSAGE(STATUS \"Installing target atos_truck_simulator\")")
install(TARGETS atos_truck_simulator
  RUNTIME DESTINATION "${CMAKE_INSTALL_LIBDIR}/atos"
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
