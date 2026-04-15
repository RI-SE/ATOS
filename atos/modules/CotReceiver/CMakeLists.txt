cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(cot_receiver)

find_package(atos_interfaces REQUIRED)

set(MODULE_TARGET ${PROJECT_NAME})

set(COREUTILS_LIBRARY ATOSCoreUtil)
set(COMMON_LIBRARY ATOSCommon)

get_target_property(COMMON_HEADERS ${COMMON_LIBRARY} INCLUDE_DIRECTORIES)
get_target_property(COREUTILS_HEADERS ${COREUTILS_LIBRARY} INCLUDE_DIRECTORIES)

include(GNUInstallDirs)

add_executable(${MODULE_TARGET}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/CotReceiver.cpp
)

target_link_libraries(${MODULE_TARGET}
    ${COREUTILS_LIBRARY}
    ${COMMON_LIBRARY}
)

target_include_directories(${MODULE_TARGET} PUBLIC SYSTEM
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${COMMON_HEADERS}
    ${COREUTILS_HEADERS}
)

ament_target_dependencies(${MODULE_TARGET}
    rclcpp
    std_msgs
    atos_interfaces
)

install(CODE "MESSAGE(STATUS \"Installing target ${MODULE_TARGET}\")")
install(TARGETS ${MODULE_TARGET}
    RUNTIME DESTINATION "${CMAKE_INSTALL_LIBDIR}/atos"
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
