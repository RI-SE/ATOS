cmake_minimum_required(VERSION 3.10)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_VERBOSE_MAKEFILE OFF CACHE BOOL "Enable verbose make")
set(CMAKE_COLOR_MAKEFILE   ON)

project(maestro VERSION 0.5.0)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

# Enable or disable modules
set(WITH_SCENARIO_CONTROL ON CACHE BOOL "Enable ScenarioControl module")
set(WITH_SUPERVISION ON CACHE BOOL "Enable Supervision module")
set(WITH_VISUALIZATION OFF CACHE BOOL "Enable Visualization module")
set(WITH_RELATIVE_KINEMATICS ON CACHE BOOL "Enable RelativeKinematics module")
set(WITH_DIRECT_CONTROL OFF CACHE BOOL "Enable DirectControl module")
set(WITH_OBJECT_MONITORING OFF CACHE BOOL "Enable ObjectMonitoring module")
set(WITH_BACK_TO_START OFF CACHE BOOL "Enable BackToStart module")

set(ENABLE_TESTS ON CACHE BOOL "Enable testing on build")

set(MAESTRO_TEST_DIR ".maestro" CACHE STRING "Directory where test data is to be stored")

set(MAESTRO_JOURNAL_DIR "${MAESTRO_TEST_DIR}/journal")
set(MAESTRO_TRAJ_DIR "${MAESTRO_TEST_DIR}/traj")
set(MAESTRO_GEOFENCE_DIR "${MAESTRO_TEST_DIR}/geofence")
set(MAESTRO_OBJECTS_DIR "${MAESTRO_TEST_DIR}/objects")

# Dependencies
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/util/cmake)
find_package(Systemd REQUIRED)

# Create list of all enabled modules
if(WITH_SCENARIO_CONTROL)
	list(APPEND ENABLED_MODULES ScenarioControl)
endif()
if(WITH_SUPERVISION)
	list(APPEND ENABLED_MODULES Supervision)
endif()
if(WITH_VISUALIZATION)
	list(APPEND ENABLED_MODULES Visualization)
endif()
if(WITH_DIRECT_CONTROL)
	list(APPEND ENABLED_MODULES DirectControl)
endif()
if(WITH_OBJECT_MONITORING)
	list(APPEND ENABLED_MODULES ObjectMonitoring)
endif()
if(WITH_BACK_TO_START)
	list(APPEND ENABLED_MODULES BackToStart)
endif()
if(WITH_RELATIVE_KINEMATICS)
	list(APPEND ENABLED_MODULES RelativeKinematics)
	set(WITH_OBJECT_CONTROL OFF)
else()
	set(WITH_OBJECT_CONTROL ON)
endif()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES maestro
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
	${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/maestro.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/maestro_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

# Add corresponding subprojects
add_subdirectory(util/C/time)
add_subdirectory(util/C/logging)
add_subdirectory(util/C/mqbus)
add_subdirectory(util/C/shmem)
add_subdirectory(util/C/iso22133)
add_subdirectory(util/C/mathutils)
add_subdirectory(util/C/osihandler)
add_subdirectory(common)
add_subdirectory(core)

foreach(MODULE ${ENABLED_MODULES})
	add_subdirectory(modules/${MODULE})
endforeach()

# Ensure .maestro directory is created at build time in build directory
add_custom_target(configurationDirectory ALL
	COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/conf
		${CMAKE_BINARY_DIR}/${MAESTRO_CONFIGURATION_DIR}
	)
add_custom_target(buildTimeDirectory ALL
	COMMAND ${CMAKE_COMMAND} -E make_directory
		${CMAKE_BINARY_DIR}/${MAESTRO_JOURNAL_DIR}
		${CMAKE_BINARY_DIR}/${MAESTRO_TRAJ_DIR}
		${CMAKE_BINARY_DIR}/${MAESTRO_GEOFENCE_DIR}
		${CMAKE_BINARY_DIR}/${MAESTRO_OBJECTS_DIR}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html
# Configure structure of output
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CATKIN_PACKAGE_LIB_DESTINATION})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CATKIN_PACKAGE_LIB_DESTINATION})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CATKIN_PACKAGE_BIN_DESTINATION})


## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


# Create directory for test data in user home directory
install(CODE "MESSAGE(STATUS \"Creating home directory environment under $ENV{HOME}\")")
install(DIRECTORY DESTINATION "$ENV{HOME}/${MAESTRO_TEST_DIR}")
install(DIRECTORY DESTINATION "$ENV{HOME}/${MAESTRO_JOURNAL_DIR}")
install(DIRECTORY DESTINATION "$ENV{HOME}/${MAESTRO_TRAJ_DIR}")
install(DIRECTORY DESTINATION "$ENV{HOME}/${MAESTRO_GEOFENCE_DIR}")
install(DIRECTORY DESTINATION "$ENV{HOME}/${MAESTRO_OBJECTS_DIR}")
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/conf DESTINATION "$ENV{HOME}/${MAESTRO_TEST_DIR}")

install(CODE "execute_process(COMMAND chown -R $ENV{USER} $ENV{HOME}/${MAESTRO_TEST_DIR})")

# Add post install instructions
add_subdirectory(cmake/postinstall)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_maestro.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

# Testing
#if(ENABLE_TESTS)
#	enable_testing()
#endif()
