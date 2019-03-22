cmake_minimum_required (VERSION 2.6)
project (ChronosProject)

# copy needed file
#configure_file(traj/0.traj traj/192.168.0.1 COPYONLY)
#configure_file(traj/1.traj traj/192.168.0.2 COPYONLY)
#configure_file(conf/test.conf conf/test.conf COPYONLY)

# add include directories
include_directories(inc)
include_directories(../util/C/logging)
include(GNUInstallDirs)

# Add logging library (hint at some locations)
find_library(MAESTRO_LOGGING_PATH
    NAMES MaestroLogging
    PATHS "../util/C/logging"
          "../util/C"
          ${CMAKE_INSTALL_LIBDIR}
          ${CMAKE_INSTALL_INCLUDEDIR})
add_library(MaestroLogging SHARED IMPORTED)
set_property(TARGET MaestroLogging PROPERTY IMPORTED_LOCATION ${MAESTRO_LOGGING_PATH})

# Create library
add_library(util
  src/util.c
  inc/util.h
)

# Create library
add_library(objctrl
  src/objectcontrol.c
  inc/objectcontrol.h
)

add_executable(UserControl
  src/usercontrol.c
)

# add the executable
add_executable(TEServer
  src/main.c
  src/logger.c
  src/objectcontrol.c
  src/supervision.c
  src/systemcontrol.c
  src/timecontrol.c
  src/supervisorcontrol.c
  inc/logger.h
  inc/objectcontrol.h
  inc/supervision.h
  inc/systemcontrol.h
  inc/timecontrol.h

  inc/supervisorcontrol.h
)

# add the install targets
install (TARGETS TEServer DESTINATION bin)

# add linking to message queue
target_link_libraries(util rt m)
target_link_libraries(util m)
target_link_libraries(util rt)
target_link_libraries(UserControl util objctrl MaestroLogging)
target_link_libraries(util MaestroLogging)
target_link_libraries(TEServer util)
target_link_libraries(TEServer MaestroLogging)
