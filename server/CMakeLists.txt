cmake_minimum_required (VERSION 2.6)
project (Maestro)

# copy needed file
#configure_file(traj/0.traj traj/192.168.0.1 COPYONLY)
#configure_file(traj/1.traj traj/192.168.0.2 COPYONLY)
#configure_file(conf/test.conf conf/test.conf COPYONLY)

# add include directories
include_directories(inc)
include_directories(../util/C/logging)
include_directories(../util/C/time)
include_directories(../util/C/MQBus)
include_directories(../util/ASN1/generatedfiles)
include(GNUInstallDirs)

# Add logging library (hint at some locations)
#find_library(MAESTRO_LOGGING_PATH
#    NAMES MaestroLogging
#    PATHS "../util/C/logging"
#          "../util/C"
#          ${CMAKE_INSTALL_LIBDIR}
#          ${CMAKE_INSTALL_INCLUDEDIR})
#add_library(MaestroLogging SHARED IMPORTED)
#set_property(TARGET MaestroLogging PROPERTY IMPORTED_LOCATION ${MAESTRO_LOGGING_PATH})

SET(USE_CITS FALSE CACHE BOOL "Flag to indicate the use of CITS.")


# MQTT PAHO
if(USE_CITS)
    find_package(OpenSSL REQUIRED)
    find_library(paho-mqtt3c NAMES libpaho-mqtt3c.so REQUIRED)
    add_library(pahomqtt3c SHARED IMPORTED)
    set_property(TARGET pahomqtt3c PROPERTY IMPORTED_LOCATION ${paho-mqtt3c})
    add_library(cits
        src/citscontrol.c
        inc/citscontrol.h
    )
add_definitions(-DCITS_ENABLED)
endif()


# Create library
add_library(MaestroLogging
  ../util/C/logging/logging.h
  ../util/C/logging/logging.c
)

add_library(MaestroTime
  ../util/C/time/maestroTime.h
  ../util/C/time/maestroTime.c
)

add_library(MQBus
  ../util/C/MQBus/mqbus.h
  ../util/C/MQBus/mqbus.c
)

# Create library
add_library(util
  src/util.c
  inc/util.h
  inc/iso22133.h
)

# Create library
add_library(objctrl
  src/objectcontrol.c
  inc/objectcontrol.h
)


# add the executable
add_executable(TEServer
  src/main.c
  src/logger.c
  src/objectcontrol.c
  src/supervision.c
  src/systemcontrol.c
  src/timecontrol.c
  src/simulatorcontrol.c
  src/datadictionary.c
  inc/logger.h
  inc/objectcontrol.h
  inc/supervision.h
  inc/systemcontrol.h
  inc/timecontrol.h
  inc/simulatorcontrol.h
  inc/datadictionary.h
)

# add the install targets
install (TARGETS TEServer DESTINATION bin)

# add linking to message queue
target_link_libraries(util rt m)
target_link_libraries(util m)
target_link_libraries(util rt)
target_link_libraries(util MaestroLogging MaestroTime MQBus)

target_link_libraries(TEServer util)

if(USE_CITS)
    target_link_libraries(cits pahomqtt3c MaestroTime)
    target_link_libraries(TEServer cits)
endif()

target_link_libraries(TEServer MaestroLogging MaestroTime MQBus)
