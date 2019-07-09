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
include(GNUInstallDirs)


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


# add the executable
add_executable(Maestro
  src/main.c
  src/logger.c
  src/objectcontrol.c
  src/supervision.c
  src/systemcontrol.c
  src/timecontrol.c
  src/util.c
  inc/logger.h
  inc/objectcontrol.h
  inc/supervision.h
  inc/systemcontrol.h
  inc/timecontrol.h
  inc/util.h
)

# add the install targets
install (TARGETS Maestro DESTINATION bin)

target_link_libraries(MQBus rt) # Make sure the MQBus has linking to rt for message queue support

# Link the local libraries together with Maestro and the math library "m"
target_link_libraries(Maestro MQBus m MaestroLogging MaestroTime) 

if(USE_CITS) # If C-ITS is used, link with 3d party software pahomqtt3c
    target_link_libraries(cits pahomqtt3c)
    target_link_libraries(Maestro cits)
endif()



