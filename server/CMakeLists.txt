cmake_minimum_required (VERSION 3.1)
project (Maestro)

# Set C and C++ dialects
SET(CMAKE_C_STANDARD 11)
SET(CMAKE_C_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

# Standard compiler flags
SET(GCC_COMPILE_FLAGS_WARNINGS "-Wall") # Use -Wall to make all warnings appear, and -Werror to make all of them into errors. To select individual warnings use e.g. -Werror=pointer-sign
SET(GCC_COMPILE_FLAGS_COVERAGE "-fprofile-arcs -ftest-coverage")
SET(GCC_COMPILE_FLAGS_OPTIMISATION "-O0") # Use -O2 for final build

# Set which compiler flags are used for the build
SET(GCC_COMPILE_FLAGS_ALL "${GCC_COMPILE_FLAGS_WARNINGS}")

# copy needed file
#configure_file(traj/0.traj traj/192.168.0.1 COPYONLY)
#configure_file(traj/1.traj traj/192.168.0.2 COPYONLY)
#configure_file(conf/test.conf conf/test.conf COPYONLY)

# add include directories
include_directories(inc)
include(GNUInstallDirs)
if (NOT TARGET MaestroLogging)
	include_directories(../util/C/logging)
	add_library(MaestroLogging
  		../util/C/logging/logging.h
  		../util/C/logging/logging.c
	)
endif()

if (NOT TARGET MaestroTime)
	include_directories(../util/C/time)
	add_library(MaestroTime
  		../util/C/time/maestroTime.h
  		../util/C/time/maestroTime.c
	)
endif()

if (NOT TARGET MQBus)
	include_directories(../util/C/MQBus)
	add_library(MQBus
		../util/C/MQBus/mqbus.c
		../util/C/MQBus/mqbus.h
	)
endif()

if (NOT TARGET util)
	add_library(util
  		inc/util.h
  		src/util.c
  		inc/iso22133.h
  		src/iso22133.c
  		inc/positioning.h
  		src/positioning.c
	)
endif()

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
    include_directories(../util/ASN1/generatedfiles)
    find_package(OpenSSL REQUIRED)
    find_library(paho-mqtt3c NAMES libpaho-mqtt3c.so REQUIRED)
    add_library(pahomqtt3c SHARED IMPORTED)
    set_property(TARGET pahomqtt3c PROPERTY IMPORTED_LOCATION ${paho-mqtt3c})
    add_library(cits
        src/citscontrol.c
        inc/citscontrol.h
    )
    file(GLOB ASN1_HEADERS "../util/ASN1/generatedfiles/*.h")
    file(GLOB ASN1_SOURCES "../util/ASN1/generatedfiles/*.c")
    add_library(ASN1_ITS
        ${ASN1_HEADERS}
        ${ASN1_SOURCES}
    )
add_definitions(-DCITS_ENABLED)
endif()


# add the executable
add_executable(TEServer
  src/main.c
  src/logger.c
  src/objectcontrol.c
  src/systemcontrol.c
  src/timecontrol.c
  src/datadictionary.c
  inc/logger.h
  inc/objectcontrol.h
  inc/systemcontrol.h
  inc/timecontrol.h
  inc/datadictionary.h
)

# add compiler flags for all targets
set_target_properties(TEServer PROPERTIES COMPILE_FLAGS ${GCC_COMPILE_FLAGS_ALL})

# add the install targets
install (TARGETS TEServer DESTINATION bin)

# add linking to message queue
target_link_libraries(util rt m)
target_link_libraries(util m)
target_link_libraries(util rt)
target_link_libraries(util MaestroLogging MaestroTime MQBus)

target_link_libraries(TEServer util)

if(USE_CITS)
    target_link_libraries(cits pahomqtt3c MaestroTime ASN1_ITS)
    target_link_libraries(TEServer cits)
endif()

target_link_libraries(TEServer MaestroLogging MaestroTime MQBus)
