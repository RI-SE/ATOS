cmake_minimum_required (VERSION 2.6)
project (ChronosObject)

# copy needed file
#configure_file(traj/0.traj traj/192.168.0.1 COPYONLY)

# add include directories
#include_directories(inc)


# add the executable
add_executable(ChronosObject 
  src/object.c
)

# add the install targets
install (TARGETS ChronosObject DESTINATION bin)

# add linking to message queue
target_link_libraries(ChronosObject m)

