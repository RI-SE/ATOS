cmake_minimum_required (VERSION 2.6)
project (TEServer)

# copy needed file
configure_file(traj/0.traj traj/192.168.0.1 COPYONLY)
configure_file(traj/1.traj traj/192.168.0.2 COPYONLY)
configure_file(conf/test.conf conf/test.conf COPYONLY)

# add include directories
include_directories(inc)

# Create library
add_library(util 
  src/util.c 
  inc/util.h
)

# add the executable
add_executable(DefaultVisualizationAdapter 
  src/visualization.c
)

# add the executable
add_executable(TEServer 
  src/main.c
  src/logger.c
  src/objectcontrol.c 
  src/supervision.c
  src/systemcontrol.c
  src/usercontrol.c
  inc/logger.h
  inc/objectcontrol.h
  inc/supervision.h
  inc/systemcontrol.h
  inc/usercontrol.h
)

# add the install targets
install (TARGETS DefaultVisualizationAdapter DESTINATION bin)
install (TARGETS TEServer DESTINATION bin)

# add linking to message queue
target_link_libraries(util rt m)
target_link_libraries(util m)
target_link_libraries(util rt)
target_link_libraries(DefaultVisualizationAdapter util)
target_link_libraries(TEServer util)


