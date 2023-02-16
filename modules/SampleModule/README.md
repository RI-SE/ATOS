# Sample module
This is a sample module to be used as template for creating new modules.
## Features
The sample module is a ros2 node that features some basic publishers and subscribers to various topics.
It also features a TCPServer running in a separate thread. 

## Usage
In order to launch compile and launch this module (or any other module created from the template) you need to go to the outer-most CMakeLists.txt file in the root of the repository and add the following line:
```
set(WITH_MODULE_X ON CACHE BOOL "Enable ModuleX module")
```


Followed by:
```
if(WITH_MODULE_X)
        list(APPEND ENABLED_MODULES ModuleX)
endif()
```

Note: When switching ON/OFF certain modules, it might be nessesscary to remove the CMakeCache.txt file in ~/atos_ws/install/atos/.

It is also necessary to add the module to a launch file, located in the launch directory. This is done by adding the following line to the list of nodes:
```
    Node(
        package='atos',
        namespace='atos',
        executable='module_x',
        name='module_x',
    )
```

Then you can compile and launch the module by running the following commands:
```
MAKEFLAGS=-j5 colcon build --packages-up-to atos
```
(tune -j5 to an approperiate number depending on your availiable RAM memory and CPU cores)
```
ros2 launch atos atos.launch.py
```