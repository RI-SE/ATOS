## JournalControl module
This module is an example for how to build a Maestro-external module and connect it via message bus. It is also written in C++ so can serve as an example for how to import Maestro C code and connect to message bus using another code language.

### Build process
1) Ensure your util repo is up to date
2) Navigate to this README.md file
3) Create the build directory: ```mkdir build```
4) Enter the build directory: ```cd build```
5) Generate necessary cmake files: ```cmake ..```
6) Build the module: ```make```

### Run the module
1) Ensure you have built the module
2) Navigate to the build directory
3) Run the module: ```./dummy```
4) Run Maestro

Note: steps 3 and 4 can be replaced with running the runServer.sh script in the top directory of this repository
