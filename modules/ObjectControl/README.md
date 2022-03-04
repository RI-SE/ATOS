## Relative kinematics module
This module executes control of objects according to a VUT-relative scenario such as is the case for a VIL rig or similar cases when the object of interest is physically standing still. 

### Dependencies
#### Eigen
Eigen is a library for performing matrix calculations.
If you are running a debian derivative, simply run the following command as root:
```bash
apt install libeigen3-dev
```
If you are running Windows (not recommended), read the [Eigen getting started guide](https://eigen.tuxfamily.org/dox/GettingStarted.html).

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
3) Ensure Core is running
3) Run the module: ```./RelativeKinematics```
