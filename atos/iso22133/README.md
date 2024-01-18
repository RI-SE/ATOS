# ISO 22133
Encoders and decoders for the ISO 22133 communication protocol.

## Build
Navigate to the project directory and create a build directory. Then run cmake and finally build the project:
```
mkdir build && cd build
cmake ..
make
```

### Build for ARM (Linux only)
Install the compilers
```
sudo apt install gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf
```

Change to the build directory, then compile and build with the toolchain
```
cmake .. -DCMAKE_TOOLCHAIN_FILE=../Toolchain_arm.cmake
make
```

### Run tests
After building the project, run
```
make test
```

## SWIG Python wrapper build
To use the encoders and decoders in other languages than C/C++, use the below procedure

### Windows 
1. Install swig by downloading swig from their webpage, the download should include a pre-compiled build for windows.
2. Add the path to the pre-compiled swig.exe file that you downloaded to path variabel in system enviroment. 
3. Download a C/C++ compiler such as Visual Studio(_MSC_VER) or MinGW (preferable 64bit,__GNUC__,  __MINGW32__ ).  
4. Add the path to the downloaded pre-compiled compiler to system enviroment in windows path variabel.
5. Test that you can access swig run in cmd.
    ```
    swig -version
    ```
    This will give you version number of your swig etc.
6. Test you compiler by running the command to run the compiler, this will be different depending on your compiler.
    On MinGW:
    ```
    g++ --version
    ```
    This will give output for the MinGW compiler.
7. Start your python virtual environment or use your base environment at your own risk.
8. Start terminal and move to iso22133 folder to the setup.py file folder.
9. Run the following command(It will build the python extension): 
    ```
    python setup.py build_ext
    ```
10. Now install the extension to your python virtual enviroment or add the generated .pyd (i think) file to your path in system enviroment.
11. For installation run in terminal:
    ```
    python setup.py install
    ```
    
### Linux
1. First install SWIG run:
    ```
    sudo apt install swig
    ```
### Build with SWIG for Python
In the build directory, remove old content, if needed, then:
```
cmake -DWITH_SWIG=TRUE -DSWIG_WITH_PYTHON=TRUE ..
make
```

If you want it installed on the local machine:
```
sudo make install
```

#### Java 
1. Make sure you have Java JDK is installed. 
2. Make sure the iso22133.i file includes the line %javaconst(1)
3. Run the following command which will create all the necessary java files.  
    ```
    swig -java iso22133.i
    ```
4. Compile using. ((Pick jdk applicaple to your machine))
    ```
    gcc -fPIC -Wall -c positioning.c iso22133.c iso22133_wrap.c -I/usr/lib/jvm/jdk-15.0.2/include/ -I/usr/lib/jvm/jdk-15.0.2/include/linux/ 
    ```
5. Create .so file
    ```
    gcc -shared positioning.o iso22133_wrap.o iso22133.o -o libiso22133.so
    ```
7. Copy your .so file to your other JNI libs or add the current folder to the path using:
    ```
    export LD_LIBRARY_PATH=. #ksh

    ```
9. You should now be able to use this to create and compile a simple java test program such as this:

    ```
    public class isotest {
        static {
                    System.loadLibrary("iso22133");
        }

        public static void main(String argv[]) {
            timeval heabTime = new timeval();
            System.out.println(ControlCenterStatusType.CONTROL_CENTER_STATUS_ABORT);

            iso22133 iso = new iso22133();
            iso.encodeHEABMessage(heabTime, ControlCenterStatusType.CONTROL_CENTER_STATUS_RUNNING, "datatadatadata", 22, '1');
            }
    }
      ```
