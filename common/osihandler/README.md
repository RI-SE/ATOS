# Install dependencies

Build OSI and dependencies from source according to [documentation](https://opensimulationinterface.github.io/osi-documentation/index.html#_installing_osi_for_c_on_linux) for Cpp usage.

Protobuf:
```bash
sudo apt-get install libprotobuf-dev protobuf-compiler
```

OSI:
```bash
git clone https://github.com/OpenSimulationInterface/open-simulation-interface.git
cd open-simulation-interface
git checkout <VERSION>
mkdir build
cd build
cmake ..
make
sudo make install
```
## Recommended versions
OSI: == v3.4.0
Protobuf: >=3.0.0 (`protoc --version`)

OSI v3.5.0 have multiple problems due to new paths where the library is installed. Which requires multiple fixes in multiple cmakelists therefore don't use that version yet.