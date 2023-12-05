# ATOS - AV Test Operating System
<img align="left" width="100" height="100" src="./docs/res/ATOS_icon.svg">
<img align="right" width="400" height="300" src="https://user-images.githubusercontent.com/15685739/227924215-d5ff67f8-1e03-45d0-ae20-8e60819b2ff7.png">

ATOS stands for AV Test Operating System, and is an ISO 22133-compliant and ROS2-based scenario execution engine, controls, monitors and coordinates both physical and virtual vehicles and equipment according to scenarios specified in the ASAM OpenSCENARIO® format. It is made for running in real-time and uses GPS time to ensure exact and repeatable execution between runs.

<br />
<br />
To build ATOS follow the guide below. More documentation can be found [here](https://atos.readthedocs.io/en/latest/).

<br />
<br />
<br />


# <a name="ATOS"></a> Installing ATOS
There are two ways to start using ATOS: using the docker image or building from source. The docker image is the easiest way to get started, but if you intend to make changes to ATOS, we recommend building from source.

## <a name="docker"></a> Using the docker image
To run ATOS using the docker image, first install docker on your computer. Then, run the following command from the root repo directory:
```bash
docker compose up
```

You can also start ATOS without the compose file with docker run:

If you run Docker Engine you can start ATOS using the computers own network stack with
```bash
docker run --network="host"  --ipc=host --privileged -it -v ~/.astazero/ATOS/:/root/.astazero/ATOS/ astazero/atos_docker_env:latest bash -c "source /root/atos_ws/install/setup.sh ; ros2 launch atos launch_basic.py"
```
If you run Docker Desktop you will need to specify the ports to expose to the host computer.
```bash
docker run --ipc=host --privileged -it -v ~/.astazero/ATOS/:/root/.astazero/ATOS/ -p 80:80 -p 8080:8080 -p 8081:8081 -p 8082:8082 -p 3000:3000 -p 3443:3443 -p 55555:55555 -p 443:443 -p 9090:9090 astazero/atos_docker_env:latest bash -c "source /root/atos_ws/install/setup.sh ; ros2 launch atos launch_basic.py"
```

You might wish to mount the config directory at ~/.astazero/ATOS/ to a different location on your host computer. This can be done by changing the path after the -v flag in the above command. You might also wish to inspect the image with instead of running the launch_basic.py script. This can be done by removing the last "bash -c ..." part of the command.


## <a name="Installation script"></a> Using the installation script
ATOS comes with an installation script that automates the installation process. It is intended for use on Ubuntu 22.04. The script will install ROS2 Humble, ATOS dependencies, and ATOS itself. It will also create a workspace (~/atos_ws) and build ATOS. The script can be executed using the following command:
```bash
./setup_atos.sh
```

## <a name="Native build"></a> Building from source manually
You can find instructions on how to manually install ATOS and its dependencies from source [here](https://atos.readthedocs.io/en/latest/Installation/installation/).

# <a name="usage"></a> Using ATOS with a Graphical User Interface (GUI)
Please click [here](https://atos.readthedocs.io/en/latest/Usage/GUI/foxglove/) for instructions on how to use ATOS with a GUI.

# Funded by
This project has partly been funded by the below organisations. The herein expressed views of the contributors do not necessarily reflect the views of the organisations.


<br>
<br>
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://user-images.githubusercontent.com/15685739/229127771-1d7e9c89-fc0d-4271-a7da-d805f2e6b884.svg">
  <source media="(prefers-color-scheme: light)" srcset="https://user-images.githubusercontent.com/15685739/229127758-612ec1a7-89cf-4d51-86bc-cb6ab47e422f.svg">
  <img alt="AstaZero logo" src="https://user-images.githubusercontent.com/15685739/229127758-612ec1a7-89cf-4d51-86bc-cb6ab47e422f.svg">
</picture>

<br>
<br>
<br>
<br>

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://user-images.githubusercontent.com/15685739/229121585-34dc9018-e142-4841-bc19-2485cdf03eac.png">
  <source media="(prefers-color-scheme: light)" srcset="https://user-images.githubusercontent.com/15685739/229119880-8c0a30eb-f805-4da4-a6d7-544ed7dbea87.png">
  <img alt="Vinnova logo" src="https://user-images.githubusercontent.com/15685739/229119880-8c0a30eb-f805-4da4-a6d7-544ed7dbea87.png">
</picture>
<br>
<br>

