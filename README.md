# ATOS - AV Test Operating System
<img align="left" width="100" height="100" src="./docs/res/ATOS_icon.svg">
<img align="right" width="400" height="300" src="https://user-images.githubusercontent.com/15685739/227924215-d5ff67f8-1e03-45d0-ae20-8e60819b2ff7.png">

ATOS (AV Test Operating System), an ISO 22133-compliant and ROS2-based scenario execution engine, controls, monitors and coordinates both physical and virtual vehicles and equipment according to scenarios specified in the ASAM OpenSCENARIO® format. It is made for running in real-time and uses GPS time to ensure exact and repeatable execution between runs.
<br />
<br />
To build ATOS follow the guide below. More documentation can be found [here](https://atos.readthedocs.io/en/latest/).

<br />
<br />
<br />
<br />
<br />


# <a name="ATOS"></a> Installing ATOS
There are too ways to start using ATOS: using the docker image or building from source. The docker image is the easiest way to get started, but if you intend to make changes to ATOS, we recommend building from source.

## <a name="docker"></a> Using the docker image
To run ATOS using the docker image, first install docker on your computer. Then, run the following command from the root directory:
```bash
docker compose up
```

## <a name="Installation script"></a> Using the installation script
ATOS comes with an installation script that automates the installation process. It is intended for use on Ubuntu 20.04 or 22.04, and has been tested on a fresh install of Ubuntu 20.04 and 22.04. The script will install ROS2, ATOS dependencies, and ATOS itself. It will also create a workspace and build ATOS. The script can be run using the following command:
```bash
./install_atos.sh
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

