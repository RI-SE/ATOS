#!/bin/bash
set -e
# Check if called without arguments
if [ $# -eq 0 ]; then
    echo "Don't call this file directly, use setup_atos.sh instead."
    echo "Usage: ./install_deps.sh <path to ATOS git repo>"           
    exit 0
fi
# Take the first argument as the PATH to the ATOS git repo
ATOS_REPO_PATH="$1"
#fail unless var is set
source "${ATOS_REPO_PATH}/scripts/installation/install_functions.sh"

# Update and install required dependencies specified in dependencies.txt and requirements.txt file
apt_deps=$(cat ${ATOS_REPO_PATH}/scripts/installation/dependencies.txt | tr '\n' ' ')
echo "Installing dependencies... $apt_deps"
sudo apt update && sudo apt install -y ${apt_deps}
pip install -r ${ATOS_REPO_PATH}/scripts/installation/requirements.txt

# Install pre-commit hooks for the ATOS repo
echo "Installing pre-commit hooks for the ATOS repo..."
cd $ATOS_REPO_PATH
# Check if we are in a git repo
if [ -d ".git" ]; then
    pre-commit install
fi

# Check if apt failed to install dependencies
check_command_failed $? "Failed to install dependencies."

#######################################
###### Install ROS2 dependencies ######
#######################################
ROS_DISTRO=humble

# Check if the ROS2 repository is already added
if ! (apt list | grep -q "ros-$ROS_DISTRO-desktop"); then
    echo "Adding the ROS2 $ROS_DISTRO apt repository..."

    # Install ROS2 prerequisites
    sudo apt update && sudo apt install -y lsb-release ros-dev-tools

    # Authorize the ROS2 gpg key with apt
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

    # Add the ROS2 repo to sources list
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
else
    echo "ROS2 $ROS_DISTRO" repository already added, skipping addition...
fi

# Install ROS2 packages
echo "Installing ROS2 packages..."
sudo apt install -y \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-geographic-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-std-srvs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-geographic-msgs \
    ros-${ROS_DISTRO}-foxglove-msgs \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-rosbridge-suite \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-launch-pytest
check_command_failed $? "Failed to install ROS2 packages."

###############################################
######## Install ATOS GUI dependencies ########
###############################################

pip install -r ${ATOS_REPO_PATH}/atos_gui/requirements.txt 

# Install pyOpenSSL
pip install pyOpenSSL

###########################################
###### Install some deps from source ######
###########################################

# Set custom path for source installation, first check if it exists
SOURCE_PATH=$HOME/temp/atos_install
if [ -d "$SOURCE_PATH" ]; then
    echo "Removing preexisting path ${SOURCE_PATH} for source installation..."
    sudo rm -rf $SOURCE_PATH
fi

echo "Creating custom path for source installation at $SOURCE_PATH"
mkdir -p $SOURCE_PATH

# Install OpenSimulationInterface but first check if the library is already installed
if [ -d "/usr/local/lib/osi3" ]; then
    echo "OpenSimulationInterface already installed, skipping installation..."
else
    echo "Installing OpenSimulationInterface..."
    git clone --depth 1 --branch v3.4.0 https://github.com/OpenSimulationInterface/open-simulation-interface.git $SOURCE_PATH/open-simulation-interface
    cd $SOURCE_PATH/open-simulation-interface
    mkdir -p build && cd build
    cmake .. && make -j8
    check_command_failed $? "Failed to  build OpenSimulationInterface."
    sudo make install
    check_command_failed $? "Failed to install OpenSimulationInterface."
    sudo sh -c "echo '/usr/local/lib/osi3' > /etc/ld.so.conf.d/osi3.conf"
    sudo ldconfig
    check_command_failed $? "Failed ldconfig after installing OpenSimulationInterface."
fi

# Install esmini but first check if the library is already installed
if [ -d "/usr/local/include/esmini" ]; then
    echo "esmini already installed, skipping installation..."
else
    echo "Downloading esmini binaries..."
    wget https://github.com/esmini/esmini/releases/download/v2.32.0/esmini-bin_Linux.zip -O $SOURCE_PATH/esmini-bin_Linux.zip
    check_command_failed $? "Failed to get esmini."
    unzip $SOURCE_PATH/esmini-bin_Linux.zip -d $SOURCE_PATH/esmini-bin_Linux
    cd $SOURCE_PATH/esmini-bin_Linux
    sudo cp esmini/bin/libesminiLib.so /usr/local/lib
    sudo cp esmini/bin/libesminiRMLib.so /usr/local/lib
    sudo mkdir -p /usr/local/include/esmini/
    sudo cp esmini/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp /usr/local/include/esmini/
    sudo cp esmini/EnvironmentSimulator/Libraries/esminiRMLib/esminiRMLib.hpp /usr/local/include/esmini
    sudo ldconfig
    check_command_failed $? "Failed ldconfig after installing esmini." 
fi

# Remove custom path for source installation
echo "Removing custom path for source installation..."
sudo rm -rf $SOURCE_PATH
check_command_failed $? "Failed to remove ${SOURCE_PATH} path for source installation."