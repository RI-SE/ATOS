#!/bin/bash

#####################################
###### Pre-installation checks ######
#####################################

# Get the script location
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"

# Function to get Ubuntu distribution codename
get_ubuntu_codename() {
    source /etc/os-release
    echo $UBUNTU_CODENAME
}

# Function that checks if command failed
check_command_failed() {
    local exitcode="$1"
    local error_message="$2"

    if [ $exitcode -ne 0 ]; then
        echo "$error_message"
        exit 1
    fi
}

# Check if running on Ubuntu
if ! grep -q "Ubuntu" /etc/os-release; then
    echo "This script is designed for Ubuntu systems only."
    exit 1
fi

# Set ROS_DISTRO based on Ubuntu distribution
case "$(get_ubuntu_codename)" in
    "focal")
        ROS_DISTRO=foxy
    ;;
    "jammy")
        ROS_DISTRO=humble
    ;;
    *)
        echo "Unsupported Ubuntu distribution. Only 20.04 (focal) and 22.04 (jammy) are supported."
        exit 1
    ;;
esac

# Prompt user to acknowledge installation
echo "This script will try to install all necessary dependencies for ATOS. Please open and inspect this script for further details."
echo "ROS distribution: $ROS_DISTRO"
read -p "Do you wish to continue? [Y/n] " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    exit 1
fi

# Check internet connection
if ! ping -q -c 1 -W 1 google.com >/dev/null; then
    echo "No internet connection detected. Please connect to the internet and try again."
    exit 1
fi

#######################################
###### Install ATOS dependencies ######
#######################################

# Update and install required dependencies
echo "Updating apt and installing dependencies..."
sudo apt update && sudo apt install -y \
    libsystemd-dev \
    libprotobuf-dev \
    protobuf-compiler \
    libeigen3-dev \
    nlohmann-json3-dev \
    python3-pip \
    python3-colcon-common-extensions
# Check if apt failed to install dependencies
check_command_failed $? "Failed to install dependencies."

################################################
###### Install Control Panel dependencies ######
################################################

echo "Installing nvm and Node.js..."
if [ "$ROS_DISTRO" == "humble" ]; then 
    NODE_VERSION=16.20.0
elif [ "$ROS_DISTRO" == "foxy" ]; then
    NODE_VERSION=12
fi
NVM_DIR="$HOME/.nvm"

if ! command -v nvm &> /dev/null; then
    echo "nvm not found, installing..."
    curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.0/install.sh | bash
    . "$NVM_DIR/nvm.sh"
    check_command_failed $? "Failed to install nvm."
else 
    echo "nvm already installed, skipping installation..."
fi

echo "Installing Node.js v${NODE_VERSION}..."
nvm install ${NODE_VERSION}
nvm use v${NODE_VERSION}
nvm alias default v${NODE_VERSION}
echo 'export PATH="$HOME/.nvm/versions/node/v${NODE_VERSION}/bin/:$PATH"' >> $HOME/.bashrc
source $HOME/.bashrc

# Install pyOpenSSL
pip install pyOpenSSL

#######################################
###### Install ROS2 dependencies ######
#######################################

# Check if the ROS2 repository is already added
if ! (dpkg -l | grep -q "ros-$ROS_DISTRO-desktop"); then
    echo "Adding the ROS2 $ROS_DISTRO apt repository..."

    # Install ROS2 prerequisites
    sudo apt update && sudo apt install curl gnupg2 lsb-release

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
    ros-${ROS_DISTRO}-rosbridge-suite
check_command_failed $? "Failed to install ROS2 packages."

if [ "$ROS_DISTRO" == "humble" ]; then
    sudo apt install -y libpaho-mqtt-dev
elif [ "$ROS_DISTRO" == "foxy" ]; then
    sudo apt install -y ros-foxy-paho-mqtt-c
fi

###########################################
###### Install some deps from source ######
###########################################

# Set custom path for source installation, first check if it exists
SOURCE_PATH=~/temp/atos_install
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
    git clone https://github.com/OpenSimulationInterface/open-simulation-interface.git -b v3.4.0 $SOURCE_PATH/open-simulation-interface
    cd $SOURCE_PATH/open-simulation-interface
    mkdir -p build && cd build
    cmake .. && make
    check_command_failed $? "Failed to  build OpenSimulationInterface."
    sudo make install
    check_command_failed $? "Failed to install OpenSimulationInterface."
    sudo sh -c "echo '/usr/local/lib/osi3' > /etc/ld.so.conf.d/osi3.conf"
    sudo ldconfig
    check_command_failed $? "Failed ldconfig after installing OpenSimulationInterface."
fi

# Install ad-xolib but first check if the library is already installed
if [ -d "/usr/local/include/ad-xolib" ]; then
    echo "ad-xolib already installed, skipping installation..."
else
    echo "Installing ad-xolib..."
    git clone https://github.com/javedulu/ad-xolib.git $SOURCE_PATH/ad-xolib
    cd $SOURCE_PATH/ad-xolib
    git submodule update --init --recursive
    mkdir -p build && cd build
    cmake .. -DBUILD_EMBED_TARGETS=OFF && make
    check_command_failed $? "Failed to build ad-xolib."
    sudo make install
    check_command_failed $? "Failed to install ad-xolib."
    sudo ldconfig
    check_command_failed $? "Failed ldconfig after installing ad-xolib."
fi

# Install esmini but first check if the library is already installed
if [ -d "/usr/local/include/esmini" ]; then
    echo "esmini already installed, skipping installation..."
else
    echo "Installing esmini..."
    git clone https://github.com/esmini/esmini.git $SOURCE_PATH/esmini
    cd $SOURCE_PATH/esmini
    mkdir -p build && cd build
    cmake .. && make
    check_command_failed $? "Failed to build esmini."
    sudo make install
    check_command_failed $? "Failed to install esmini."
    sudo cp ../bin/libesminiLib.so /usr/local/lib
    sudo cp ../bin/libesminiRMLib.so /usr/local/lib
    sudo mkdir -p /usr/local/include/esmini/
    sudo cp ../EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp /usr/local/include/esmini/
    sudo cp ../EnvironmentSimulator/Libraries/esminiRMLib/esminiRMLib.hpp /usr/local/include/esmini
    sudo ldconfig
    check_command_failed $? "Failed ldconfig after installing esmini." 
fi

# Remove custom path for source installation
echo "Removing custom path for source installation..."
sudo rm -rf $SOURCE_PATH
check_command_failed $? "Failed to remove ${$SOURCE_PATH} path for source installation."

########################################
###### Start installation of ATOS ######
########################################

echo "Installing ATOS..."
# Create a workspace dir if it doesn't exist
if [ ! -d "~/atos_ws/src" ]; then
    mkdir -p ~/atos_ws/src
fi
cd ~/atos_ws

# Set ATOS_PATH to the script location
ATOS_PATH="$SCRIPT_DIR"

# Set ATOS_INTERFACES_PATH using ATOS_PATH
ATOS_INTERFACES_PATH="$ATOS_PATH/atos_interfaces"

# Function to update symlink if it doesn't point to the correct location
update_symlink() {
    local target="$1"
    local link_name="$2"

    if [ -L "$link_name" ]; then
        current_target="$(readlink "$link_name")"
        if [ "$current_target" != "$target" ]; then
            rm "$link_name"
            ln -s "$target" "$link_name"
            echo "Updated symlink $link_name to $target because it pointed to $current_target previously."
        fi
    else
        ln -s "$target" "$link_name"
        echo "Created symlink $link_name to $target"
    fi
}
# Update symlinks to atos and atos_interfaces
update_symlink "$ATOS_PATH" ~/atos_ws/src/atos
update_symlink "$ATOS_INTERFACES_PATH" ~/atos_ws/src/atos_interfaces


# Change directory into the workspace and build, check with the user before continuing
echo "Dependecy installation done and ATOS workspace created."
read -p "Do you wish to continue by installing ATOS with colcon? [Y/n] " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    exit 1
fi

# First make sure the submodules are up to date
echo "Updating submodules to make sure they are up to date..."
cd ~/atos_ws/src/atos
git submodule update --init --recursive
cd -

# temporarily cd into the workspace and build with colcon
echo "Building ATOS..."
cd ~/atos_ws
MAKEFLAGS=-j8 && colcon build --symlink-install
check_command_failed $? "Failed to build ATOS."
cd -

#####################################
###### Configure setup scripts ######
#####################################

# Function to add the setup.bash source line if it doesn't exist
add_source_line_if_needed() {
    local file="$1"
    local shell_type="$2"
    local source_line="$3$shell_type"

    if ! grep -qF "$source_line" "$file"; then
        echo "Adding the following line to your shells config file: $file"
        echo "$source_line"
        echo "$source_line" >> "$file"
    fi
}

atos_setup_script="source ~/atos_ws/install/setup."
ros2_setup_script="source /opt/ros/$ROS_DISTRO/setup."

case "$SHELL" in
    */bash)
        add_source_line_if_needed ~/.bashrc "bash" "${atos_setup_script}"
        add_source_line_if_needed ~/.bashrc "bash" "${ros2_setup_script}"
        source ~/.bashrc
    ;;
    */zsh)
        add_source_line_if_needed ~/.zshrc "zsh" "${atos_setup_script}"
        add_source_line_if_needed ~/.zshrc "zsh" "${ros2_setup_script}"
        source ~/.zshrc
    ;;
    *)
        echo "Unsupported shell detected! Please use either bash or zsh shells to run ATOS"
        exit 1
    ;;
esac

echo "ATOS installation and setup is complete. You can now test run ATOS by running the following command:"
echo "ros2 launch atos launch_basic.py"
