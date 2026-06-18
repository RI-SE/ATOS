#!/bin/bash
set -e
# Check if called without arguments
if [ $# -eq 0 ]; then
    echo "Don't call this file directly, use setup_atos.sh instead."
    echo "Usage: ./install_deps.sh <path to ATOS git repo> [flags]"
    echo "Flags:"
    echo "  -r             Reinstall all dependencies, including esmini and OpenSimulationInterface"
    exit 0
fi
# Take the first argument as the PATH to the ATOS git repo
ATOS_REPO_PATH="$1"

# Check if the FULL_INSTALL flag is set
REINSTALL=false
if [ "$2" == "-r" ]; then
    REINSTALL=true
fi

source "${ATOS_REPO_PATH}/scripts/installation/install_functions.sh"

if [ -z "${ROS_DISTRO:-}" ]; then
    echo "ROS_DISTRO is not set. Source /opt/ros/<distro>/setup.bash before running this script."
    exit 1
fi

ATOS_VENV_PATH="$(get_atos_venv_path)"
apt_update_retry() {
    local attempts=5
    local delay=5
    local i=1
    while [ "$i" -le "$attempts" ]; do
        if sudo apt-get update; then
            return 0
        fi
        echo "apt update failed (attempt ${i}/${attempts}); cleaning apt cache and retrying..."
        sudo apt-get clean
        sudo rm -rf /var/lib/apt/lists/*
        sleep "$delay"
        i=$((i + 1))
    done
    return 1
}

apt_install_retry() {
    local attempts=3
    local delay=5
    local i=1
    while [ "$i" -le "$attempts" ]; do
        if sudo apt-get install -y "$@"; then
            return 0
        fi
        echo "apt install failed (attempt ${i}/${attempts}); retrying..."
        sleep "$delay"
        i=$((i + 1))
    done
    return 1
}

# Update and install required dependencies specified in dependencies.txt and requirements.txt file
apt_deps=$(cat ${ATOS_REPO_PATH}/scripts/installation/dependencies.txt | tr '\n' ' ')
echo "Installing dependencies... $apt_deps"
apt_update_retry
apt_install_retry ${apt_deps}
apt_install_retry python3-pip
apt_install_retry python3-venv

if [ -d "${ATOS_VENV_PATH}" ] && [ "$REINSTALL" = true ]; then
    echo "Removing existing ATOS Python virtual environment at ${ATOS_VENV_PATH}..."
    rm -rf "${ATOS_VENV_PATH}"
fi

if [ ! -f "${ATOS_VENV_PATH}/bin/activate" ]; then
    echo "Creating ATOS Python virtual environment at ${ATOS_VENV_PATH}..."
    mkdir -p "$(dirname "${ATOS_VENV_PATH}")"
    # ROS Python tooling depends on distro packages such as `empy` being importable
    # while the ATOS venv is active during colcon builds.
    python3 -m venv --system-site-packages "${ATOS_VENV_PATH}"
fi

"${ATOS_VENV_PATH}/bin/python" -m pip install --upgrade pip
"${ATOS_VENV_PATH}/bin/python" -m pip install -r ${ATOS_REPO_PATH}/scripts/installation/requirements.txt

# Check if apt failed to install dependencies
check_command_failed $? "Failed to install dependencies."

#######################################
###### Install ROS2 dependencies ######
#######################################
# Check if the ROS2 repository is already added
if ! (apt list | grep -q "ros-$ROS_DISTRO-desktop"); then
    echo "Adding the ROS2 $ROS_DISTRO apt repository..."

    # Install ROS2 prerequisites
    apt_update_retry
    apt_install_retry lsb-release ros-dev-tools

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
apt_update_retry
apt_install_retry \
    ros-${ROS_DISTRO}-desktop \
    python3-rosdep \
    ros-${ROS_DISTRO}-launch-pytest
check_command_failed $? "Failed to install ROS2 packages."

sudo rosdep init || true && \
    rosdep update || true && \
    rosdep install --from-paths ${ATOS_REPO_PATH} --ignore-src --rosdistro $ROS_DISTRO -y
check_command_failed $? "Failed to install ROS2 dependencies."

###############################################
######## Install ATOS GUI dependencies ########
###############################################

"${ATOS_VENV_PATH}/bin/python" -m pip install -r ${ATOS_REPO_PATH}/atos_gui/requirements.txt

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
if [ -d "/usr/local/lib/osi3" ] && [ "$REINSTALL" = false ]; then
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

# Install esmini but first check if the library is already installed, override check if FULL_INSTALL is set
if [ -d "/usr/local/include/esmini" ] && [ "$REINSTALL" = false ]; then
    echo "esmini already installed, skipping installation..."
else
    echo "Downloading esmini binaries..."
    wget https://github.com/esmini/esmini/releases/download/v2.37.17/esmini-bin_Linux.zip -O $SOURCE_PATH/esmini-bin_Linux.zip
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
