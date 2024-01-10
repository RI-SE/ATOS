#!/bin/bash


# Check if called without arguments
if [ $# -eq 0 ]; then
    echo "Don't call this file directly, use setup_atos.sh instead."
    echo "Usage: ./install_deps.sh <path to ATOS git repo>"           
    exit 0
fi

# Take the first argument as the PATH to the ATOS git repo
ATOS_REPO_PATH="$1"
source "${ATOS_REPO_PATH}/scripts/installation/install_functions.sh"
check_command_failed $? "Failed to source ${ATOS_REPO_PATH}/scripts/installation/install_functions.sh"

################################################
###### Install Control Panel dependencies ######
################################################

echo "Installing nvm and Node.js..."
NODE_VERSION=16.20.0    # Also specified in the Dockerfile
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
npm --version

# Install pyOpenSSL
pip install pyOpenSSL

################################################
############## Install ATOS ####################
################################################

# Create a workspace dir if it doesn't exist
if [ ! -d "$HOME/atos_ws/src" ]; then
    mkdir -p $HOME/atos_ws/src
fi
cd $HOME/atos_ws

# Set ATOS_INTERFACES_PATH using ATOS_PATH
ATOS_INTERFACES_PATH="$ATOS_REPO_PATH/atos_interfaces"
ATOS_GUI_PATH="$ATOS_REPO_PATH/gui/ros_gui"

# Update symlinks to atos and atos_interfaces
update_symlink "$ATOS_REPO_PATH" $HOME/atos_ws/src/atos
update_symlink "$ATOS_INTERFACES_PATH" $HOME/atos_ws/src/atos_interfaces
update_symlink "$ATOS_GUI_PATH" $HOME/atos_ws/src/atos_gui

# Change directory into the workspace and build, check with the user before continuing
echo "Dependecy installation done and ATOS workspace created."

# First make sure the submodules are up to date
echo "Updating submodules to make sure they are up to date..."
cd $HOME/atos_ws/src/atos
git submodule update --init --recursive
cd -

# temporarily cd into the workspace and build with colcon
echo "Building ATOS..."
cd $HOME/atos_ws
source /opt/ros/$ROS_DISTRO/setup.bash
MAKEFLAGS=-j4 colcon build --symlink-install
check_command_failed $? "Failed to build ATOS."
cd -

#####################################
###### Configure setup scripts ######
#####################################

atos_setup_script="source $HOME/atos_ws/install/setup."
ros2_setup_script="source /opt/ros/$ROS_DISTRO/setup."

case "$SHELL" in
    */bash)
        add_source_line_if_needed $HOME/.bashrc "bash" "${ros2_setup_script}"
        add_source_line_if_needed $HOME/.bashrc "bash" "${atos_setup_script}"
    ;;
    */zsh)
        add_source_line_if_needed $HOME/.zshrc "zsh" "${ros2_setup_script}"
        add_source_line_if_needed $HOME/.zshrc "zsh" "${atos_setup_script}"
    ;;
    *)
        echo "Unsupported shell detected! Please use either bash or zsh shells to run ATOS"
        exit 1
    ;;
esac