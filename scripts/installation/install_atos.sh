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
source "${ATOS_REPO_PATH}/scripts/installation/install_functions.sh"
check_command_failed $? "Failed to source ${ATOS_REPO_PATH}/scripts/installation/install_functions.sh"

ROS_DISTRO="${ROS_DISTRO:-$(get_supported_ros_distro)}"
check_command_failed $? "Failed to determine ROS 2 distribution for this Ubuntu release."
ATOS_VENV_PATH="$(get_atos_venv_path)"

if [ -z "${ROS_DISTRO}" ]; then
    echo "Failed to determine a supported ROS 2 distribution for this Ubuntu release."
    exit 1
fi

if [ ! -f "${ATOS_VENV_PATH}/bin/activate" ]; then
    echo "ATOS Python virtual environment was not found at ${ATOS_VENV_PATH}."
    echo "Run setup_atos.sh again to recreate Python dependencies."
    exit 1
fi

################################################
############## Install ATOS ####################
################################################

# Create a workspace dir if it doesn't exist
if [ ! -d "$HOME/atos_ws/src" ]; then
    mkdir -p $HOME/atos_ws/src
fi
cd $HOME/atos_ws

# Update symlinks to atos and atos_interfaces
update_symlink "$ATOS_REPO_PATH" $HOME/atos_ws/src/atos

# Change directory into the workspace and build, check with the user before continuing
echo "Dependecy installation done and ATOS workspace created."

# First make sure the submodules are up to date
echo "Updating submodules to make sure they are up to date..."
cd $HOME/atos_ws/src/atos
if command -v git >/dev/null 2>&1 && git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    git submodule update --init --recursive
else
    echo "Skipping submodule update: source tree is not a git repository in this environment."
fi
cd -

# temporarily cd into the workspace and build with colcon
echo "Building ATOS..."
cd $HOME/atos_ws
# shellcheck disable=SC1090
source "${ATOS_VENV_PATH}/bin/activate"
source /opt/ros/$ROS_DISTRO/setup.bash
MAKEFLAGS=-j4 colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DWITH_TRUCK_OBJECT_CONTROL=ON
check_command_failed $? "Failed to build ATOS."
cd -

#####################################
###### Configure setup scripts ######
#####################################

if [ -t 0 ] && [ -z "$DEBIAN_FRONTEND" ] && [ -z "$GITHUB_ACTION" ]; then
    echo ""
    echo "Would you like to add ATOS source lines to your shell config file (.bashrc/.zshrc)?"
    echo "This will auto-activate the ATOS environment in new terminals. (y/n)"
    read -r answer
    if [ "$answer" == "${answer#[Yy]}" ]; then
        echo "Skipping shell configuration."
        exit 0
    fi
fi

atos_venv_setup_script="source $ATOS_VENV_PATH/bin/activate"
atos_python_site_packages="$(python -c 'import site; print(site.getsitepackages()[0])')"
check_command_failed $? "Failed to determine ATOS Python site-packages path."
atos_pythonpath_script="export PYTHONPATH=$atos_python_site_packages:\$PYTHONPATH"

case "$SHELL" in
    */bash)
        add_source_line_if_needed $HOME/.bashrc "bash" "${atos_venv_setup_script}"
        add_source_line_if_needed $HOME/.bashrc "bash" "${atos_pythonpath_script}"
        add_source_line_if_needed $HOME/.bashrc "bash" "source /opt/ros/$ROS_DISTRO/setup.sh"
        add_source_line_if_needed $HOME/.bashrc "bash" "source $HOME/atos_ws/install/setup.sh"
    ;;
    */zsh)
        add_source_line_if_needed $HOME/.zshrc "zsh" "${atos_venv_setup_script}"
        add_source_line_if_needed $HOME/.zshrc "zsh" "${atos_pythonpath_script}"
        add_source_line_if_needed $HOME/.zshrc "zsh" "source /opt/ros/$ROS_DISTRO/setup.zsh"
        add_source_line_if_needed $HOME/.zshrc "zsh" "source $HOME/atos_ws/install/setup.zsh"
    ;;
    *)
        echo "Unsupported shell detected! Please use either bash or zsh shells to run ATOS"
        exit 1
    ;;
esac
