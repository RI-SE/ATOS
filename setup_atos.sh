#!/bin/bash

#####################################
###### Pre-installation checks ######
#####################################

source "scripts/installation/install_functions.sh"

# Get this file location
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"

# Check if running on Ubuntu
if ! grep -q "Ubuntu" /etc/os-release; then
    echo "This script is designed for Ubuntu systems only."
    exit 1
fi

# Set ROS_DISTRO based on Ubuntu distribution
case "$(get_ubuntu_codename)" in
    "focal")
    ;;
    "jammy")
    ;;
    *)
    echo "Unsupported Ubuntu distribution. Only 20.04 (focal) and 22.04 (jammy) are supported."
    exit 1
    ;;
esac

# Add -h/--help option
if [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
    echo "Usage: ./setup_atos.sh [single option]"
    echo "This script will install all necessary dependencies, setup the ROS workspace at ~/atos_ws and install ATOS. Please open and inspect this script for further details."
    #Show options
    echo "Options:"
    echo "  -h, --help      Show this help message and exit"
    echo "  -d              Only install dependencies"              
    exit 0

# Only install dependencies
elif [ "$1" == "-d" ]; then
    INSTALL_ATOS=false
else
    INSTALL_ATOS=true
fi

#######################################
###### Install ATOS dependencies ######
#######################################
echo "Installing ATOS dependencies..."
${REPO_DIR}/scripts/installation/install_deps.sh ${REPO_DIR}

if [ $? -ne 0 ]; then
    echo "Failed to install dependencies."
    exit 1
fi
########################################
###### Start installation of ATOS ######
########################################
if [ "$INSTALL_ATOS" = false ]; then
    echo "ATOS installation skipped."
    exit 0
fi
echo "Installing ATOS..."
${REPO_DIR}/scripts/installation/install_atos.sh ${REPO_DIR}

if [ $? -ne 0 ]; then
    echo "Failed to install ATOS."
    exit 1
fi

echo "ATOS installation and setup is complete. You can now test run ATOS by running the following command:"
echo "ros2 launch atos launch_basic.py"
echo "Please see the documentation for further details: https://atos.readthedocs.io/en/latest/"
