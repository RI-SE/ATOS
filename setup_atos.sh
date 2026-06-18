#!/bin/bash

#####################################
###### Pre-installation checks ######
#####################################

source "scripts/installation/install_functions.sh"

# Get this file location
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"

# Check if running on a supported Ubuntu variant
if is_ubuntu_core; then
    echo "Ubuntu Core is not supported for native ATOS installation."
    echo "Use Docker instead, or run this script on Ubuntu 20.04, 22.04, or 24.04 with apt available."
    exit 1
fi

if ! is_standard_ubuntu; then
    echo "This script is designed for standard Ubuntu systems only."
    exit 1
fi

UBUNTU_CODENAME="$(get_ubuntu_codename)"
ROS_DISTRO="$(get_supported_ros_distro)"

if [ -z "${UBUNTU_CODENAME}" ] || [ -z "${ROS_DISTRO}" ]; then
    echo "Unsupported Ubuntu distribution. Supported releases are 20.04 (focal), 22.04 (jammy), and 24.04 (noble)."
    exit 1
fi

if ! command -v apt-get >/dev/null 2>&1; then
    echo "apt-get is required for native ATOS installation but was not found on this system."
    exit 1
fi

export ROS_DISTRO

# Add -h/--help option
if [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
    echo "Usage: ./setup_atos.sh [single option]"
    echo "This script will install all necessary dependencies, setup the ROS workspace at ~/atos_ws and install ATOS."
    echo "Supported native targets: Ubuntu 20.04 with ROS 2 Foxy, Ubuntu 22.04 with ROS 2 Humble, and Ubuntu 24.04 with ROS 2 Jazzy."
    echo "Options:"
    echo "  -h, --help      Show this help message and exit"
    echo "  -r              Reinstall dependencies"              
    exit 0
fi

# Only install dependencies
if [ "$1" == "-r" ]; then
    REINSTALL_DEPS="-r"
fi

#######################################
###### Install ATOS dependencies ######
#######################################
echo "Installing ATOS dependencies..."
${REPO_DIR}/scripts/installation/install_deps.sh ${REPO_DIR} ${REINSTALL_DEPS}

if [ $? -ne 0 ]; then
    echo "Failed to install dependencies."
    exit 1
fi
########################################
###### Start installation of ATOS ######
########################################
if ! [ -z "$REINSTALL_DEPS" ]; then
    echo "ATOS installation skipped."
    exit 0
fi

echo "Installing ATOS..."
${REPO_DIR}/scripts/installation/install_atos.sh ${REPO_DIR}

if [ $? -ne 0 ]; then
    echo "Failed to install ATOS."
    exit 1
fi

echo "ATOS build and setup is complete. Please restart your terminal to complete the installation."
echo "Please see the documentation for further details: https://atos.readthedocs.io/en/latest/"
