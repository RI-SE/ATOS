ARG ROS_DISTRO=humble
ARG OS=jammy
ARG FROM_IMAGE=ros:${ROS_DISTRO}-ros-base-${OS}
ARG OVERLAY_WS=/opt/ros/${ROS_DISTRO}/overlay_ws

FROM ${FROM_IMAGE} AS cacher

ENV DEBIAN_FRONTEND=noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Install system libs
RUN \
    --mount=type=cache,target=/var/cache/apt \
    apt update && apt install -y \
    curl \
    libsystemd-dev \
    libprotobuf-dev \
    protobuf-compiler \
    libeigen3-dev \
    nlohmann-json3-dev \
    libpaho-mqtt-dev \
    gnupg2 \
    lsb-release \
    python3-pip

RUN pip install pyOpenSSL

RUN if [ "$OS" == "jammy" ]; then sudo apt install -y libpaho-mqtt-dev; fi
RUN if [ "$OS" == "foxy" ]; then sudo apt install -y ros-foxy-paho-mqtt-c; fi

ENV NODE_VERSION=16.13.0
RUN apt install -y curl
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.0/install.sh | bash
ENV NVM_DIR=/root/.nvm
RUN . "$NVM_DIR/nvm.sh" && nvm install ${NODE_VERSION}
RUN . "$NVM_DIR/nvm.sh" && nvm use v${NODE_VERSION}
RUN . "$NVM_DIR/nvm.sh" && nvm alias default v${NODE_VERSION}
ENV PATH="/root/.nvm/versions/node/v${NODE_VERSION}/bin/:${PATH}"
RUN node --version
RUN npm --version

# Install ROS2 libs
RUN \
    --mount=type=cache,target=/var/cache/apt \
    apt update && apt install -y \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-geographic-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-std-srvs \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-geographic-msgs \
    ros-${ROS_DISTRO}-foxglove-msgs \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-rosbridge-suite


ENV TEMP_SRC_PATH=/root/sourceinstall
RUN mkdir -p $TEMP_SRC_PATH
WORKDIR /root/sourceinstall
# Install OpenSimulationInterface
RUN cd $TEMP_SRC_PATH && \
    git clone https://github.com/OpenSimulationInterface/open-simulation-interface.git -b v3.4.0 && \
    cd open-simulation-interface && \
    mkdir -p build && cd build && \
    cmake .. && make -j4 && \
    sudo make install && \
    sudo sh -c "echo '/usr/local/lib/osi3' > /etc/ld.so.conf.d/osi3.conf" && \
    sudo ldconfig && \
    sudo rm -rf $TEMP_SRC_PATH/open-simulation-interface

# Install ad-xolib
RUN cd $TEMP_SRC_PATH && \
    git clone https://github.com/javedulu/ad-xolib.git && \
    cd ad-xolib && \
    git submodule update --init --recursive && \
    mkdir -p build && cd build && \
    cmake .. -DBUILD_EMBED_TARGETS=OFF && make -j4 && \
    sudo make install && \
    sudo ldconfig && \
    sudo rm -rf $TEMP_SRC_PATH/ad-xolib


# Install esmini
RUN cd $TEMP_SRC_PATH && \
    git clone https://github.com/esmini/esmini && \
    cd esmini && \
    mkdir -p build && cd build && \
    cmake .. && make -j4 && \
    sudo make install && \
    sudo cp ../bin/libesminiLib.so /usr/local/lib && \
    sudo cp ../bin/libesminiRMLib.so /usr/local/lib && \
    sudo mkdir -p /usr/local/include/esmini/ && \
    sudo cp ../EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp /usr/local/include/esmini/ && \
    sudo cp ../EnvironmentSimulator/Libraries/esminiRMLib/esminiRMLib.hpp /usr/local/include/esmini/ && \
    sudo ldconfig && \
    sudo rm -rf $TEMP_SRC_PATH/esmini



WORKDIR /root/atos_ws
# Setup workspace (.dockerignore is used to avoid copying unnecessary files)
RUN mkdir -p ./src/atos 
COPY . ./src/atos
RUN mv ./src/atos/atos_interfaces ./src

# Build
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    MAKEFLAGS=-j4 colcon build && \
    chmod +x /root/atos_ws/install/setup.sh && \
    rm -rf /root/atos_ws/build /root/atos_ws/src

RUN mkdir -p /root/.astazero/ATOS
COPY ./conf /root/.astazero/ATOS/

RUN echo "source /root/atos_ws/install/setup.sh" >> /root/.bashrc