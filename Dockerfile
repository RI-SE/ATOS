ARG ROS_DISTRO=humble
ARG OS=jammy
ARG FROM_IMAGE=osrf/ros:${ROS_DISTRO}-desktop-full-${OS}
ARG OVERLAY_WS=/opt/ros/${ROS_DISTRO}/overlay_ws

FROM ${FROM_IMAGE} AS cacher

ENV DEBIAN_FRONTEND=noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
ENV CUSTOM_PATH=/../sourceinstall

WORKDIR /root/atos_ws

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

# Install system libs
RUN \
    --mount=type=cache,target=/var/cache/apt \
    apt update && apt install -y \
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


# Install OpenSimulationInterface
RUN mkdir -p .$CUSTOM_PATH && \
    cd .$CUSTOM_PATH && \
    git clone https://github.com/OpenSimulationInterface/open-simulation-interface.git -b v3.4.0 && \
    cd open-simulation-interface && \
    mkdir -p build && cd build && \
    cmake .. && make && \
    sudo make install && \
    sudo sh -c "echo '/usr/local/lib/osi3' > /etc/ld.so.conf.d/osi3.conf" && \
    sudo ldconfig

# Install ad-xolib
RUN cd .$CUSTOM_PATH && \
    git clone https://github.com/javedulu/ad-xolib.git && \
    cd ad-xolib && \
    git submodule update --init --recursive && \
    mkdir -p build && cd build && \
    cmake .. -DBUILD_EMBED_TARGETS=OFF && make && \
    sudo make install && \
    sudo ldconfig


# Install esmini
RUN cd .$CUSTOM_PATH && \
    git clone https://github.com/esmini/esmini && \
    cd esmini && \
    mkdir -p build && cd build && \
    cmake .. && make && \
    sudo make install && \
    sudo cp ../bin/libesminiLib.so /usr/local/lib && \
    sudo cp ../bin/libesminiRMLib.so /usr/local/lib && \
    sudo mkdir -p /usr/local/include/esmini/ && \
    sudo cp ../EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp /usr/local/include/esmini/ && \
    sudo cp ../EnvironmentSimulator/Libraries/esminiRMLib/esminiRMLib.hpp /usr/local/include/esmini/ && \
    sudo ldconfig



# Setup workspace (.dockerignore is used to avoid copying unnecessary files)
RUN mkdir -p ./src/atos 
COPY . ./src/atos
RUN mv ./src/atos/atos_interfaces ./src

# Build
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && MAKEFLAGS=-j1 colcon build \
    && chmod +x /root/atos_ws/install/setup.sh 

RUN mkdir -p /root/.astazero/ATOS
COPY ./conf /root/.astazero/ATOS/

RUN echo "source /root/atos_ws/install/setup.sh" >> /root/.bashrc