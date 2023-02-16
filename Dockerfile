ARG ROS_DISTRO=humble
ARG FROM_IMAGE=osrf/ros:${ROS_DISTRO}-desktop-full-jammy
ARG OVERLAY_WS=/opt/ros/${ROS_DISTRO}/overlay_ws

FROM ${FROM_IMAGE} AS cacher

ENV DEBIAN_FRONTEND=noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
ENV CUSTOM_PATH=/../sourceinstall

WORKDIR /root/atos_ws


RUN sudo apt update \
    && sudo apt install -y libsystemd-dev \
    && sudo apt install -y libprotobuf-dev  \
    && sudo apt install -y protobuf-compiler \
    && sudo apt install -y libeigen3-dev \
    && sudo apt install -y nlohmann-json3-dev \
    && sudo apt install -y libpaho-mqtt-dev \
    && sudo apt install -y curl \
    && sudo apt install -y gnupg2 \
    && sudo apt install -y lsb-release \
    && sudo apt install -y mesa-common-dev 

RUN sudo apt install -y ros-${ROS_DISTRO}-geographic-msgs \
    && sudo apt install -y ros-${ROS_DISTRO}-geometry-msgs \
    && sudo apt install -y ros-${ROS_DISTRO}-std-msgs \
    && sudo apt install -y ros-${ROS_DISTRO}-std-srvs \
    && sudo apt install ros-${ROS_DISTRO}-desktop \
    && sudo apt install python3-colcon-common-extensions \
    && sudo apt install ros-${ROS_DISTRO}-nav-msgs  \
    && sudo apt install ros-${ROS_DISTRO}-geographic-msgs

#TODO fucking up the apt atm fix later
# RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg \
#     && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null 



RUN mkdir -p .$CUSTOM_PATH \
    && cd .$CUSTOM_PATH \
    && git clone https://github.com/OpenSimulationInterface/open-simulation-interface.git -b v3.4.0 \
    && cd open-simulation-interface \
    && mkdir -p build && cd build \
    && cmake .. && make \
    && sudo make install 

RUN sudo sh -c "echo '/usr/local/lib/osi3' > /etc/ld.so.conf.d/osi3.conf" \
    && sudo ldconfig

RUN cd .$CUSTOM_PATH \
    && git clone https://github.com/javedulu/ad-xolib.git \
    && cd ad-xolib \
    && git submodule update --init --recursive \
    && mkdir -p build && cd build \
    && cmake .. -DBUILD_EMBED_TARGETS=OFF && make \
    && sudo make install \
    && sudo ldconfig


RUN cd .$CUSTOM_PATH \
    && git clone https://github.com/esmini/esmini  \
    && cd esmini \
    && mkdir -p  build && cd build \
    && cmake .. && make \
    && sudo make install \
    && sudo cp ../bin/libesminiLib.so /usr/local/lib \
    && sudo cp ../bin/libesminiRMLib.so /usr/local/lib \
    && sudo mkdir -p /usr/local/include/esmini/ \
    && sudo cp ../EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp /usr/local/include/esmini/ \
    && sudo cp ../EnvironmentSimulator/Libraries/esminiRMLib/esminiRMLib.hpp /usr/local/include/esmini/ \
    && sudo ldconfig


RUN mkdir -p ./src/atos 

#  me being lazy can fix this another day
COPY . ./src/atos

# me being very lazy
RUN mv ./src/atos/atos_interfaces ./src


RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
   && colcon build 