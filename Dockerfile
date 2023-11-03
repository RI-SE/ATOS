ARG ROS_DISTRO=humble
ARG OS=jammy
ARG FROM_IMAGE=ros:${ROS_DISTRO}-ros-base-${OS}
ARG OVERLAY_WS=/opt/ros/${ROS_DISTRO}/overlay_ws

FROM ${FROM_IMAGE} AS cacher

ENV DEBIAN_FRONTEND=noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

ENV ROS_DISTRO=${ROS_DISTRO}
ENV REPO_DIR=/root/atos_git
# NODE_VERSION is also specified in the install_atos.sh script
ENV NODE_VERSION=16.20.0
ENV PATH="/root/.nvm/versions/node/v${NODE_VERSION}/bin/:${PATH}"
SHELL ["/bin/bash", "-c"]

WORKDIR /root/atos_git
COPY ./scripts/installation/ ./scripts/installation/
RUN --mount=type=cache,target=/var/cache/apt \ 
        ./scripts/installation/install_deps.sh ${REPO_DIR}
COPY . .
RUN ./scripts/installation/install_atos.sh ${REPO_DIR}
WORKDIR /root/atos_ws